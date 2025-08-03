/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Converter.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "Map.h"
#include "Associater.h"

#include "FeatureExtractorFactory.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "CorrelationMatcher.h"

// DEBUG
#include <map>
#include "CorrelationEdge.h"
#include "MapPoint.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

// ====================== HDF5 Key Point Output ======================== //
#include <array>
// ====================== HDF5 Key Point Output ======================== //

using namespace ::std;

namespace ORB_SLAM2 {

CorrelationMatcher Tracking::sMatcher;

// ====================== HDF5 Key Point Output ======================== //
void Tracking::InitLogFile()
{
  	if(!mbDoLog) return;

    std::string fname = "slam_log_" + std::to_string(std::time(nullptr)) + ".h5";
    mpLogFile = std::make_unique<HighFive::File>(
        fname,
        HighFive::File::Overwrite | HighFive::File::Create
    );
}

void Tracking::BuildMatchesWithLastPerChannel(int ch, std::vector<std::pair<int,int>>& vMatches) const
{
    const auto& CL = mLastFrame.Channels[ch];
    const auto& CC = mCurrentFrame.Channels[ch];

    if (CL.N == 0 || CC.N == 0) return;

    std::unordered_map<MapPoint*, int> mp2idxLast;
    for (int j = 0; j < CL.N; ++j)
        if (auto* p = CL.mvpMapPoints[j]; p && !CL.mvbOutlier[j])
            mp2idxLast[p] = j;

    for (int i = 0; i < CC.N; ++i)
        if (auto* p = CC.mvpMapPoints[i]; p && !CC.mvbOutlier[i])
            if (auto it = mp2idxLast.find(p); it != mp2idxLast.end())
                vMatches.emplace_back(i, it->second);
}


void Tracking::LogFrame()
{
    if(!mbDoLog) return;
    if(!mpLogFile) InitLogFile();

    const std::string gFrame = "/frames/" + std::to_string(mCurrentFrame.mnId);
    mpLogFile->createGroup(gFrame);

    for(int ch = 0; ch < Ntype; ++ch)                   // Channel Loop Recursive
    {
        const auto& C = mCurrentFrame.Channels[ch];
        if(C.N == 0) continue;

        const std::string g = gFrame + "/ch" + std::to_string(ch);
        mpLogFile->createGroup(g);

        /* 1) keypoints [N,7] */
        std::vector<std::array<float,7>> kpbuf;
        kpbuf.reserve(C.N);
        for(int i = 0; i < C.N; ++i){
            const cv::KeyPoint& kp = C.mvKeys[i];
            float mpid = (C.mvpMapPoints[i] && !C.mvbOutlier[i])
                        ? static_cast<float>(C.mvpMapPoints[i]->mnId)
                        : -1.f;
            kpbuf.push_back({kp.pt.x, kp.pt.y,
                             static_cast<float>(kp.octave),
                             kp.angle, kp.size, kp.response,
                             mpid});
        }
        mpLogFile->createDataSet<float>(g+"/keypoints",
                                        {kpbuf.size(),7}).write(kpbuf);

        /* 2) descriptors [N,32] */
        std::vector<std::array<uint8_t,32>> desc(C.N);
        for(int i=0;i<C.N;++i)
            std::memcpy(desc[i].data(),
                        C.mDescriptors.ptr<uint8_t>(i),32);
        mpLogFile->createDataSet<uint8_t>(g+"/descriptors",
                                          {desc.size(),32}).write(desc);

        /* 3) matches to last frame */
        std::vector<std::pair<int,int>> vMatches;
        if (mState == NOT_INITIALIZED) {
            const auto& vIni = mvIniMatches[ch];     // Initial mapping
            for (size_t idxIni = 0; idxIni < vIni.size(); ++idxIni) {
                int idxCur = vIni[idxIni];
                if(idxCur >= 0) vMatches.emplace_back(idxCur, static_cast<int>(idxIni));
            }
        } else {
            BuildMatchesWithLastPerChannel(ch, vMatches);
        }
        if (!vMatches.empty()) {
            std::vector<std::array<int,2>> mbuf;
            for(auto &p:vMatches) mbuf.push_back({p.first,p.second});
            mpLogFile->createDataSet<int>(g+"/matches_prev",
                                          {mbuf.size(),2}).write(mbuf);
            mpLogFile->getGroup(g)
                     .createAttribute<int>("prev_id",
                         HighFive::DataSpace(1))
                     .write(mCurrentFrame.mnId-1);
        }
    }
}

void Tracking::CloseLogFile(){
    if(mpLogFile){
        mpLogFile->flush();
        mpLogFile.reset();
    }
}
// ====================== HDF5 Key Point Output ======================== //


Tracking::Tracking(System *pSys, std::vector<FbowVocabulary *> pVoc, std::vector<FrameDrawer *> pFrameDrawer,
                   MapDrawer *pMapDrawer, Map *pMap, std::vector<KeyFrameDatabase *> pKFDB,
                   const string &strSettingPath, const int sensor, int Ntype)
    : mState(NO_IMAGES_YET), 
      mSensor(sensor),
      mCurrentFrame(Ntype),
      mLastFrame(Ntype),
      mInitialFrame(Ntype),
      mbOnlyTracking(false),
      mbVO(false),
      mpSystem(pSys),
      mpViewer(NULL),
      mpInitializer(static_cast<Initializer *>(NULL)), 
      mpFrameDrawer(pFrameDrawer), 
      mpMapDrawer(pMapDrawer),
      mpMap(pMap), 
      mnLastRelocFrameId(0),
      mpVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      Ntype(Ntype) {

  /*
  // Initlize vocabulary vector
  mpVocabulary.resize(Ntype);
  mpKeyFrameDB.resize(Ntype);
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    mpVocabulary[Ftype] = pVoc[Ftype];
    mpKeyFrameDB[Ftype] = pKFDB[Ftype];
  }
  */
  mvbPrevMatched.resize(Ntype);
  mvIniMatches.resize(Ntype);
  mvIniP3D.resize(Ntype);
  
  // Load camera parameters from settings file
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  float fps = fSettings["Camera.fps"];
  if (fps == 0)
    fps = 30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  // Initial pose is identity
  mLastPose = cv::Mat::eye(4, 4, CV_32F);

  cout << endl << "Camera Parameters: " << endl;
  cout << "- fx: " << fx << endl;
  cout << "- fy: " << fy << endl;
  cout << "- cx: " << cx << endl;
  cout << "- cy: " << cy << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows == 5)
    cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;
  cout << "- fps: " << fps << endl;

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
    cout << "- color order: BGR (ignored if grayscale)" << endl;

  /*

  // Load ORB parameters

  int nFeatures = fSettings["ORBextractor.nFeatures"];
  float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
  int nLevels = fSettings["ORBextractor.nLevels"];
  int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
  int fMinThFAST = fSettings["ORBextractor.minThFAST"];

  // create left extractor
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    mpFeatureExtractorLeft[Ftype] = createFeatureExtractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, Ftype);

  if (sensor == System::STEREO){
    // create right extractor
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      mpFeatureExtractorRight[Ftype] = createFeatureExtractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, Ftype);
  }

  if (sensor == System::MONOCULAR){
    // create init extractor
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      mpIniFeatureExtractor[Ftype] = createFeatureExtractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, Ftype);
  }

  cout << endl << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

  */

  // Config loading and initialization for each feature extractor
  mpFeatureExtractorLeft.resize(Ntype);
  mpFeatureExtractorRight.resize(Ntype);
  mpIniFeatureExtractor.resize(Ntype);

  // Read the number of extractor and their corrisponding name
  std::vector<std::string> extractor_names;
  cv::FileNode extractor_list = fSettings["Extractors"];
  for (auto it = extractor_list.begin(); it != extractor_list.end(); ++it)
      extractor_names.push_back((std::string)*it);

  // Associator: Resize/Initialize TH vectors
  Associater::mvTH_LOW.resize(Ntype);
  Associater::mvTH_HIGH.resize(Ntype);

  // Read and initialize parameters for each extractor
  for (int i = 0; i < Ntype; ++i) {
    std::string& name = extractor_names[i];
    const cv::FileNode& extractor_config = fSettings[name];

    Associater::mvTH_LOW[i] = static_cast<float>(extractor_config["TH_LOW"]);
    Associater::mvTH_HIGH[i] = static_cast<float>(extractor_config["TH_HIGH"]);

    mpFeatureExtractorLeft[i] = FeatureExtractorFactory::Instance().Create(name, extractor_config, false);

    if (sensor == System::STEREO)
      mpFeatureExtractorRight[i] = FeatureExtractorFactory::Instance().Create(name, extractor_config, false);

    if (sensor == System::MONOCULAR)
      mpIniFeatureExtractor[i] = FeatureExtractorFactory::Instance().Create(name, extractor_config, true);

    cout << endl << "[Feature " + std::to_string(i) + "] " + name + " Extractor Parameters: " << endl;
	cout << " ( TH_LOW: " << static_cast<float>(extractor_config["TH_LOW"]) <<
	", TH_HIGH: " << static_cast<float>(extractor_config["TH_HIGH"]) << " )" << endl;
    mpFeatureExtractorLeft[i]->InfoConfigs();
  }

  if (sensor == System::STEREO || sensor == System::RGBD) {
    mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
  }

  if (sensor == System::RGBD) {
    mDepthMapFactor = fSettings["DepthMapFactor"];
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer) { mpViewer = pViewer; }

// Stereo
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;

  if (mImGray.channels() == 3) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
    }
  } else if (mImGray.channels() == 4) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
    }
  }

  //if (getenv("FULL_RESOLUTION") == nullptr) {
  //  cv::resize(mImGray, mImGray, cv::Size(320, 240));
  //  cv::resize(imGrayRight, imGrayRight, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
  //}

  mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpFeatureExtractorLeft, mpFeatureExtractorRight,
                        mpVocabulary, mK, mDistCoef, mbf, mThDepth, Ntype);

  Track();

  return mCurrentFrame.mTcw.clone();
}

// RGBD
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
  mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  //if (getenv("FULL_RESOLUTION") == nullptr) {
  //  cv::resize(mImGray, mImGray, cv::Size(320, 240));
  //  cv::resize(imDepth, imDepth, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
  //}

  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpFeatureExtractorLeft, mpVocabulary, mK, mDistCoef, mbf, mThDepth, Ntype);

  Track();

  return mCurrentFrame.mTcw.clone();
}

// MONO
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
  mImGray = im;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  //if (getenv("FULL_RESOLUTION") == nullptr) {
  //  cv::resize(mImGray, mImGray, cv::Size(320, 240));
  //}

  if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGray, timestamp, mpIniFeatureExtractor, mpVocabulary, mK, mDistCoef, mbf, mThDepth, Ntype);
  else
    mCurrentFrame = Frame(mImGray, timestamp, mpFeatureExtractorLeft, mpVocabulary, mK, mDistCoef, mbf, mThDepth, Ntype);

  Track();

  return mCurrentFrame.mTcw.clone();
}

void Tracking::Track() {
  if (mState == NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  if (mState == LOST) 
    cout << "The lost frame ID: " << mCurrentFrame.mnId << endl;

  mLastProcessedState = mState;

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

  if (mState == NOT_INITIALIZED) {
    if (mSensor == System::STEREO || mSensor == System::RGBD) {
      if (mCurrentFrame.mnId == 0) {
        mtStart = clock();
      }

      StereoInitializationMultiChannels();

      if (mState == OK) {
        printf("Tracking Initlized in %.2fs\n", (double)(clock() - mtStart) / CLOCKS_PER_SEC);
        cout << "The initlized frame ID: " << mCurrentFrame.mnId << endl;
        mInitlizedID = mCurrentFrame.mnId;
      }
    } else {
      if (mCurrentFrame.mnId == 0) {
        mtStart = clock();
      }

      MonocularInitializationMultiChannels();

      if (mState == OK) {
        printf("Tracking Initlized in %.2fs\n", (double)(clock() - mtStart) / CLOCKS_PER_SEC);
        cout << "The initlized frame ID: " << mCurrentFrame.mnId << endl;
        mInitlizedID = mCurrentFrame.mnId;
      }
    }

// ====================== HDF5 Key Point Output ======================== //
    /* ---------- HDF5 Log Hooker begin ---------- */
    LogFrame();
    /* ---------- HDF5 Log Hooker end ------------ */
// ====================== HDF5 Key Point Output ======================== //

    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      mpFrameDrawer[Ftype]->Update(this);

    if (mState != OK)
      return;
  } else {

    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (!mbOnlyTracking) {
      // Local Mapping is activated. This is the normal behaviour, unless you explicitly activate the "only tracking" mode.
      if (mState == OK) {
        // Local Mapping might have changed some MapPoints tracked in last frame
        for (int Ftype = 0; Ftype < Ntype; Ftype++)
          CheckReplacedInLastFrame(Ftype);

        if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
           
          bOK = TrackReferenceKeyFrameMultiChannels();

        } else {
          bOK = TrackWithMotionModelMultiChannels();

          if (!bOK) {
            bOK = TrackReferenceKeyFrameMultiChannels();
          }
        }
      } else {
        for (int Ftype = 0; Ftype < Ntype; Ftype++) {
          bOK = Relocalization(Ftype);
          if (bOK == true) {
            break;
          }
        }
      }
    } else {
      // Localization Mode: Local Mapping is deactivated
      if (mState == LOST) {
        for (int Ftype = 0; Ftype < Ntype; Ftype++) {
          bOK = Relocalization(Ftype);
          if (bOK == true) {
            break;
          }
        }
      } else {
        if (!mbVO) {
          // In last frame we tracked enough MapPoints in the map
          if (!mVelocity.empty()) {
            bOK = TrackWithMotionModelMultiChannels();
          } else {
            bOK = TrackReferenceKeyFrameMultiChannels();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing relocalization. If relocalization is sucessfull we choose that
          // solution, otherwise we retain the "visual odometry" solution.
          bool bOKMM = false;
          bool bOKReloc = false;
          vector<vector<MapPoint *>> vpMPsMM;
          vector<vector<bool>> vbOutMM;
          vpMPsMM.resize(Ntype);
          vbOutMM.resize(Ntype);
          cv::Mat TcwMM;
          if (!mVelocity.empty()) {
            bOKMM = TrackWithMotionModelMultiChannels();
            for (int Ftype = 0; Ftype < Ntype; Ftype++) {
              vpMPsMM[Ftype] = mCurrentFrame.Channels[Ftype].mvpMapPoints;
              vbOutMM[Ftype] = mCurrentFrame.Channels[Ftype].mvbOutlier;
            }
            TcwMM = mCurrentFrame.mTcw.clone();
          }

          for (int Ftype = 0; Ftype < Ntype; Ftype++) {
            bOKReloc = Relocalization(Ftype);
            if (bOKReloc == true) {
              break;
            }
          }
            
          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            for (int Ftype = 0; Ftype < Ntype; Ftype++) {
              mCurrentFrame.Channels[Ftype].mvpMapPoints = vpMPsMM[Ftype];
              mCurrentFrame.Channels[Ftype].mvbOutlier = vbOutMM[Ftype];
            }

            if (mbVO) {
              for (int Ftype = 0; Ftype < Ntype; Ftype++) {
                for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
                  if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i] && !mCurrentFrame.Channels[Ftype].mvbOutlier[i]) {
                    mCurrentFrame.Channels[Ftype].mvpMapPoints[i]->IncreaseFound();
                  }
                }
              }
            }
          } else if (bOKReloc) {
            mbVO = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (!mbOnlyTracking) {


      if (bOK) {

        bOK = TrackLocalMapMultiChannels();

      }

      if (!bOK) {
        mCurrentFrame.SetPose(mLastPose);
        bOK = TrackLocalMapMultiChannels();
      }
    } else {
      // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve a local map and therefore we do not perform
      // TrackLocalMap(). Once the system relocalizes the camera we will use the local map again.
      if (bOK && !mbVO)
        bOK = TrackLocalMapMultiChannels();
    }

    if (bOK)
      mState = OK;
    else
      mState = LOST;

// ====================== HDF5 Key Point Output ======================== //
    /* ---------- HDF5 Log Hooker begin ---------- */
    LogFrame();
    /* ---------- HDF5 Log Hooker end ------------ */
// ====================== HDF5 Key Point Output ======================== //

    // Update drawer
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      mpFrameDrawer[Ftype]->Update(this);

    // If tracking were good, check if we insert a keyframe
    if (bOK) {
      // Update motion model
      if (!mLastFrame.mTcw.empty()) {
        cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
        mVelocity = mCurrentFrame.mTcw * LastTwc;
      } else
        mVelocity = cv::Mat();

      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

      // Clean VO matches
      for (int Ftype = 0; Ftype < Ntype; Ftype++) {
        for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
        MapPoint *pMP = mCurrentFrame.Channels[Ftype].mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            mCurrentFrame.Channels[Ftype].mvbOutlier[i] = false;
            mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          }
        }
      }

      // -------------------- Correlation ---------------------- //
      // Correlation Matching
      const float th_px = 2.0f;  // Threshold

      for (int a = 0; a < Ntype; ++a) {
        for (int b = a + 1; b < Ntype; ++b) {
            float cRI = sMatcher.BuildEdges(mCurrentFrame, a, b, th_px, 5);
            // std::cout << cRI << std::endl;
        }
      }


      /*
      // Debug Logging
      std::map<int, size_t> counterHist;
      size_t countAbove5 = 0;
      const auto& vpMapPoints = mpMap->GetAllMapPoints();
      for (ORB_SLAM2::MapPoint* p : vpMapPoints) {
        if (!p || p->isBad()) continue;

        for (const auto& kv : p->mAdjEdges) {
          const auto& edge = kv.second;
          if (edge->pA != p) continue;
          int c = edge->counter.load();
          counterHist[c]++;
          if (c > 5)
            ++countAbove5;
        }
      }
      std::cout << "[TRACK STAT] Edges with counter > 5 across map: " << countAbove5 << std::endl;
      std::cout << "[TRACK STAT] Edge Counter Histogram:" << std::endl;
      for (const auto& kv : counterHist) {
        std::cout << "  counter = " << kv.first << " → " << kv.second << " edges" << std::endl;
      }
      */
      // -------------------- Correlation ---------------------- //


      // Delete temporal MapPoints
      for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrameMultiChannels()) {
        CreateNewKeyFrameMultiChannels();
      } // TO-DO Multi Channels ??
        

      // We allow points with high innovation (considererd outliers by the Huber Function) pass to the new keyframe, so that bundle adjustment will
      // finally decide if they are outliers or not. We don't want next frame to estimate its position with those points so we discard them in the
      // frame.
      for (int Ftype = 0; Ftype < Ntype; Ftype++) {
        for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
          if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i] && mCurrentFrame.Channels[Ftype].mvbOutlier[i])
            mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }
      }

      mLastPose = mCurrentFrame.mTcw.clone();
    }

    // Reset if the camera get lost soon after initialization
    if (mState == LOST) {
      if (mpMap->KeyFramesInMap() <= 5) {
        cout << "Track lost soon after initialisation, reseting..." << endl;
        mpSystem->Reset();
        return;
      }
      // Uncomment this if reseting is needed on the fly
      // cout << "Track lost, reseting..." << endl;
      // mpSystem->Reset();
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  // Store frame pose information to retrieve the complete camera trajectory afterwards.
  if (!mCurrentFrame.mTcw.empty()) {
    cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    mlbLost.push_back(mState == LOST);
  } else {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState == LOST);
  }
}

void Tracking::StereoInitialization(const int Ftype) {
  if (mCurrentFrame.Channels[Ftype].N > 50) {
    // Set Frame pose to the origin
    mCurrentFrame.SetPose(mLastPose);

    int nGood = 0;
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      float z = mCurrentFrame.Channels[Ftype].mvDepth[i];
      if (z > 0) {
        nGood++;
      }
    }

    if (nGood < 50) {
      cout << "Cannot create new map with only " << nGood << " points" << endl;
      return;
    }

    // Create KeyFrame
    KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, Ntype);

    // Insert KeyFrame in the map
    mpMap->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      float z = mCurrentFrame.Channels[Ftype].mvDepth[i];
      if (z > 0) {
        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i, Ftype);
        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap, Ftype);
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i, Ftype);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpMap->AddMapPoint(pNewMP);

        mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = pNewMP;
      }
    }

    cout << "New map created with " << mpMap->MapPointsInMap(Ftype) << " points"
         << endl;

    mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

    mState = OK;
  }
}

// void Tracking::MonocularInitialization(const int Ftype) {

//   if (!mpInitializer) {

//     // Set Reference Frame
//     if (mCurrentFrame.Channels[Ftype].mvKeys.size() > 100) {
      
//       mInitialFrame = Frame(mCurrentFrame);
//       mLastFrame = Frame(mCurrentFrame);
      
//       mvbPrevMatched[Ftype].resize(mCurrentFrame.Channels[Ftype].mvKeysUn.size());
      
//       for (size_t i = 0; i < mCurrentFrame.Channels[Ftype].mvKeysUn.size(); i++)
//         mvbPrevMatched[Ftype][i] = mCurrentFrame.Channels[Ftype].mvKeysUn[i].pt;

//       if (mpInitializer)
//         delete mpInitializer;

//       mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

//       fill(mvIniMatches[Ftype].begin(), mvIniMatches[Ftype].end(), -1);

//       return;
//     }
//   } else {
//     // Try to initialize
//     if ((int)mCurrentFrame.Channels[Ftype].mvKeys.size() <= 100) {
//       delete mpInitializer;
//       mpInitializer = static_cast<Initializer *>(NULL);
//       fill(mvIniMatches[Ftype].begin(), mvIniMatches[Ftype].end(), -1);
//       return;
//     }

//     // Find correspondences
//     Associater associater(0.9, true);

//     int nmatches = associater.SearchForInitialization(Ftype, mInitialFrame, mCurrentFrame, mvbPrevMatched[Ftype], mvIniMatches[Ftype], 100);

//     // Check if there are enough correspondences
//     if (nmatches < 100) {
//       delete mpInitializer;
//       mpInitializer = static_cast<Initializer *>(NULL);
//       return;
//     }

//     cv::Mat Rcw;                 // Current Camera Rotation
//     cv::Mat tcw;                 // Current Camera Translation
//     vector<vector<bool>> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
//     vbTriangulated.resize(Ntype);
    
//     if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches[Ftype], Rcw, tcw, mvIniP3D[Ftype], vbTriangulated[Ftype])) {
//       for (size_t i = 0, iend = mvIniMatches[Ftype].size(); i < iend; i++) {
//         if (mvIniMatches[Ftype][i] >= 0 && !vbTriangulated[Ftype][i]) {
//           mvIniMatches[Ftype][i] = -1;
//           nmatches--;
//         }
//       }

//       // Set Frame Poses
//       mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
//       cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
//       Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
//       tcw.copyTo(Tcw.rowRange(0, 3).col(3));
//       mCurrentFrame.SetPose(Tcw);

//       CreateInitialMapMonocular(Ftype);
//     }
//   }
// }

// ComputeSceneMedianDepth should have ftype
// void Tracking::CreateInitialMapMonocular(const int Ftype) {
//   // Create KeyFrames
//   KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
//   KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

//   pKFini->ComputeBoW(Ftype);
//   pKFcur->ComputeBoW(Ftype);

//   // Insert KFs in the map
//   mpMap->AddKeyFrame(pKFini);
//   mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
//   for (size_t i = 0; i < mvIniMatches[Ftype].size(); i++) {
//     if (mvIniMatches[Ftype][i] < 0)
//       continue;

//     // Create MapPoint.
//     cv::Mat worldPos(mvIniP3D[Ftype][i]);

//     MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap, Ftype);

//     pKFini->AddMapPoint(pMP, i, Ftype);
//     pKFcur->AddMapPoint(pMP, mvIniMatches[Ftype][i], Ftype);

//     pMP->AddObservation(pKFini, i);
//     pMP->AddObservation(pKFcur, mvIniMatches[Ftype][i]);

//     pMP->ComputeDistinctiveDescriptors();
//     pMP->UpdateNormalAndDepth();

//     // Fill Current Frame structure
//     mCurrentFrame.Channels[Ftype].mvpMapPoints[mvIniMatches[Ftype][i]] = pMP;
//     mCurrentFrame.Channels[Ftype].mvbOutlier[mvIniMatches[Ftype][i]] = false;

//     // Add to Map
//     mpMap->AddMapPoint(pMP);
//   }

//   // Update Connections
//   pKFini->UpdateConnectionsMultiChannels(); //TO-DO may ahve problems ? use UpdateConnectionsMultiChannels()
//   pKFcur->UpdateConnectionsMultiChannels(); // The function will only initlize one channel, but update connection of two channels

//   // Bundle Adjustment
//   cout << "New Map created with " << mpMap->MapPointsInMap() << " points"
//        << endl;

//   Optimizer::GlobalBundleAdjustemnt(mpMap, 20); //TO-DO ??

//   // Set median depth to 1
//   float medianDepth = pKFini->ComputeSceneMedianDepth(2, Ftype);
//   float invMedianDepth = 1.0f / medianDepth;

//   if (medianDepth < 0 || pKFcur->TrackedMapPoints(1, Ftype) < 100) {
//     cout << "Wrong initialization, reseting..." << endl;
//     Reset();
//     return;
//   }

//   // Scale initial baseline
//   cv::Mat Tc2w = pKFcur->GetPose();
//   Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
//   pKFcur->SetPose(Tc2w);

//   // Scale points
//   vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches(Ftype);
//   for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
//     if (vpAllMapPoints[iMP]) {
//       MapPoint *pMP = vpAllMapPoints[iMP];
//       pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
//     }
//   }

//   mpLocalMapper->InsertKeyFrame(pKFini);
//   mpLocalMapper->InsertKeyFrame(pKFcur);

//   mCurrentFrame.SetPose(pKFcur->GetPose());
//   mnLastKeyFrameId = mCurrentFrame.mnId;
//   mpLastKeyFrame = pKFcur;

//   mvpLocalKeyFrames.push_back(pKFcur);
//   mvpLocalKeyFrames.push_back(pKFini);
//   mvpLocalMapPoints = mpMap->GetAllMapPoints();
//   mpReferenceKF = pKFcur;
//   mCurrentFrame.mpReferenceKF = pKFcur;

//   mLastFrame = Frame(mCurrentFrame);

//   mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

//   mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

//   mpMap->mvpKeyFrameOrigins.push_back(pKFini);

//   mState = OK;
// }

void Tracking::Reset() {

  cout << "System Reseting" << endl;
  if (mpViewer) {
    mpViewer->RequestStop();
    while (!mpViewer->isStopped())
      this_thread::sleep_for(chrono::microseconds(3000));
  }

  // Reset Local Mapping
  cout << "Reseting Local Mapper...";
  mpLocalMapper->RequestReset();
  cout << " done" << endl;

  // Reset Loop Closing
  cout << "Reseting Loop Closing...";
  mpLoopClosing->RequestReset();
  cout << " done" << endl;

  // Clear BoW Database
  cout << "Reseting Database...";
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    mpKeyFrameDB[Ftype]->clear();
  cout << " done" << endl;

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;


  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = static_cast<Initializer *>(NULL);
  }
  

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();

  if (mpViewer)
    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) { mbOnlyTracking = flag; }

//////////////////////////////////Rewrite/////////////////////////////////

void Tracking::StereoInitializationMultiChannels() {

  // step 1 : the number of both channels should be greater than 50
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    if (mCurrentFrame.Channels[Ftype].N <= 50)
      return;

  // step 2 : Set Frame pose to the origin
  mCurrentFrame.SetPose(mLastPose);

  // step 3 : the number of good points should be greater than 50 
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    int nGood = 0;
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      float z = mCurrentFrame.Channels[Ftype].mvDepth[i];
      if (z > 0) {
        nGood++;
      }
    }

    if (nGood < 50) {
      cout << "Cannot create new map with only " << nGood << " points for Channels :" << Ftype << endl;
      return;
    }
  }

  // step 4 : Create KeyFrame
  KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, Ntype);

  // step 5 : Insert KeyFrame in the map
  mpMap->AddKeyFrame(pKFini);

  // step 6 : Create MapPoints and asscoiate to KeyFrame
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      float z = mCurrentFrame.Channels[Ftype].mvDepth[i];
      if (z > 0) {
        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i, Ftype);
        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap, Ftype);
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i, Ftype);
        pNewMP->ComputeDistinctiveDescriptors(); 
        pNewMP->UpdateNormalAndDepth(); 
        mpMap->AddMapPoint(pNewMP);

        mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = pNewMP;
      }
    }
  }

  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    cout << "New map created with " << mpMap->MapPointsInMap(Ftype) << " points for " << Ftype << " Feature Type " << endl;

  mpLocalMapper->InsertKeyFrame(pKFini);

  mLastFrame = Frame(mCurrentFrame);
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFini;

  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFini;
  mCurrentFrame.mpReferenceKF = pKFini;

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

  mState = OK;
}

void Tracking::MonocularInitializationMultiChannels() {
  
  if (!mpInitializer) {

    // Sum all mappoints
    int nKeysSum = 0;
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      nKeysSum += mCurrentFrame.Channels[Ftype].mvKeys.size();

    // Set Reference Frame
    if (nKeysSum > 100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);

      for (int Ftype = 0; Ftype < Ntype; Ftype++) 
        mvbPrevMatched[Ftype].resize(mCurrentFrame.Channels[Ftype].mvKeysUn.size());
      
      for (int Ftype = 0; Ftype < Ntype; Ftype++)
        for (size_t i = 0; i < mCurrentFrame.Channels[Ftype].mvKeysUn.size(); i++) 
          mvbPrevMatched[Ftype][i] = mCurrentFrame.Channels[Ftype].mvKeysUn[i].pt;
  

      if (mpInitializer) 
          delete mpInitializer;

      mpInitializer = new Initializer(mCurrentFrame, Ntype, 1.0, 200);
      
      for (int Ftype = 0; Ftype < Ntype; Ftype++)
        fill(mvIniMatches[Ftype].begin(), mvIniMatches[Ftype].end(), -1);
      
      return;
    }
  } else {
    // Sum numbers of keys
    int nKeysSum = 0;
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      nKeysSum += mCurrentFrame.Channels[Ftype].mvKeys.size();

    if (nKeysSum <= 100) {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
        for (int Ftype = 0; Ftype < Ntype; Ftype++)
          fill(mvIniMatches[Ftype].begin(), mvIniMatches[Ftype].end(), -1);
        return; 
    }

    // Find correspondences
    Associater associater(0.9, true);
    
    int nmatches[Ntype];
    for (int Ftype = 0; Ftype < Ntype; Ftype++) //TO-DO Multi Channels 
      nmatches[Ftype] = associater.SearchForInitialization(Ftype, mInitialFrame, mCurrentFrame, mvbPrevMatched[Ftype], mvIniMatches[Ftype], 100);

    int nmatchesSum = 0;
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      nmatchesSum += nmatches[Ftype];
    
    // Check if there are enough correspondences
    if (nmatchesSum < 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer *>(NULL);
      return;
    }

    cv::Mat Rcw;                 // Current Camera Rotation
    cv::Mat tcw;                 // Current Camera Translation
    vector<vector<bool>> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    vbTriangulated.resize(Ntype);

    // vector<bool> tryInit;
    // tryInit.resize(Ntype);
  
    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
      
      for (int Ftype = 0; Ftype < Ntype; Ftype++) {
        for (size_t i = 0, iend = mvIniMatches[Ftype].size(); i < iend; i++) {
          if (mvIniMatches[Ftype][i] >= 0 && !vbTriangulated[Ftype][i]) {
            mvIniMatches[Ftype][i] = -1;
            nmatches[Ftype]--;
          }
        }
      }



      // Set Frame Poses
      mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocularMultiChannels();
    }
  }
}


void Tracking::CreateInitialMapMonocularMultiChannels() {
  // Create KeyFrames
  KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB, Ntype);
  KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, Ntype);

  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    pKFini->ComputeBoW(Ftype);
    pKFcur->ComputeBoW(Ftype);
  }

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (size_t i = 0; i < mvIniMatches[Ftype].size(); i++) {
      if (mvIniMatches[Ftype][i] < 0)
        continue;

      // Create MapPoint.
      cv::Mat worldPos(mvIniP3D[Ftype][i]);

      MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap, Ftype);

      pKFini->AddMapPoint(pMP, i, Ftype);
      pKFcur->AddMapPoint(pMP, mvIniMatches[Ftype][i], Ftype);

      pMP->AddObservation(pKFini, i);
      pMP->AddObservation(pKFcur, mvIniMatches[Ftype][i]);


      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();

      // Fill Current Frame structure
      mCurrentFrame.Channels[Ftype].mvpMapPoints[mvIniMatches[Ftype][i]] = pMP;
      mCurrentFrame.Channels[Ftype].mvbOutlier[mvIniMatches[Ftype][i]] = false;

      // Add to Map
      mpMap->AddMapPoint(pMP);
    }
  }

  // Update Connections
  pKFini->UpdateConnectionsMultiChannels(); //TO-DO may ahve problems ? use UpdateConnectionsMultiChannels()
  pKFcur->UpdateConnectionsMultiChannels(); // The function will only initlize one channel, but update connection of two channels

  // Bundle Adjustment
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    cout << "New map created with " << mpMap->MapPointsInMap(Ftype) << " points for " << Ftype << " Feature Type " << endl;

  Optimizer::GlobalBundleAdjustemnt(mpMap, 20); 

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f / medianDepth;

  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
    cout << "Wrong initialization, reseting..." << endl;
    Reset();
    return;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->GetPose();
  Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches(Ftype);
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
      if (vpAllMapPoints[iMP]) {
        MapPoint *pMP = vpAllMapPoints[iMP];
        pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      }
    }
  }


  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  mLastFrame = Frame(mCurrentFrame);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame(const int Ftype) {

  for (int i = 0; i < mLastFrame.Channels[Ftype].N; i++) {
    MapPoint *pMP = mLastFrame.Channels[Ftype].mvpMapPoints[i];

    if (pMP) {
      MapPoint *pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.Channels[Ftype].mvpMapPoints[i] = pRep;
      }
    }
  }
}

void Tracking::UpdateLastFrame(const int Ftype) {
  // Update pose according to reference keyframe
  KeyFrame *pRef = mLastFrame.mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame.SetPose(Tlr * pRef->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
    return;

  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  vector<pair<float, int>> vDepthIdx;
  vDepthIdx.reserve(mLastFrame.Channels[Ftype].N);
  for (int i = 0; i < mLastFrame.Channels[Ftype].N; i++) {
    float z = mLastFrame.Channels[Ftype].mvDepth[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty())
    return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth). If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    bool bCreateNew = false;

    MapPoint *pMP = mLastFrame.Channels[Ftype].mvpMapPoints[i];
    if (!pMP)
      bCreateNew = true;
    else if (pMP->Observations() < 1) {
      bCreateNew = true;
    }

    if (bCreateNew) {
      cv::Mat x3D = mLastFrame.UnprojectStereo(i, Ftype);
      MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i, Ftype);

      mLastFrame.Channels[Ftype].mvpMapPoints[i] = pNewMP;

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
      break;
  }
}

bool Tracking::TrackWithMotionModelMultiChannels() {
  Associater associater(0.9, true);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  for (int Ftype = 0; Ftype < Ntype; Ftype++) 
    UpdateLastFrame(Ftype);

  mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    fill(mCurrentFrame.Channels[Ftype].mvpMapPoints.begin(), mCurrentFrame.Channels[Ftype].mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

  // Project points seen in previous frame
  // int th;
  // if(mSensor!=System::STEREO)
  //     th=15;
  // else
  //     th=7;

  int th = 15;
  int nmatches[Ntype];
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    nmatches[Ftype] = associater.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR, Ftype);

    // nmatches[Ftype] = associater.SearchByNN(mCurrentFrame,mLastFrame, Ftype);

  // sum of all matches
  int nmatchesSum = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    nmatchesSum += nmatches[Ftype];

  if (nmatchesSum < 20) {
    for (int Ftype = 0; Ftype < Ntype; Ftype++) {
      fill(mCurrentFrame.Channels[Ftype].mvpMapPoints.begin(), mCurrentFrame.Channels[Ftype].mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
      nmatches[Ftype] = associater.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR, Ftype);

      // nmatches[Ftype] = associater.SearchByNN(mCurrentFrame,mLastFrame, Ftype);
    }

    // sum of all matches
    nmatchesSum = 0;
    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      nmatchesSum += nmatches[Ftype];
  }
  
  if (nmatchesSum < 20)
    return false;

  // Optimize frame pose with all matches
  Optimizer::PoseOptimizationMultiChannels(&mCurrentFrame);
  //Optimizer::PoseOptimizationMultiChannels(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]) {
        if (mCurrentFrame.Channels[Ftype].mvbOutlier[i]) {
          MapPoint *pMP = mCurrentFrame.Channels[Ftype].mvpMapPoints[i];

          mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          mCurrentFrame.Channels[Ftype].mvbOutlier[i] = false;
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame.mnId;
          nmatchesSum--;
        } else if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]->Observations() > 0)
          nmatchesMap++;
      }
    }
  }

  if (mbOnlyTracking) {
    mbVO = nmatchesMap < 10;
    return nmatchesSum > 20;
  }

  return nmatchesMap >= 10;
}

bool Tracking::TrackReferenceKeyFrameMultiChannels() {
  // Compute Bag of Words vector
  for (int Ftype = 0; Ftype < Ntype; Ftype++) 
    mCurrentFrame.ComputeBoW(Ftype);
  
  // Assocaiter
  Associater associater(0.7, true);
  vector<vector<MapPoint *>> vvpMapPointMatches;
  vvpMapPointMatches.resize(Ntype);

  int nmatches[Ntype];
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    nmatches[Ftype] = associater.SearchByBoW(mpReferenceKF, mCurrentFrame, vvpMapPointMatches[Ftype], Ftype); 

    // nmatches[Ftype] = associater.SearchByNN(mpReferenceKF, mCurrentFrame, vvpMapPointMatches[Ftype], Ftype);

  // sum of all matches
  int nmatchesSum = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    nmatchesSum += nmatches[Ftype];
  
  if (nmatchesSum < 15)
    return false;

  for (int Ftype = 0; Ftype < Ntype; Ftype++) 
    mCurrentFrame.Channels[Ftype].mvpMapPoints = vvpMapPointMatches[Ftype];
  
  mCurrentFrame.SetPose(mLastFrame.mTcw);

  Optimizer::PoseOptimizationMultiChannels(&mCurrentFrame);
  //Optimizer::PoseOptimizationMultiChannels(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]) {
        if (mCurrentFrame.Channels[Ftype].mvbOutlier[i]) {
          MapPoint *pMP = mCurrentFrame.Channels[Ftype].mvpMapPoints[i];

          mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          mCurrentFrame.Channels[Ftype].mvbOutlier[i] = false;
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame.mnId;
          nmatchesSum--;
        } else if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]->Observations() > 0)
          nmatchesMap++;
      }
    }
  }

  return nmatchesMap >= 10;
}

bool Tracking::Relocalization(const int Ftype) {
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW(Ftype);

  // Relocalization is performed when tracking is lost. Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
  vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB[Ftype]->DetectRelocalizationCandidates(&mCurrentFrame, Ftype); // Multi Channels ??

  if (vpCandidateKFs.empty())
    return false;

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate. If enough matches are found we setup a PnP solver
  Associater associater(0.75, true);

  vector<PnPsolver *> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<vector<MapPoint *>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame *pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = associater.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i], Ftype);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i], Ftype);
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
        vpPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  Associater associater2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
        continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      PnPsolver *pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
        Tcw.copyTo(mCurrentFrame.mTcw);

        set<MapPoint *> sFound;

        const int np = vbInliers.size();

        for (int j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame.Channels[Ftype].mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.Channels[Ftype].mvpMapPoints[j] = NULL;
        }

        int nGood = Optimizer::PoseOptimization(&mCurrentFrame, Ftype);

        if (nGood < 10)
          continue;

        for (int io = 0; io < mCurrentFrame.Channels[Ftype].N; io++)
          if (mCurrentFrame.Channels[Ftype].mvbOutlier[io])
            mCurrentFrame.Channels[Ftype].mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

        // If few inliers, search by projection in a coarse window and optimize again
        if (nGood < 50) {
          int nadditional = associater2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100, Ftype);

          if (nadditional + nGood >= 50) {
            nGood = Optimizer::PoseOptimization(&mCurrentFrame, Ftype);

            // If many inliers but still not enough, search by projection again in a narrower window the camera has been already optimized with many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.Channels[Ftype].N; ip++)
                if (mCurrentFrame.Channels[Ftype].mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.Channels[Ftype].mvpMapPoints[ip]);
              nadditional = associater2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64, Ftype);

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame, Ftype);

                for (int io = 0; io < mCurrentFrame.Channels[Ftype].N; io++)
                  if (mCurrentFrame.Channels[Ftype].mvbOutlier[io])
                    mCurrentFrame.Channels[Ftype].mvpMapPoints[io] = NULL;
              }
            }
          }
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= 50) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    return true;
  }
}

void Tracking::UpdateLocalKeyFramesMultiChannels() {
  
  // Iterate all channels
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame *, int> keyframeCounter;
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]) {
        MapPoint *pMP = mCurrentFrame.Channels[Ftype].mvpMapPoints[i];
        if (!pMP->isBad()) {
          const map<KeyFrame *, size_t> observations = pMP->GetObservations();
          for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = NULL;
        }
      }
    }
  }

  if (keyframeCounter.empty())
    return;

  // used for finding the referencekeyframe
  int max = 0;
  KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++) {
    KeyFrame *pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to
  // already-included keyframes
  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80)
      break;

    KeyFrame *pKF = *itKF;

    const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame *pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame *> spChilds = pKF->GetChilds();
    for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
      KeyFrame *pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame *pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break; // BUG ??
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

void Tracking::UpdateLocalPointsMultiChannels() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
    KeyFrame *pKF = *itKF;
    for (int Ftype = 0; Ftype < Ntype; Ftype++) {
      const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches(Ftype);
      for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {
        MapPoint *pMP = *itMP;
        if (!pMP)
          continue;
        if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
          continue;
        if (!pMP->isBad()) {
          mvpLocalMapPoints.push_back(pMP);
          pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }
      }
    }
  }
}

void Tracking::SearchLocalPointsMultiChannels() {
  // Do not search map points already matched
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (vector<MapPoint *>::iterator vit = mCurrentFrame.Channels[Ftype].mvpMapPoints.begin(), vend = mCurrentFrame.Channels[Ftype].mvpMapPoints.end(); vit != vend; vit++) {
      MapPoint *pMP = *vit;
      if (pMP) {
        if (pMP->isBad()) {
          *vit = static_cast<MapPoint *>(NULL);
        } else {
          pMP->IncreaseVisible();
          pMP->mnLastFrameSeen = mCurrentFrame.mnId;
          pMP->mbTrackInView = false;
        }
      }
    }
  }
  
  int nToMatch = 0;
  // Project points in frame and check its visibility
  for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++) {
    MapPoint *pMP = *vit;
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    Associater associater(0.8);
    int th = 1;
    if(mSensor==System::RGBD)
        th=3;
    // If the camera has been relocalised recently, perform a coarser search
    if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
        th=5;
      
    associater.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);

    // // NN only matching
    // associater.SearchByNN(mCurrentFrame, mvpLocalMapPoints);
  }
}

void Tracking::UpdateLocalMapMultiChannels() {
  // This is for visualization
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFramesMultiChannels();
  UpdateLocalPointsMultiChannels();

}

bool Tracking::TrackLocalMapMultiChannels() {

  UpdateLocalMapMultiChannels();


  SearchLocalPointsMultiChannels();


  Optimizer::PoseOptimizationMultiChannels(&mCurrentFrame);
  mnMatchesInliers = 0;

  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
      if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]) {
        if (!mCurrentFrame.Channels[Ftype].mvbOutlier[i]) {
          mCurrentFrame.Channels[Ftype].mvpMapPoints[i]->IncreaseFound();
          if (!mbOnlyTracking) {
            if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i]->Observations() > 0)
              mnMatchesInliers++;
          } else
            mnMatchesInliers++;
        } else if (mSensor == System::STEREO)
          mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
      }
    }
  }
  
  // Decide if the tracking was succesful. More restrictive if there was a relocalization recently
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
    return false;

  if (mnMatchesInliers < 10)
    return false;
  else
    return true;
}

bool Tracking::NeedNewKeyFrameMultiChannels() {
  
  // step 1 : check VO
  if (mbOnlyTracking)
    return false;

  // step 2 : If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  // step 3 : Do not insert keyframes if not enough frames have passed from last relocalisation
  const int nKFs = mpMap->KeyFramesInMap();
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
    return false;

  // step 4 : Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;

  // sum all tracked map points
  int nRefMatches = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    nRefMatches += mpReferenceKF->TrackedMapPoints(nMinObs, Ftype);

  // step 5 : Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // if (bLocalMappingIdle)
  //   cout << "bLocalMappingIdle=ture" << endl;

  // step 6 : Check how many "close" points are being tracked and how many could be potentially created (for rgdb and sterreo)
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;
  if (mSensor != System::MONOCULAR) {
    for (int Ftype = 0; Ftype < Ntype; Ftype++) {
      for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
        if (mCurrentFrame.Channels[Ftype].mvDepth[i] > 0 && mCurrentFrame.Channels[Ftype].mvDepth[i] < mThDepth) {
          if (mCurrentFrame.Channels[Ftype].mvpMapPoints[i] && !mCurrentFrame.Channels[Ftype].mvbOutlier[i])
            nTrackedClose++;
          else
            nNonTrackedClose++;
        }
      }
    }
  }
  
  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  // step 7   : decision
  // step 7.1 : Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2)
    thRefRatio = 0.4f;

  if (mSensor == System::MONOCULAR)
    thRefRatio = 0.9f;

  // step 7.2 : Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  
  // step 7.3 : Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
  
  // step 7.4 : Condition 1c: tracking is weak
  const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  
  // step 7.5 : Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

  // cout << "mnMatchesInliers:" << mnMatchesInliers << endl;
  // cout << "nRefMatches:" << nRefMatches << endl;
  // cout << "nRefMatches*thRefRatio:"<< nRefMatches*thRefRatio << endl;
  // if (bNeedToInsertClose)
  //   cout << "bNeedToInsertClose = true" << endl;

  if ((c1a || c1b || c1c) && c2) {
    // If the mapping accepts keyframes, insert keyframe. Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      // cout << "INSERT KEYFRAME" << endl;
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor != System::MONOCULAR) {
        if (mpLocalMapper->KeyframesInQueue() < 3)
          return true;
        else
          return false;
      } else
        return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrameMultiChannels() {
  if (!mpLocalMapper->SetNotStop(true))
    return;

  // step 1 : create keyframe
  KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, Ntype);

  // step 2 : reference keyframe 
  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  

  // step 3 : create map points
  if (mSensor != System::MONOCULAR) {
    mCurrentFrame.UpdatePoseMatrices();

    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    // step 3.1 
    vector<vector<pair<float, int>>> vDepthIdx;
    vDepthIdx.resize(Ntype);
    for (int Ftype = 0; Ftype < Ntype; Ftype++) 
      vDepthIdx[Ftype].reserve(mCurrentFrame.Channels[Ftype].N);

    for (int Ftype = 0; Ftype < Ntype; Ftype++) {
      for (int i = 0; i < mCurrentFrame.Channels[Ftype].N; i++) {
        float z = mCurrentFrame.Channels[Ftype].mvDepth[i];
        if (z > 0) {
          vDepthIdx[Ftype].push_back(make_pair(z, i));
        }
      }
    }
    
    for (int Ftype = 0; Ftype < Ntype; Ftype++) {
      if (!vDepthIdx[Ftype].empty()) {
        sort(vDepthIdx[Ftype].begin(), vDepthIdx[Ftype].end());

        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx[Ftype].size(); j++) {
          int i = vDepthIdx[Ftype][j].second;

          bool bCreateNew = false;

          MapPoint *pMP = mCurrentFrame.Channels[Ftype].mvpMapPoints[i];
          if (!pMP)
            bCreateNew = true;
          else if (pMP->Observations() < 1) {
            bCreateNew = true;
            mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          }

          if (bCreateNew) {
            cv::Mat x3D = mCurrentFrame.UnprojectStereo(i, Ftype);
            MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap, Ftype);
            pNewMP->AddObservation(pKF, i);
            pKF->AddMapPoint(pNewMP, i, Ftype);
            pNewMP->ComputeDistinctiveDescriptors();
            pNewMP->UpdateNormalAndDepth();
            mpMap->AddMapPoint(pNewMP);

            mCurrentFrame.Channels[Ftype].mvpMapPoints[i] = pNewMP;
            nPoints++;
          } else {
            nPoints++;
          }

          if (vDepthIdx[Ftype][j].first > mThDepth && nPoints > 100)
            break;
        }
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::DiscardUnobservedMappoints(Frame &F, const int Ftype) {

  for(int i = 0; i < F.Channels[Ftype].N; i++) {
    MapPoint* pMP = F.Channels[Ftype].mvpMapPoints[i];
    if(pMP) {
      if(pMP->Observations()<1) {
        F.Channels[Ftype].mvbOutlier[i] = false;
        F.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint*>(NULL); 
      }
    }
  }
}

void Tracking::DiscardOutliersMappoints(Frame &F, const int Ftype) {

  for (int i = 0; i < F.Channels[Ftype].N; i++) {
    if (F.Channels[Ftype].mvpMapPoints[i] && F.Channels[Ftype].mvbOutlier[i]) {
      F.Channels[Ftype].mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
    }
  }
}


} // namespace ORB_SLAM2





