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

#include "Frame.h"
// #include "Converter.h"
#include "Associater.h"
#include <thread>

using namespace ::std;

namespace ORB_SLAM2 {

long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame(int Ntype) {
  Channels.resize(Ntype);
  mpFeatureExtractorLeft.resize(Ntype);
  mpFeatureExtractorRight.resize(Ntype);
}

// Copy Constructor
Frame::Frame(const Frame &frame)
    : mpVocabulary(frame.mpVocabulary), 
      mTimeStamp(frame.mTimeStamp), 
      mK(frame.mK.clone()),
      mDistCoef(frame.mDistCoef.clone()), 
      mbf(frame.mbf), 
      mb(frame.mb),
      mThDepth(frame.mThDepth), 
      Channels(frame.Channels), 
      mnId(frame.mnId), 
      mpReferenceKF(frame.mpReferenceKF),
      mnScaleLevels(frame.mnScaleLevels), 
      mfScaleFactor(frame.mfScaleFactor),
      mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors),
      mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2),
      mvInvLevelSigma2(frame.mvInvLevelSigma2),
      mpFeatureExtractorLeft(frame.mpFeatureExtractorLeft),
      mpFeatureExtractorRight(frame.mpFeatureExtractorRight),
      Ntype(frame.Ntype) {
  Channels.resize(Ntype);
  /*
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    mpFeatureExtractorLeft[Ftype] = frame.mpFeatureExtractorLeft[Ftype];
    mpFeatureExtractorRight[Ftype] = frame.mpFeatureExtractorRight[Ftype];
  }
  */

  if (!frame.mTcw.empty())
    SetPose(frame.mTcw);
}

// Stereo
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, 
             std::vector<FeatureExtractor *> extractorLeft, std::vector<FeatureExtractor *> extractorRight,
             vector<FbowVocabulary *> voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth, int Ntype)
    : mpVocabulary(voc), 
      mTimeStamp(timeStamp),
      mK(K.clone()), 
      mDistCoef(distCoef.clone()), 
      mbf(bf), 
      mThDepth(thDepth),
      mpReferenceKF(static_cast<KeyFrame *>(NULL)),
      mpFeatureExtractorLeft(extractorLeft),
      mpFeatureExtractorRight(extractorRight),
      Ntype(Ntype) {
  Channels.resize(Ntype);

  // Frame ID
  mnId = nNextId++;

  // Scale Level Info
  mnScaleLevels = mpFeatureExtractorLeft[0]->GetLevels();
  mfScaleFactor = mpFeatureExtractorLeft[0]->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpFeatureExtractorLeft[0]->GetScaleFactors();
  mvInvScaleFactors = mpFeatureExtractorLeft[0]->GetInverseScaleFactors();
  mvLevelSigma2 = mpFeatureExtractorLeft[0]->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpFeatureExtractorLeft[0]->GetInverseScaleSigmaSquares();

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imLeft);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  mb = mbf / fx;

  thread CompFeaturesThread[Ntype];

  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype] = thread(&Frame::ComputeFeaturesStereo, this, Ftype, imLeft, imRight);
    
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype].join();

  // for (int Ftype = 0; Ftype < Ntype; Ftype++)
  //   ComputeFeaturesStereo(Ftype, imLeft, imRight);
}

// RGB-D
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,
             const double &timeStamp, 
             std::vector<FeatureExtractor *> extractor,
             vector<FbowVocabulary *> voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
             const float &thDepth, int Ntype)
    : mpVocabulary(voc), 
      mTimeStamp(timeStamp), 
      mK(K.clone()), 
      mDistCoef(distCoef.clone()),
      mbf(bf), 
      mThDepth(thDepth),
      mpFeatureExtractorLeft(extractor),
      Ntype(Ntype) {
  // Resize the vectors
  Channels.resize(Ntype);
  mpFeatureExtractorRight.resize(Ntype);

  // Frame ID
  mnId = nNextId++;

  // Feature extractor initlization
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    // mpFeatureExtractorLeft[Ftype] = extractor[Ftype];
    mpFeatureExtractorRight[Ftype] = static_cast<FeatureExtractor *>(NULL);
  }

  // Scale Level Info
  mnScaleLevels = mpFeatureExtractorLeft[0]->GetLevels();
  mfScaleFactor = mpFeatureExtractorLeft[0]->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpFeatureExtractorLeft[0]->GetScaleFactors();
  mvInvScaleFactors = mpFeatureExtractorLeft[0]->GetInverseScaleFactors();
  mvLevelSigma2 = mpFeatureExtractorLeft[0]->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpFeatureExtractorLeft[0]->GetInverseScaleSigmaSquares();

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  mb = mbf / fx;

  thread CompFeaturesThread[Ntype];

  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype] = thread(&Frame::ComputeFeaturesRGBD, this, Ftype, imGray, imDepth);
    
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype].join();

}

// Mono
Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             std::vector<FeatureExtractor *> extractor,
             vector<FbowVocabulary *> voc, cv::Mat &K,
             cv::Mat &distCoef, const float &bf, const float &thDepth, int Ntype)
    : mpVocabulary(voc),
      mTimeStamp(timeStamp), 
      mK(K.clone()), 
      mDistCoef(distCoef.clone()),
      mbf(bf), 
      mThDepth(thDepth),
      mpFeatureExtractorLeft(extractor),
      Ntype(Ntype) {
  // Resize the vectors
  Channels.resize(Ntype);
  mpFeatureExtractorRight.resize(Ntype);

  // Frame ID
  mnId = nNextId++;

  // Feature extractor initlization
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    //mpFeatureExtractorLeft[Ftype] = extractor[Ftype];
    mpFeatureExtractorRight[Ftype] = static_cast<FeatureExtractor *>(NULL);
  }

  // Scale Level Info
  mnScaleLevels = mpFeatureExtractorLeft[0]->GetLevels();
  mfScaleFactor = mpFeatureExtractorLeft[0]->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpFeatureExtractorLeft[0]->GetScaleFactors();
  mvInvScaleFactors = mpFeatureExtractorLeft[0]->GetInverseScaleFactors();
  mvLevelSigma2 = mpFeatureExtractorLeft[0]->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpFeatureExtractorLeft[0]->GetInverseScaleSigmaSquares();

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);
    invfx = 1.0f / fx;
    invfy = 1.0f / fy;

    mbInitialComputations = false;
  }

  mb = mbf / fx;

  thread CompFeaturesThread[Ntype];
  
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype] = thread(&Frame::ComputeFeaturesMono, this, Ftype, imGray);

  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    CompFeaturesThread[Ftype].join();
}

void Frame::AssignFeaturesToGrid(const int Ftype) {
  int nReserve = 0.5f * Channels[Ftype].N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
  Channels[Ftype].mGrid.resize(FRAME_GRID_COLS);
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++) {
    Channels[Ftype].mGrid[i].resize(FRAME_GRID_ROWS);
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
      Channels[Ftype].mGrid[i][j].reserve(nReserve);
    }
  }
 
  for (int i = 0; i < Channels[Ftype].N; i++) {
    const cv::KeyPoint &kp = Channels[Ftype].mvKeysUn[i];

    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY))
      Channels[Ftype].mGrid[nGridPosX][nGridPosY].push_back(i);
  }
}

//rewrite AssignFeaturesToGrid()
void Frame::AssignFeaturesToGrid(const int &refN, const vector<cv::KeyPoint> &KeysUn,vector<vector<vector<size_t>>> &Grid) {
  int nReserve = 0.5f * refN / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
  Grid.resize(FRAME_GRID_COLS);
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++) {
    Grid[i].resize(FRAME_GRID_ROWS);
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
      Grid[i][j].reserve(nReserve);
    }
  }

  for (int i = 0; i < refN; i++) {
    const cv::KeyPoint &kp = KeysUn[i];

    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY))
      Grid[nGridPosX][nGridPosY].push_back(i);
  }
}

void Frame::ExtractFeatures(const int Ftype, int imageFlag, const cv::Mat &im) {
  if (imageFlag == 0) {
    (*mpFeatureExtractorLeft[Ftype])(im, cv::Mat(), Channels[Ftype].mvKeys, Channels[Ftype].mDescriptors);
  }
  else {
    (*mpFeatureExtractorRight[Ftype])(im, cv::Mat(), Channels[Ftype].mvKeysRight, Channels[Ftype].mDescriptorsRight);
  }
}

void Frame::SetPose(cv::Mat Tcw) {
  mTcw = Tcw.clone();
  UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices() {
  mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
  mRwc = mRcw.t();
  mtcw = mTcw.rowRange(0, 3).col(3);
  mOw = -mRcw.t() * mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  pMP->mbTrackInView = false;

  // 3D in absolute coordinates
  cv::Mat P = pMP->GetWorldPos();

  // 3D in camera coordinates
  const cv::Mat Pc = mRcw * P + mtcw;
  const float &PcX = Pc.at<float>(0);
  const float &PcY = Pc.at<float>(1);
  const float &PcZ = Pc.at<float>(2);

  // Check positive depth
  if (PcZ < 0.0f)
    return false;

  // Project in image and check it is not outside
  const float invz = 1.0f / PcZ;
  const float u = fx * PcX * invz + cx;
  const float v = fy * PcY * invz + cy;

  if (u < mnMinX || u > mnMaxX)
    return false;
  if (v < mnMinY || v > mnMaxY)
    return false;

  // Check distance is in the scale invariance region of the MapPoint
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  const cv::Mat PO = P - mOw;
  const float dist = cv::norm(PO);

  if (dist < minDistance || dist > maxDistance)
    return false;

  // Check viewing angle
  cv::Mat Pn = pMP->GetNormal();

  const float viewCos = PO.dot(Pn) / dist;

  if (viewCos < viewingCosLimit)
    return false;

  // Predict scale in the image
  const int nPredictedLevel = pMP->PredictScale(dist, this);

  // Data used by the tracking
  pMP->mbTrackInView = true;
  pMP->mTrackProjX = u;
  pMP->mTrackProjXR = u - mbf * invz;
  pMP->mTrackProjY = v;
  pMP->mnTrackScaleLevel = nPredictedLevel;
  pMP->mTrackViewCos = viewCos;

  return true;
}

// Rewrite GetFeaturesInArea()
vector<size_t> Frame::GetFeaturesInArea(const int Ftype, const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const {
  vector<size_t> vIndices;
  vIndices.reserve(Channels[Ftype].N);

  const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
  if (nMinCellX >= FRAME_GRID_COLS)
    return vIndices;

  const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
  if (nMaxCellX < 0)
    return vIndices;

  const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
  if (nMinCellY >= FRAME_GRID_ROWS)
    return vIndices;

  const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
  if (nMaxCellY < 0)
    return vIndices;

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const vector<size_t> vCell = Channels[Ftype].mGrid[ix][iy];
      if (vCell.empty())
        continue;

      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = Channels[Ftype].mvKeysUn[vCell[j]];
        if (bCheckLevels) {
          if (kpUn.octave < minLevel)
            continue;
          if (maxLevel >= 0)
            if (kpUn.octave > maxLevel)
              continue;
        }

        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}

void Frame::ComputeBoW(const int Ftype) {
  if (Channels[Ftype].mBowVec.empty()) {
    // vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(Channels[Ftype].mDescriptors);
    mpVocabulary[Ftype]->transform(Channels[Ftype].mDescriptors, Channels[Ftype].mBowVec, Channels[Ftype].mFeatVec, 4);
  }
}

void Frame::UndistortKeyPoints(const int Ftype) {
  if (mDistCoef.at<float>(0) == 0.0) {
    Channels[Ftype].mvKeysUn = Channels[Ftype].mvKeys;
    return;
  }

  // Fill matrix with points
  cv::Mat mat(Channels[Ftype].N, 2, CV_32F);
  for (int i = 0; i < Channels[Ftype].N; i++) {
    mat.at<float>(i, 0) = Channels[Ftype].mvKeys[i].pt.x;
    mat.at<float>(i, 1) = Channels[Ftype].mvKeys[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  Channels[Ftype].mvKeysUn.resize(Channels[Ftype].N);
  for (int i = 0; i < Channels[Ftype].N; i++) {
    cv::KeyPoint kp = Channels[Ftype].mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    Channels[Ftype].mvKeysUn[i] = kp;
  }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  if (mDistCoef.at<float>(0) != 0.0) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // Undistort corners
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));

  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

void Frame::ComputeStereoMatches(const int Ftype) {
  Channels[Ftype].mvuRight = vector<float>(Channels[Ftype].N, -1.0f);
  Channels[Ftype].mvDepth = vector<float>(Channels[Ftype].N, -1.0f);

  const float thOrbDist = (Associater::mvTH_HIGH[Ftype] + Associater::mvTH_LOW[Ftype]) / 2;

  const int nRows = mpFeatureExtractorLeft[Ftype]->mvImagePyramid[0].rows;

  // Assign keypoints to row table
  vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());
  for (int i = 0; i < nRows; i++)
    vRowIndices[i].reserve(200);

  const int Nr =  Channels[Ftype].mvKeysRight.size();

  for (int iR = 0; iR < Nr; iR++) {
    const cv::KeyPoint &kp = Channels[Ftype].mvKeysRight[iR];
    const float &kpY = kp.pt.y;
    const float r = 2.0f * mvScaleFactors[Channels[Ftype].mvKeysRight[iR].octave];
    const int maxr = ceil(kpY + r);
    const int minr = floor(kpY - r);

    for (int yi = minr; yi <= maxr; yi++)
      vRowIndices[yi].push_back(iR);
  }

  // Set limits for search
  const float minZ = mb;
  const float minD = 0;
  const float maxD = mbf / minZ;

  // For each left keypoint search a match in the right image
  vector<pair<int, int>> vDistIdx;
  vDistIdx.reserve(Channels[Ftype].N);

  for (int iL = 0; iL < Channels[Ftype].N; iL++) {
    const cv::KeyPoint &kpL = Channels[Ftype].mvKeys[iL];
    const int &levelL = kpL.octave;
    const float &vL = kpL.pt.y;
    const float &uL = kpL.pt.x;

    const vector<size_t> &vCandidates = vRowIndices[vL];

    if (vCandidates.empty())
      continue;

    const float minU = uL - maxD;
    const float maxU = uL - minD;

    if (maxU < 0)
      continue;

    float bestDist = Associater::mvTH_HIGH[Ftype];
    size_t bestIdxR = 0;

    const cv::Mat &dL = Channels[Ftype].mDescriptors.row(iL);

    // Compare descriptor to right keypoints
    for (size_t iC = 0; iC < vCandidates.size(); iC++) {
      const size_t iR = vCandidates[iC];
      const cv::KeyPoint &kpR = Channels[Ftype].mvKeysRight[iR];

      if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
        continue;

      const float &uR = kpR.pt.x;

      if (uR >= minU && uR <= maxU) {
        const cv::Mat &dR = Channels[Ftype].mDescriptorsRight.row(iR);
        const float dist = Associater::DescriptorDistance(dL, dR);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdxR = iR;
        }
      }
    }

    // Subpixel match by correlation
    if (bestDist < thOrbDist) {
      // coordinates in image pyramid at keypoint scale
      const float uR0 = Channels[Ftype].mvKeysRight[bestIdxR].pt.x;
      const float scaleFactor = mvInvScaleFactors[kpL.octave];
      const float scaleduL = round(kpL.pt.x * scaleFactor);
      const float scaledvL = round(kpL.pt.y * scaleFactor);
      const float scaleduR0 = round(uR0 * scaleFactor);

      // sliding window search
      const int w = 5;

      // Range protection
	  const cv::Mat& pyrImgL = mpFeatureExtractorLeft[Ftype]->mvImagePyramid[kpL.octave];
      const cv::Mat& pyrImgR = mpFeatureExtractorRight[Ftype]->mvImagePyramid[kpL.octave];
	  if (scaledvL - w < 0 || scaledvL + w + 1 > pyrImgL.rows || scaleduL - w < 0 || scaleduL + w + 1 > pyrImgL.cols)
        continue;

      cv::Mat IL = mpFeatureExtractorLeft[Ftype]->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
      IL.convertTo(IL, CV_32F);
      IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

      int bestDist = INT_MAX;
      int bestincR = 0;
      const int L = 5;
      vector<float> vDists;
      vDists.resize(2 * L + 1);

      const float iniu = scaleduR0 + L - w;
      const float endu = scaleduR0 + L + w + 1;
      if (iniu < 0 || endu >= mpFeatureExtractorRight[Ftype]->mvImagePyramid[kpL.octave].cols)
        continue;

      for (int incR = -L; incR <= +L; incR++) {
		// Range protection
		int u = scaleduR0 + incR;
        if (scaledvL - w < 0 || scaledvL + w + 1 > pyrImgR.rows || u - w < 0 || u + w + 1 > pyrImgR.cols)
          continue;

        cv::Mat IR = mpFeatureExtractorRight[Ftype]->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
        IR.convertTo(IR, CV_32F);
        IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

        float dist = cv::norm(IL, IR, cv::NORM_L1);
        if (dist < bestDist) {
          bestDist = dist;
          bestincR = incR;
        }

        vDists[L + incR] = dist;
      }

      if (bestincR == -L || bestincR == L)
        continue;

      // Sub-pixel match (Parabola fitting)
      const float dist1 = vDists[L + bestincR - 1];
      const float dist2 = vDists[L + bestincR];
      const float dist3 = vDists[L + bestincR + 1];

      const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

      if (deltaR < -1 || deltaR > 1)
        continue;

      // Re-scaled coordinate
      float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

      float disparity = (uL - bestuR);

      if (disparity >= minD && disparity < maxD) {
        if (disparity <= 0) {
          disparity = 0.01;
          bestuR = uL - 0.01;
        }
        Channels[Ftype].mvDepth[iL] = mbf / disparity;
        Channels[Ftype].mvuRight[iL] = bestuR;
        vDistIdx.push_back(pair<int, int>(bestDist, iL));
      }
    }
  }

  sort(vDistIdx.begin(), vDistIdx.end());
  const float median = vDistIdx[vDistIdx.size() / 2].first;
  const float thDist = 1.5f * 1.4f * median;

  for (int i = vDistIdx.size() - 1; i >= 0; i--) {
    if (vDistIdx[i].first < thDist)
      break;
    else {
      Channels[Ftype].mvuRight[vDistIdx[i].second] = -1;
      Channels[Ftype].mvDepth[vDistIdx[i].second] = -1;
    }
  }
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth, const int Ftype) {
  Channels[Ftype].mvuRight = vector<float>(Channels[Ftype].N, -1);
  Channels[Ftype].mvDepth = vector<float>(Channels[Ftype].N, -1);

  for (int i = 0; i < Channels[Ftype].N; i++) {
    const cv::KeyPoint &kp = Channels[Ftype].mvKeys[i];
    const cv::KeyPoint &kpU = Channels[Ftype].mvKeysUn[i];

    const float &v = kp.pt.y;
    const float &u = kp.pt.x;

    const float d = imDepth.at<float>(v, u);

    // if(d>0)
    if (d > 0.1f && d < 20.f) {
      Channels[Ftype].mvDepth[i] = d;
      Channels[Ftype].mvuRight[i] = kpU.pt.x - mbf / d;
    }
  }
}

cv::Mat Frame::UnprojectStereo(const int &i, const int Ftype) {
  const float z = Channels[Ftype].mvDepth[i];
  if (z > 0) {
    const float u = Channels[Ftype].mvKeysUn[i].pt.x;
    const float v = Channels[Ftype].mvKeysUn[i].pt.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
    return mRwc * x3Dc + mOw;
  } else
    return cv::Mat();
}

void Frame::ComputeFeaturesRGBD(const int Ftype, const cv::Mat &imGray, const cv::Mat &imDepth) {
  // Feature extraction
  
  // cout << "ExtractFeatures (before)" << Ftype << endl;

  ExtractFeatures(Ftype, 0, imGray);

  // cout << "ExtractFeatures (after)" << Ftype << endl;


  Channels[Ftype].N = Channels[Ftype].mvKeys.size();
  
  if (Channels[Ftype].mvKeys.empty())
    return;

  
  // mvKeysUn, Left image
  UndistortKeyPoints(Ftype);
  // UndistortKeyPoints(Channels[Ftype].mvKeys, Channels[Ftype].mvKeysUn, Channels[Ftype].N);

  // cout << "UndistortKeyPoints" << Ftype << endl;
  
  // compute mvuRight and mvDepth
  ComputeStereoFromRGBD(imDepth, Ftype);
  // ComputeStereoFromRGBD(imDepth, Channels[Ftype].mvuRight, Channels[Ftype].mvDepth, Channels[Ftype].N, Channels[Ftype].mvKeys, Channels[Ftype].mvKeysUn);  

  // cout << "ComputeStereoFromRGBD" << Ftype << endl;

  // map points
  Channels[Ftype].mvpMapPoints = vector<MapPoint *>(Channels[Ftype].N, static_cast<MapPoint *>(NULL));

  // outliers
  Channels[Ftype].mvbOutlier = vector<bool>(Channels[Ftype].N, false);

  
  AssignFeaturesToGrid(Ftype);
  //AssignFeaturesToGrid(Channels[Ftype].N, Channels[Ftype].mvKeysUn, Channels[Ftype].mGrid);
  // cout << "AssignFeaturesToGrid" << Ftype << endl;
}

void Frame::ComputeFeaturesStereo(const int Ftype, const cv::Mat &imLeft, const cv::Mat &imRight) {
  // Feature extraction
  thread threadLeft(&Frame::ExtractFeatures, this, Ftype, 0, imLeft);
  thread threadRight(&Frame::ExtractFeatures, this, Ftype, 1, imRight);
  threadLeft.join();
  threadRight.join();

  Channels[Ftype].N = Channels[Ftype].mvKeys.size();
  
  if (Channels[Ftype].mvKeys.empty())
    return;

  // mvKeysUn, Left image
  UndistortKeyPoints(Ftype);

  // compute mvuRight and mvDepth
  ComputeStereoMatches(Ftype);
  
  // map points
  Channels[Ftype].mvpMapPoints = vector<MapPoint *>(Channels[Ftype].N, static_cast<MapPoint *>(NULL));

  // outliers
  Channels[Ftype].mvbOutlier = vector<bool>(Channels[Ftype].N, false);

  AssignFeaturesToGrid(Ftype);
}

void Frame::ComputeFeaturesMono(const int Ftype, const cv::Mat &imGray) {
  // Feature extraction
  ExtractFeatures(Ftype, 0, imGray);

  Channels[Ftype].N = Channels[Ftype].mvKeys.size();
  
  if (Channels[Ftype].mvKeys.empty())
    return;

  // mvKeysUn, Left image
  UndistortKeyPoints(Ftype);

  // Set no stereo information
  Channels[Ftype].mvuRight = vector<float>(Channels[Ftype].N, -1);
  Channels[Ftype].mvDepth = vector<float>(Channels[Ftype].N, -1);

  // map points
  Channels[Ftype].mvpMapPoints = vector<MapPoint *>(Channels[Ftype].N, static_cast<MapPoint *>(NULL));

  // outliers
  Channels[Ftype].mvbOutlier = vector<bool>(Channels[Ftype].N, false);

  AssignFeaturesToGrid(Ftype);
}

// Rewrite UndistortKeyPoints
// void Frame::UndistortKeyPoints(const vector<cv::KeyPoint> &Keys, vector<cv::KeyPoint> &KeysUn, const int &refN) {
//   if (mDistCoef.at<float>(0) == 0.0) {
//     KeysUn = Keys;
//   }

//   // Fill matrix with points
//   cv::Mat mat(refN, 2, CV_32F);
//   for (int i = 0; i < refN; i++) {
//     mat.at<float>(i, 0) = Keys[i].pt.x;
//     mat.at<float>(i, 1) = Keys[i].pt.y;
//   }

//   // Undistort points
//   mat = mat.reshape(2);
//   cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
//   mat = mat.reshape(1);

//   // Fill undistorted keypoint vector
//   KeysUn.resize(refN);
//   for (int i = 0; i < refN; i++) {
//     cv::KeyPoint kp = Keys[i];
//     kp.pt.x = mat.at<float>(i, 0);
//     kp.pt.y = mat.at<float>(i, 1);
//     KeysUn[i] = kp;
//   }
// }

// Rewrite UnprojectStereo()
// cv::Mat Frame::UnprojectStereo(const int &i, const vector<float> &Depth, const vector<cv::KeyPoint> &KeysUn) {
//   const float z = Depth[i];
//   if (z > 0) {
//     const float u = KeysUn[i].pt.x;
//     const float v = KeysUn[i].pt.y;
//     const float x = (u - cx) * z * invfx;
//     const float y = (v - cy) * z * invfy;
//     cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
//     return mRwc * x3Dc + mOw;
//   } else
//     return cv::Mat();
// }

// Rewrite ComputeStereoFromRGBD
// void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth, vector<float> &uRight, vector<float> &Depth, const int &refN, 
//                                   const vector<cv::KeyPoint> &Keys, const vector<cv::KeyPoint> &KeysUn) {
//   uRight = vector<float>(refN, -1);
//   Depth = vector<float>(refN, -1);

//   for (int i = 0; i < refN; i++) {

//     const cv::KeyPoint &kp = Keys[i];
//     const cv::KeyPoint &kpU = KeysUn[i];

//     const float &v = kp.pt.y;
//     const float &u = kp.pt.x;

//     const float d = imDepth.at<float>(v, u);

//     if (d > 0.1f && d < 20.f) {
//       Depth[i] = d;
//       uRight[i] = kpU.pt.x - mbf / d;
//     }
//   }
// }

} // namespace ORB_SLAM2
