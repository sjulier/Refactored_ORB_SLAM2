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

#include "KeyFrame.h"
#include "Converter.h"
#include "Associater.h"
#include <mutex>

using namespace ::std;

namespace ORB_SLAM2 {

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, vector<KeyFrameDatabase *> pKFDB, int Ntype)
    : mnFrameId(F.mnId), 
      mTimeStamp(F.mTimeStamp), 
      mnGridCols(FRAME_GRID_COLS),
      mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(F.mfGridElementWidthInv),
      mfGridElementHeightInv(F.mfGridElementHeightInv),
      mnTrackReferenceForFrame(0), 
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnBAGlobalForKF(0), 
      fx(F.fx), 
      fy(F.fy), 
      cx(F.cx),
      cy(F.cy), 
      invfx(F.invfx), 
      invfy(F.invfy), 
      mbf(F.mbf),
      mb(F.mb),
      mThDepth(F.mThDepth),
      Channels(F.Channels), 
      mnScaleLevels(F.mnScaleLevels),
      mfScaleFactor(F.mfScaleFactor), 
      mfLogScaleFactor(F.mfLogScaleFactor),
      mvScaleFactors(F.mvScaleFactors), 
      mvLevelSigma2(F.mvLevelSigma2),
      mvInvLevelSigma2(F.mvInvLevelSigma2), 
      mnMinX(F.mnMinX), 
      mnMinY(F.mnMinY),
      mnMaxX(F.mnMaxX), 
      mnMaxY(F.mnMaxY), 
      mK(F.mK),
      mpKeyFrameDB(pKFDB),
      mpVocabulary(F.mpVocabulary), 
      mbFirstConnection(true),
      mpParent(NULL), 
      mbNotErase(false), 
      mbToBeErased(false), 
      mbBad(false),
      mHalfBaseline(F.mb / 2), 
      mpMap(pMap),
      Ntype(Ntype) {
  mnId = nNextId++;
  // Resize for vectors
  mnLoopQuery.resize(Ntype);
  mnLoopWords.resize(Ntype);
  mLoopScore.resize(Ntype);
  mnRelocQuery.resize(Ntype);
  mnRelocWords.resize(Ntype);
  mRelocScore.resize(Ntype);
  Channels.resize(Ntype);

  // Initlizer for Reloc
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    mnRelocQuery[Ftype] = 0;
    mnRelocWords[Ftype] = 0;
    mnLoopQuery[Ftype] = 0; 
    mnLoopWords[Ftype] = 0; 
  } 

  SetPose(F.mTcw);
}

/*
void KeyFrame::ComputeBoW(const int Ftype) {
  if (Channels[Ftype].mBowVec.empty() || Channels[Ftype].mFeatVec.empty()) {
    std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(Channels[Ftype].mDescriptors);
    mpVocabulary[Ftype]->transform(vCurrentDesc, Channels[Ftype].mBowVec, Channels[Ftype].mFeatVec, 4);
  }
}
*/

void KeyFrame::ComputeBoW(const int Ftype) {
  if (Channels[Ftype].mBowVec.empty() || Channels[Ftype].mFeatVec.empty()) {
    mpVocabulary[Ftype]->transform (
        Channels[Ftype].mDescriptors,
        Channels[Ftype].mBowVec,
        Channels[Ftype].mFeatVec, 4);
  }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_) {
  unique_lock<mutex> lock(mMutexPose);
  Tcw_.copyTo(Tcw);
  cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
  cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
  cv::Mat Rwc = Rcw.t();
  Ow = -Rwc * tcw;

  Twc = cv::Mat::eye(4, 4, Tcw.type());
  Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
  Ow.copyTo(Twc.rowRange(0, 3).col(3));
  cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
  Cw = Twc * center;
}

cv::Mat KeyFrame::GetPose() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse() {
  unique_lock<mutex> lock(mMutexPose);
  return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return Cw.clone();
}

cv::Mat KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw.rowRange(0, 3).col(3).clone();
}

// TO-DO multi channels ?
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

// TO-DO multi channels ?
void KeyFrame::UpdateBestCovisibles() {
  unique_lock<mutex> lock(mMutexConnections);
  std::vector<pair<int, KeyFrame *>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
    vPairs.push_back(make_pair(mit->second, mit->first));

  sort(vPairs.begin(), vPairs.end());
  std::list<KeyFrame *> lKFs;
  std::list<int> lWs;
  for (std::size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames = std::vector<KeyFrame *>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  set<KeyFrame *> s;
  for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
    s.insert(mit->first);
  return s;
}

std::vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

std::vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  unique_lock<mutex> lock(mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    return std::vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
}

std::vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
  unique_lock<mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty())
    return std::vector<KeyFrame *>();

  std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);
  if (it == mvOrderedWeights.end())
    return std::vector<KeyFrame *>();
  else {
    int n = it - mvOrderedWeights.begin();
    return std::vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
  }
}

int KeyFrame::GetWeight(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const std::size_t &idx, const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);
  Channels[Ftype].mvpMapPoints[idx] = pMP;
}

void KeyFrame::EraseMapPointMatch(const std::size_t &idx, const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);
  Channels[Ftype].mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
}

// Delete Ftype ? Ftype is useless because we have pMp ??
void KeyFrame::EraseMapPointMatch(MapPoint *pMP, const int Ftype) {
  int idx = pMP->GetIndexInKeyFrame(this);
  int _Ftype = pMP->GetFeatureType();
  if (idx >= 0)
    Channels[Ftype].mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
}

// Delete Ftype ? Ftype is useless because we have pMp ??
void KeyFrame::ReplaceMapPointMatch(const std::size_t &idx, MapPoint *pMP, const int Ftype) {
  int _Ftype = pMP->GetFeatureType();
  Channels[Ftype].mvpMapPoints[idx] = pMP;
}

set<MapPoint *> KeyFrame::GetMapPoints(const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);
  set<MapPoint *> s;
  for (std::size_t i = 0, iend = Channels[Ftype].mvpMapPoints.size(); i < iend; i++) {
    if (!Channels[Ftype].mvpMapPoints[i])
      continue;
    MapPoint *pMP = Channels[Ftype].mvpMapPoints[i];
    if (!pMP->isBad())
      s.insert(pMP);
  }
  return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs, const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  for (int i = 0; i < Channels[Ftype].N; i++) {
    MapPoint *pMP = Channels[Ftype].mvpMapPoints[i];
    if (pMP) {
      if (!pMP->isBad()) {
        if (bCheckObs) {
          if (Channels[Ftype].mvpMapPoints[i]->Observations() >= minObs)
            nPoints++;
        } else
          nPoints++;
      }
    }
  }

  return nPoints;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
  unique_lock<mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (int i = 0; i < Channels[Ftype].N; i++) {
      MapPoint *pMP = Channels[Ftype].mvpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          if (bCheckObs) {
            if (Channels[Ftype].mvpMapPoints[i]->Observations() >= minObs)
              nPoints++;
          } else
            nPoints++;
        }
      }
    }
  }
  
  return nPoints;
}

std::vector<MapPoint *> KeyFrame::GetMapPointMatches(const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);
  return Channels[Ftype].mvpMapPoints;
}

MapPoint *KeyFrame::GetMapPoint(const std::size_t &idx, const int Ftype) {
  unique_lock<mutex> lock(mMutexFeatures);
  return Channels[Ftype].mvpMapPoints[idx];
}

// Multi Channels ??
void KeyFrame::UpdateConnectionsMultiChannels() {
  map<KeyFrame *, int> KFcounter;

  std::vector<std::vector<MapPoint *>> vpMP;
  vpMP.resize(Ntype);

  {
    unique_lock<mutex> lockMPs(mMutexFeatures);
    for (int Ftype = 0; Ftype < Ntype; Ftype++) 
      vpMP[Ftype] = Channels[Ftype].mvpMapPoints;
  }

  // For all map points in keyframe check in which other keyframes are they seen, increase counter for those keyframes
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (std::vector<MapPoint *>::iterator vit = vpMP[Ftype].begin(), vend = vpMP[Ftype].end(); vit != vend; vit++) {
      MapPoint *pMP = *vit;

      if (!pMP)
        continue;

      if (pMP->isBad())
        continue;

      map<KeyFrame *, std::size_t> observations = pMP->GetObservations();

      for (map<KeyFrame *, std::size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
        if (mit->first->mnId == mnId)
          continue;
        KFcounter[mit->first]++;
      }
    }
  }

  // This should not happen
  if (KFcounter.empty())
    return;

  // If the counter is greater than threshold add connection, in case no keyframe counter is over threshold add the one with maximum counter
  int nmax = 0;
  KeyFrame *pKFmax = NULL;
  int th = 15;

  std::vector<pair<int, KeyFrame *>> vPairs;
  vPairs.reserve(KFcounter.size());
  for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }

    if (mit->second >= th) {
      vPairs.push_back(make_pair(mit->second, mit->first));
      (mit->first)->AddConnection(this, mit->second);
    }
  }

  if (vPairs.empty()) {
    vPairs.push_back(make_pair(nmax, pKFmax));
    pKFmax->AddConnection(this, nmax);
  }

  sort(vPairs.begin(), vPairs.end());
  std::list<KeyFrame *> lKFs;
  std::list<int> lWs;
  for (std::size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  {
    unique_lock<mutex> lockCon(mMutexConnections);

    // mspConnectedKeyFrames = spConnectedKeyFrames;
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = std::vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && mnId != 0) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(this);
      mbFirstConnection = false;
    }
  }
}

void KeyFrame::AddChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mpParent = pKF;
  pKF->AddChild(this);
}

set<KeyFrame *> KeyFrame::GetChilds() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

KeyFrame *KeyFrame::GetParent() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspLoopEdges.insert(pKF);
}

set<KeyFrame *> KeyFrame::GetLoopEdges() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
  unique_lock<mutex> lock(mMutexConnections);
  mbNotErase = true;
}

void KeyFrame::SetErase() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mspLoopEdges.empty()) {
      mbNotErase = false;
    }
  }

  if (mbToBeErased) {
    SetBadFlag();
  }
}

void KeyFrame::SetBadFlag() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mnId == 0)
      return;
    else if (mbNotErase) {
      mbToBeErased = true;
      return;
    }
  }

  for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
    mit->first->EraseConnection(this);

  // erase observation of every kind of map points (one keyframe, two kinds of mappoints)
  for (int Ftype = 0; Ftype < Ntype; Ftype++) {
    for (std::size_t i = 0; i < Channels[Ftype].mvpMapPoints.size(); i++)
      if (Channels[Ftype].mvpMapPoints[i])
        Channels[Ftype].mvpMapPoints[i]->EraseObservation(this);
  }
  
  {
    unique_lock<mutex> lock(mMutexConnections);
    unique_lock<mutex> lock1(mMutexFeatures);

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();

    // Update Spanning Tree
    set<KeyFrame *> sParentCandidates;
    sParentCandidates.insert(mpParent);

    // Assign at each iteration one children with a parent (the pair with highest covisibility weight) Include that children as new parent candidate for the rest
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int max = -1;
      KeyFrame *pC;
      KeyFrame *pP;

      for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++) {
        KeyFrame *pKF = *sit;
        if (pKF->isBad())
          continue;

        // Check if a parent candidate is connected to the keyframe
        std::vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
        for (std::size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
          for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++) {
            if (vpConnected[i]->mnId == (*spcit)->mnId) {
              int w = pKF->GetWeight(vpConnected[i]);
              if (w > max) {
                pC = pKF;
                pP = vpConnected[i];
                max = w;
                bContinue = true;
              }
            }
          }
        }
      }

      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else
        break;
    }

    // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
    if (!mspChildrens.empty())
      for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
        (*sit)->ChangeParent(mpParent);
      }

    mpParent->EraseChild(this);
    mTcp = Tcw * mpParent->GetPoseInverse();
    mbBad = true;
  }

  mpMap->EraseKeyFrame(this);
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    mpKeyFrameDB[Ftype]->erase(this, Ftype);
}

bool KeyFrame::isBad() {
  unique_lock<mutex> lock(mMutexConnections);
  return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame *pKF) {
  bool bUpdate = false;
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate)
    UpdateBestCovisibles();
}

std::vector<std::size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int Ftype) const {
  std::vector<std::size_t> vIndices;
  vIndices.reserve(Channels[Ftype].N);

  const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
  if (nMinCellX >= mnGridCols)
    return vIndices;

  const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
  if (nMaxCellX < 0)
    return vIndices;

  const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
  if (nMinCellY >= mnGridRows)
    return vIndices;

  const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
  if (nMaxCellY < 0)
    return vIndices;

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<std::size_t> vCell = Channels[Ftype].mGrid[ix][iy];
      for (std::size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = Channels[Ftype].mvKeysUn[vCell[j]];
        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
  return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i, const int Ftype) {
  const float z = Channels[Ftype].mvDepth[i];
  if (z > 0) {
    const float u = Channels[Ftype].mvKeys[i].pt.x;
    const float v = Channels[Ftype].mvKeys[i].pt.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
  } else
    return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q, const int Ftype) {
  std::vector<MapPoint *> vpMapPoints;
  cv::Mat Tcw_;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPose);
    vpMapPoints = Channels[Ftype].mvpMapPoints;
    Tcw_ = Tcw.clone();
  }

  std::vector<float> vDepths;
  vDepths.reserve(vpMapPoints.size());
  cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw_.at<float>(2, 3);
  for (int i = 0; i < vpMapPoints.size(); i++) {
    if (vpMapPoints[i]) {  //vpmappoints ??
      MapPoint *pMP = vpMapPoints[i];
      cv::Mat x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

 // If no mappoint, return invalid depth
  if(vDepths.empty())
    return -1.0f;

  sort(vDepths.begin(), vDepths.end());

  return vDepths[(vDepths.size() - 1) / q];
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  std::vector<MapPoint *> vpMapPoints;
  cv::Mat Tcw_;

  int Nsum = 0;
  for (int Ftype = 0; Ftype < Ntype; Ftype++)
    Nsum += Channels[Ftype].N;
  vpMapPoints.reserve(Nsum);

  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPose);

    for (int Ftype = 0; Ftype < Ntype; Ftype++)
      vpMapPoints.insert(vpMapPoints.end(), Channels[Ftype].mvpMapPoints.begin(), Channels[Ftype].mvpMapPoints.end());

    Tcw_ = Tcw.clone();
  }

  std::vector<float> vDepths;
  vDepths.reserve(vpMapPoints.size());
  cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw_.at<float>(2, 3);
  for (int i = 0; i < vpMapPoints.size(); i++) {
    if (vpMapPoints[i]) {
      MapPoint *pMP = vpMapPoints[i];
      cv::Mat x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

  sort(vDepths.begin(), vDepths.end());

  return vDepths[(vDepths.size() - 1) / q];
}

} // namespace ORB_SLAM2
