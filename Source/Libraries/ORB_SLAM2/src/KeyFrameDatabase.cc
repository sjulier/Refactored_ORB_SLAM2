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

#include "KeyFrameDatabase.h"

// #include "DBoW2/BowVector.h"
#include "KeyFrame.h"

#include <mutex>

using namespace std;

using namespace ::std;

namespace ORB_SLAM2 {

KeyFrameDatabase::KeyFrameDatabase(const FbowVocabulary &voc) : mpVoc(&voc) {
  // mvInvertedFile.resize(voc.size());
}

//TO-DO add Ftype
void KeyFrameDatabase::add(KeyFrame *pKF, const int Ftype) {
  unique_lock<mutex> lock(mMutex);

  // for (fbow::fBow::const_iterator vit = pKF->Channels[Ftype].mBowVec.begin(), vend = pKF->Channels[Ftype].mBowVec.end(); vit != vend; vit++)
  //   mvInvertedFile[vit->first].push_back(pKF);
  for (auto const& kv : pKF->Channels[Ftype].mBowVec)
        mInvertedFile[kv.first].push_back(pKF);
}

/*
//TO-DO add Ftype
void KeyFrameDatabase::erase(KeyFrame *pKF, const int Ftype) {
  unique_lock<mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (fbow::fBow::const_iterator vit = pKF->Channels[Ftype].mBowVec.begin(), vend = pKF->Channels[Ftype].mBowVec.end(); vit != vend; vit++) {
    // List of keyframes that share the word
    std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

    for (std::list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
      if (pKF == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
}
*/
void KeyFrameDatabase::erase(KeyFrame* pKF, int Ftype)
{
    std::unique_lock<std::mutex> lock(mMutex);

    for (auto const& kv : pKF->Channels[Ftype].mBowVec) {
        auto& lst = mInvertedFile[kv.first];
        lst.remove(pKF);
    }
}

/*
void KeyFrameDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}
*/
void KeyFrameDatabase::clear()
{
    std::unique_lock<std::mutex> lock(mMutex);
    for (auto &kv : mInvertedFile)
        kv.second.clear();
}

std::vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, float minScore, const int Ftype) {
  set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame *> lKFsSharingWords;

  // Search all keyframes that share a word with current keyframes. Discard keyframes connected to the query keyframe
  {
    unique_lock<mutex> lock(mMutex);

    for (fbow::fBow::const_iterator vit = pKF->Channels[Ftype].mBowVec.begin(), vend = pKF->Channels[Ftype].mBowVec.end(); vit != vend; vit++) {
      // std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];
      auto& lKFs = mInvertedFile[vit->first];

      for (std::list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (pKFi->mnLoopQuery[Ftype] != pKF->mnId) {
          pKFi->mnLoopWords[Ftype] = 0;
          if (!spConnectedKeyFrames.count(pKFi)) {
            pKFi->mnLoopQuery[Ftype] = pKF->mnId;
            lKFsSharingWords.push_back(pKFi);
          }
        }
        pKFi->mnLoopWords[Ftype]++;
      }
    }
  }

  if (lKFsSharingWords.empty())
    return std::vector<KeyFrame *>();

  std::list<pair<float, KeyFrame *>> lScoreAndMatch;

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (std::list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    if ((*lit)->mnLoopWords[Ftype] > maxCommonWords)
      maxCommonWords = (*lit)->mnLoopWords[Ftype];
  }

  int minCommonWords = maxCommonWords * 0.8f;

  int nscores = 0;

  // Compute similarity score. Retain the matches whose score is higher than minScore
  for (std::list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    KeyFrame *pKFi = *lit;

    if (pKFi->mnLoopWords[Ftype] > minCommonWords) {
      nscores++;

      float si = mpVoc->score(pKF->Channels[Ftype].mBowVec, pKFi->Channels[Ftype].mBowVec);

      pKFi->mLoopScore[Ftype] = si;
      if (si >= minScore)
        lScoreAndMatch.push_back(make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty())
    return std::vector<KeyFrame *>();

  std::list<pair<float, KeyFrame *>> lAccScoreAndMatch;
  float bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  for (std::list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
    KeyFrame *pKFi = it->second;
    std::vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = it->first;
    KeyFrame *pBestKF = pKFi;
    for (std::vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
      KeyFrame *pKF2 = *vit;
      if (pKF2->mnLoopQuery[Ftype] == pKF->mnId && pKF2->mnLoopWords[Ftype] > minCommonWords) {
        accScore += pKF2->mLoopScore[Ftype];
        if (pKF2->mLoopScore[Ftype] > bestScore) {
          pBestKF = pKF2;
          bestScore = pKF2->mLoopScore[Ftype];
        }
      }
    }

    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
    if (accScore > bestAccScore)
      bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;

  set<KeyFrame *> spAlreadyAddedKF;
  std::vector<KeyFrame *> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  for (std::list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
    if (it->first > minScoreToRetain) {
      KeyFrame *pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpLoopCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpLoopCandidates;
}

std::vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, const int Ftype) {
  std::list<KeyFrame *> lKFsSharingWords;

  // Search all keyframes that share a word with current frame
  {
    unique_lock<mutex> lock(mMutex);

    for (fbow::fBow::const_iterator vit = F->Channels[Ftype].mBowVec.begin(), vend = F->Channels[Ftype].mBowVec.end(); vit != vend; vit++) {
      // std::list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];
      auto& lKFs = mInvertedFile[vit->first];

      for (std::list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (pKFi->mnRelocQuery[Ftype] != F->mnId) {
          pKFi->mnRelocWords[Ftype] = 0;
          pKFi->mnRelocQuery[Ftype] = F->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnRelocWords[Ftype]++;
      }
    }
  }
  if (lKFsSharingWords.empty())
    return std::vector<KeyFrame *>();

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (std::list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    if ((*lit)->mnRelocWords[Ftype] > maxCommonWords)
      maxCommonWords = (*lit)->mnRelocWords[Ftype];
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<pair<float, KeyFrame *>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (std::list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++) {
    KeyFrame *pKFi = *lit;

    if (pKFi->mnRelocWords[Ftype] > minCommonWords) {
      nscores++;
      float si = mpVoc->score(F->Channels[Ftype].mBowVec, pKFi->Channels[Ftype].mBowVec);
      pKFi->mRelocScore[Ftype] = si;
      lScoreAndMatch.push_back(make_pair(si, pKFi));
    }
  }

  if (lScoreAndMatch.empty())
    return std::vector<KeyFrame *>();

  std::list<pair<float, KeyFrame *>> lAccScoreAndMatch;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (std::list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++) {
    KeyFrame *pKFi = it->second;
    std::vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = bestScore;
    KeyFrame *pBestKF = pKFi;
    for (std::vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
      KeyFrame *pKF2 = *vit;
      if (pKF2->mnRelocQuery[Ftype] != F->mnId)
        continue;

      accScore += pKF2->mRelocScore[Ftype];
      if (pKF2->mRelocScore[Ftype] > bestScore) {
        pBestKF = pKF2;
        bestScore = pKF2->mRelocScore[Ftype];
      }
    }
    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
    if (accScore > bestAccScore)
      bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;
  set<KeyFrame *> spAlreadyAddedKF;
  std::vector<KeyFrame *> vpRelocCandidates;
  vpRelocCandidates.reserve(lAccScoreAndMatch.size());
  for (std::list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++) {
    const float &si = it->first;
    if (si > minScoreToRetain) {
      KeyFrame *pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpRelocCandidates;
}

} // namespace ORB_SLAM2
