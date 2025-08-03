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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <list>
#include <set>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
// #include "ORBVocabulary.h"
#include "FbowVocabulary.h"
#include <fbow.h>
#include <unordered_map>

#include <mutex>

namespace ORB_SLAM2 {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
public:
  // KeyFrameDatabase(const ORBVocabulary &voc);
  KeyFrameDatabase(const FbowVocabulary &voc);

  void add(KeyFrame *pKF, const int Ftype);

  void erase(KeyFrame *pKF, const int Ftype);

  void clear();

  // Loop Detection
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore, const int Ftype);

  // Relocalization
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F, const int Ftype);

protected:
  // Associated vocabulary
  // const ORBVocabulary *mpVoc;
  const FbowVocabulary *mpVoc;

  // Inverted file
  std::unordered_map<uint32_t, std::list<KeyFrame*>> mInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} // namespace ORB_SLAM2

#endif
