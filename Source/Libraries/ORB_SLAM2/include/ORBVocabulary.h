/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "DBoW2/DBoW2.h"

namespace ORB_SLAM2 {

class ORBVocabulary : public ::OrbVocabulary {
public:
  ORBVocabulary(int k = 10, int L = 5,
                DBoW2::WeightingType weighting = DBoW2::TF_IDF,
                DBoW2::ScoringType scoring = DBoW2::L1_NORM);

  bool loadFromTextFile(const std::string &filename);

  void saveToTextFile(const std::string &filename) const;

  //  https://github.com/raulmur/ORB_SLAM2/pull/21
  /**
   * Loads the vocabulary from a binary file
   * @param filename
   */
  bool loadFromBinaryFile(const std::string &filename);

  /**
   * Saves the vocabulary into a binary file
   * @param filename
   */
  bool saveToBinaryFile(const std::string &filename) const;

private:
  using F = DBoW2::FORB;

  std::vector<unsigned char> mvDictionaryMemoryPool;
};

} // namespace ORB_SLAM2

#endif // ORBVOCABULARY_H
