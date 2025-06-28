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

#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

namespace fs = ::boost::filesystem;
using namespace ::std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 3) {
    cerr << endl
         << "Usage: ./stereo_kitti path_to_settings path_to_sequence" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Settings
  string settingsFile =
      string(DEFAULT_STEREO_SETTINGS_DIR) + string("/") + string(argv[1]);

  // Load both ORB and GCN vocabulary file whether or not "USE_ORB" is detected
  //const int Ntype = 1;
  //string vocabularyFile[Ntype];

  //vocabularyFile[0] = DEFAULT_BINARY_ORB_VOCABULARY;
  //vocabularyFile[1] = DEFAULT_BINARY_ORB_VOCABULARY;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(settingsFile, ORB_SLAM2::System::STEREO, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  int main_error = 0;
  std::thread runthread([&]() {
    cv::Mat imLeft, imRight;
    for (int ni = 0; ni < nImages; ni++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[ni])
             << endl;
        return 1;
      }

      if (SLAM.isFinished() == true) {
        break;
      }

      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();

      // Pass the images to the SLAM system
      SLAM.TrackStereo(imLeft, imRight, tframe);

      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages - 1)
        T = vTimestamps[ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[ni - 1];

      if (ttrack < T)
        this_thread::sleep_for(chrono::duration<double>(T - ttrack));
      //            usleep((T-ttrack)*1e6);
    }
    SLAM.StopViewer();
  });

  // Start the visualization thread; this blocks until the SLAM system
  // has finished.
  SLAM.StartViewer();

  runthread.join();

  if (main_error != 0)
    return main_error;

  // Stop all threads
  SLAM.Shutdown();
  cout << "System Shutdown" << endl;

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";

  // Check the file exists
  if (fs::exists(strPathTimeFile) == false) {
    cerr << "FATAL: Could not find the timestamp file " << strPathTimeFile
         << endl;
    exit(0);
  }

  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";
  string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}
