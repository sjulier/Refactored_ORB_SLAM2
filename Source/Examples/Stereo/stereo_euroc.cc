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
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sysexits.h>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>

#include <System.h>

namespace fs = ::boost::filesystem;
using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

string FindFile(const string& baseFileName, const string& pathHint);

int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl
         << "Usage: ./stereo_euroc settings_file path_to_left_folder "
            "path_to_right_folder path_to_times_file"
         << endl;
    return EX_USAGE;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimeStamp;
  string timeStampsFile = string(DEFAULT_STEREO_SETTINGS_DIR) + string("EuRoC_TimeStamps/") + string(argv[4]);
  
  LoadImages(string(argv[2]), string(argv[3]), timeStampsFile, vstrImageLeft,
             vstrImageRight, vTimeStamp);

  if (vstrImageLeft.empty() || vstrImageRight.empty()) {
    cerr << "ERROR: No images in provided path." << endl;
    return EX_NOINPUT;
  }

  if (vstrImageLeft.size() != vstrImageRight.size()) {
    cerr << "ERROR: Different number of left and right images." << endl;
    return EX_DATAERR;
  }

  // Read rectification parameters
  string settingsFile = FindFile(string(argv[1]), string(DEFAULT_STEREO_SETTINGS_DIR));
    
  cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return EX_DATAERR;
  }

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;

  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;

  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;
  
  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  cout << "Config: " << rows_l << "," << cols_l << "," << rows_r << "," << cols_r << endl;
  

  if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() ||
      R_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 || rows_r == 0 ||
      cols_l == 0 || cols_r == 0) {
    cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
         << endl;
    return EX_DATAERR;
  }

  cv::Mat M1l, M2l, M1r, M2r;
  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.

  ORB_SLAM2::System SLAM(DEFAULT_BINARY_ORB_VOCABULARY, settingsFile,
                         ORB_SLAM2::System::STEREO, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  int main_error = EX_OK;
  thread runthread([&]() { // Start in new thread

  cv::Mat imLeft, imRight, imLeftRect, imRightRect;
  for (int ni = 0; ni < nImages; ni++) {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

    if (imLeft.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
        main_error = EX_DATAERR;
        break;
    }

    if (imRight.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(vstrImageRight[ni]) << endl;
        main_error = EX_DATAERR;
        break;
    }

      if (SLAM.isFinished() == true) {
	  break;
      }

    
    cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

    double tframe = vTimeStamp[ni];

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeftRect, imRightRect, tframe);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

    double ttrack =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1)
            .count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vTimeStamp[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimeStamp[ni - 1];

    if (ttrack < T)
      this_thread::sleep_for(chrono::duration<double>(T - ttrack));
  }
  SLAM.StopViewer();
    });

  // Start the visualization thread; this blocks until the SLAM system
  // has finished.
  SLAM.StartViewer();
  
  cout << "Viewer started, waiting for thread." << endl;

  runthread.join();
  
  if (main_error != EX_OK)
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
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

  return main_error;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
  // Check the file exists
  if (fs::exists(strPathTimes) == false) {
    cerr << "FATAL: Could not find the EuRoC Timestamp file file " << strPathTimes << endl;
    exit(EX_DATAERR);
  }

  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
      vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
    }
  }
}

string FindFile(const string& baseFileName, const string& pathHint)
{
  fs::path baseFilePath(baseFileName);
  
  // If we can find it, return it directly
  if (fs::exists(baseFileName) == true)
    {
      return baseFileName;
    }

  // Apply the path hind and see if that works
  string candidateFilename = pathHint + baseFileName;
  
  if (fs::exists(candidateFilename) == true)
    {      
      return candidateFilename;
    }

  // Couldn't find; return the path directly and maybe the ORBSLAM instance can still find it
  return baseFileName;
}
