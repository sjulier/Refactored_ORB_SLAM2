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
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sysexits.h>
#include <boost/filesystem.hpp>

#include "System.h"

namespace fs = ::boost::filesystem;
using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

string FindFile(const string& baseFileName, const string& pathHint);

int main(int argc, char **argv) {
  if (argc != 5) {
    cerr << endl
          << "Usage: " << argv[0] << " settings_files path_to_image_folder path_to_times_file  results_file" << endl;
    return EX_USAGE;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  // Resolve the timestamp file the SAME way the settings file is resolved
  // (FindFile), rather than blindly concatenating the installed settings
  // directory onto argv[3]. Concatenation accepted ONLY a bare filename and
  // silently produced a spliced, nonsensical path for anything else -- so
  // passing a perfectly good absolute path failed with a confusing error.
  const string timesHint =
      string(DEFAULT_MONO_SETTINGS_DIR) + string("EuRoC_TimeStamps/");
  string timeStampsFile = FindFile(string(argv[3]), timesHint);

  // FindFile hands back its input unchanged when it resolves nothing, so say
  // exactly what was asked for, where it was looked for, and what form is
  // expected. "File not found" alone leaves the reader with nowhere to go.
  if (fs::exists(timeStampsFile) == false) {
    cerr << "FATAL: could not find the EuRoC timestamp file." << endl
         << "  requested: " << argv[3] << endl
         << "  looked in: the path as given, then " << timesHint << endl
         << "  expected:  a bare name such as MH01.txt, or a path to one"
         << endl;
    return EX_DATAERR;
  }

  LoadImages(string(argv[2]), timeStampsFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  if (nImages <= 0) {
    cerr << "ERROR: Failed to load images" << endl;
    return EX_DATAERR;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  string settingsFile = FindFile(string(argv[1]), string(DEFAULT_MONO_SETTINGS_DIR));

  // Off-thread g2o-graph dump prefix derived from the images-dir basename
  // (e.g. "/.../MH_01/cam0/data" → "euroc_data" — coarse but unique-per-run
  // assuming the user re-uses this launcher with one path at a time;
  // override by editing this line if you need a finer prefix).
  string logPrefix = "euroc_" + fs::path(argv[2]).filename().string();

  ORB_SLAM2::System SLAM(DEFAULT_ORB_VOCABULARY, settingsFile,
                         ORB_SLAM2::System::MONOCULAR, true, logPrefix);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  int main_error = EX_OK;
  thread runthread([&]() { // Start in new thread
    // Main loop
    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++) {
      // Read image from file
      im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
        main_error = EX_DATAERR;
        break;
      }

      if (SLAM.isFinished() == true) {
	  break;
      }

      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im, tframe);

      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

      double ttrack =
          chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages - 1)
        T = vTimestamps[ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[ni - 1];

      if (ttrack < T)
        this_thread::sleep_for(chrono::duration<double>(T - ttrack));
    }
    SLAM.StopViewer();
  });

  SLAM.StartViewer();
  cout << "Viewer started, waiting for thread." << endl;

  runthread.join();
  if (main_error != EX_OK)
    return main_error;
  cout << "Tracking thread joined..." << endl;

  // Stop all threads
  SLAM.Shutdown();

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
  //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM(string(argv[4]));

  return main_error;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps) {
  // Check the file exists
  if (fs::exists(strPathTimes) == false) {
    cerr << "FATAL: Could not find the EuRoC Timestamp file file " << strPathTimes << endl;
    exit(EX_DATAERR);
  }

  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
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
