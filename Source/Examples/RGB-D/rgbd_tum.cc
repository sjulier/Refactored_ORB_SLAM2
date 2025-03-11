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
#include <sysexits.h>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>

#include <System.h>

namespace fs = ::boost::filesystem;
using namespace std;

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps);

string FindFile(const string& baseFileName, const string& pathHint);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl << "Usage: " << argv[0] << " settings_files path_to_sequence path_to_association" << endl;
    return EX_USAGE;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  string strAssociationFilename = FindFile(string(argv[3]), string(DEFAULT_RGBD_SETTINGS_DIR) + "associations/");
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD,
             vTimestamps);

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if (vstrImageFilenamesRGB.empty()) {
    cerr << endl << "No images found in provided path." << endl;
    return EX_NOINPUT;
  } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return EX_DATAERR;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  string settingsFile = FindFile(string(argv[1]), string(DEFAULT_RGBD_SETTINGS_DIR));
  ORB_SLAM2::System SLAM(DEFAULT_BINARY_ORB_VOCABULARY, settingsFile,
                         ORB_SLAM2::System::RGBD, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  int main_error = EX_OK;
  thread runthread([&]() { // Start in new thread
      cv::Mat imRGB, imD;
      for (int ni = 0; ni < nImages; ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[2]) + "/" + vstrImageFilenamesRGB[ni],
                           cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[2]) + "/" + vstrImageFilenamesD[ni],
                         cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty()) {
          cerr << endl
               << "Failed to load image at: " << string(argv[2]) << "/"
               << vstrImageFilenamesRGB[ni] << endl;
          main_error = EX_DATAERR;
          break;
        }

      if (SLAM.isFinished() == true) {
	  break;
      }

      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        
      // Pass the image to the SLAM system
      SLAM.TrackRGBD(imRGB, imD, tframe);
      
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

      double ttrack =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1)
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
      }
      SLAM.StopViewer();
    });

  // Start the visualization thread; this blocks until the SLAM system
  // has finished.
  SLAM.StartViewer();

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
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return main_error;
}

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps) {
  if (fs::exists(strAssociationFilename) == false) {
    cerr << "FATAL: Could not find the association file file " << strAssociationFilename << endl;
    exit(EX_DATAERR);
  }

  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
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
