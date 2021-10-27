/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImagesAndTimestamps(
	const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_huizhou path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    string path2voc = argv[1];
    string path2setting = argv[2];
    string path2imgs = argv[3];
    cv::FileStorage frameSettings(path2setting, cv::FileStorage::READ);

	const int endFrame = (int)frameSettings["traj.EndFrame"];
	const int startFrame = (int)frameSettings["traj.StartFrame"];

    // Retrieve paths to images
	vector<vector<string>> imgFilenames;
	vector<double> timestamps;
	LoadImagesAndTimestamps(startFrame, endFrame, path2imgs, imgFilenames, timestamps);

	int nImages = imgFilenames[0].size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(path2voc,path2setting,ORB_SLAM3::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(imgFilenames[0][ni],cv::IMREAD_UNCHANGED);
        double tframe = timestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << imgFilenames[0][ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vector<ORB_SLAM3::IMU::Point>(), imgFilenames[0][ni]);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = timestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-timestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

void LoadImagesAndTimestamps(const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps)
{
	std::cout << "load images and timestamps " << std::endl;
	vstrImageFilenames.resize(4);
	ifstream fTimes;
	string strPathTimeFile = path2imgs + "/Camera_2021-03-08_10-45-56_timestamp.csv";

	std::cout << "str to timestamp file: " << strPathTimeFile << std::endl;
	fTimes.open(strPathTimeFile.c_str());
	string line;


	int cnt = 1;
	int img_cnt = 0;
	std::cout << "start, end: " << startFrame << ", " << endFrame << std::endl; 
	while (std::getline(fTimes, line))
	{
		if (cnt >= startFrame && cnt < endFrame) // skip until startframe
		{
			std::istringstream iss(line);
			double timestamp;
			string pathimg1, pathimg2, pathimg3, pathimg4;
			if (!(iss >> timestamp >> pathimg1 >> pathimg2 >> pathimg3 >> pathimg4))
				break;
			vTimestamps.push_back(timestamp);
			vstrImageFilenames[0].push_back(path2imgs + "/Camera_2021-03-08_10-45-56_Right/" + pathimg3);
			vstrImageFilenames[1].push_back(path2imgs + "/Camera_2021-03-08_10-45-56_Front/" + pathimg1);
			vstrImageFilenames[2].push_back(path2imgs + "/Camera_2021-03-08_10-45-56_Left/" + pathimg2);
			vstrImageFilenames[3].push_back(path2imgs + "/Camera_2021-03-08_10-45-56_Rear/" + pathimg4);

			img_cnt ++;
		}
		++cnt;

	}
	std::cout << "timstamp loaded: " << cnt << std::endl;
	std::cout << "img loaded: " << img_cnt << std::endl;
}
