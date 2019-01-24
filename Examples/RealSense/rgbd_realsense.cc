/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

cv::Mat frame_to_mat(const rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		cvtColor(r, r, CV_RGB2BGR);
		return r;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}

	throw std::runtime_error("Frame format is not supported yet!");
}


int main(int argc, char **argv)
{
	if (argc != 3)
	{
		cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
		return 1;
	}

	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

	rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
	// Create a simple OpenGL window for rendering:
	//window app(1280, 720, "RealSense Capture Example");

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	// Declare rates printer for showing streaming rates of the enabled streams.
	rs2::rates_printer printer;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	// Start streaming with default recommended configuration
	// The default video configuration contains Depth and Color streams
	// If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
	pipe.start();

	try{
		while (true) // Application still alive?
		{
			rs2::frameset data = pipe.wait_for_frames();
			//.    // Wait for next set of frames from the camera
			//apply_filter(printer).     // Print each enabled stream frame rate
			//apply_filter(color_map);   // Find and colorize the depth data

            // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
            // Each texture is displayed on different viewport according to it's stream unique id
            //app.show(data);

			auto depth = data.get_depth_frame();
			auto color = data.get_color_frame();

			cv::Mat imRGB, imD;

			imRGB = frame_to_mat(color);
			imD = frame_to_mat(depth);


			auto tframe = data.get_timestamp();
			SLAM.TrackRGBD(imRGB, imD, tframe);

		}
		//return EXIT_SUCCESS;
		//break;
	}
	catch (const rs2::error & e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		//return EXIT_FAILURE;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		//return EXIT_FAILURE;
	}

	// Stop all threads
	SLAM.Shutdown();
}
