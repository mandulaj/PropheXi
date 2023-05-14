/**********************************************************************************************************************
 * Copyright (c) 2023 Jakub Mandula.                                                                                       *
 *                                                                                                                    *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 * software and associated documentation files (the “Software”), to deal in the Software 
 * without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit 
 * persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 **********************************************************************************************************************/


#include "ximea.hpp"
#include "device.hpp"



#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif
#include <opencv2/imgproc.hpp>

#include <m3api/xiApi.h> // Linux, OSX
#include <tiffio.h>

#define CE(func) {XI_RETURN stat = (func); if (XI_OK!=stat) {printf("Error:%d returned from function:"#func"\n",stat);throw "Error";}}

#include <boost/filesystem.hpp>

#include <fstream>

namespace fs = boost::filesystem;


void WriteImage(cv::Mat& image, const char* filename)
{
	TIFF* tiff_img = TIFFOpen(filename, "w");
	if (!tiff_img)
		throw "Opening image by TIFFOpen";

	// set tiff tags
	int width = image.cols;
	int height = image.rows; 


	int bits_per_sample = 8;
    if(image.type() == CV_16U || image.type() == CV_16UC1 || image.type () == CV_16UC3){
        bits_per_sample = 16;
    }

    int line_len = 0;
	line_len = width * (bits_per_sample / 8);
	// printf("Saving image %dx%d to file:%s\n", width, height, filename);

	TIFFSetField(tiff_img, TIFFTAG_IMAGEWIDTH, width);
	TIFFSetField(tiff_img, TIFFTAG_IMAGELENGTH, height);
	TIFFSetField(tiff_img, TIFFTAG_ROWSPERSTRIP, height);
	
    TIFFSetField(tiff_img, TIFFTAG_BITSPERSAMPLE, bits_per_sample);
	TIFFSetField(tiff_img, TIFFTAG_MINSAMPLEVALUE, 0);
	TIFFSetField(tiff_img, TIFFTAG_MAXSAMPLEVALUE, (1 << bits_per_sample) - 1);

	TIFFSetField(tiff_img, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tiff_img, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

	if (image.type() == CV_16UC3 || image.type() == CV_8UC3){
        TIFFSetField(tiff_img, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
	    TIFFSetField(tiff_img, TIFFTAG_SAMPLESPERPIXEL, 3);
    } else {
        TIFFSetField(tiff_img, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
	    TIFFSetField(tiff_img, TIFFTAG_SAMPLESPERPIXEL, 1);
    }
	//TIFFSetField(tiff_img, TIFFTAG_COMPRESSION, COMPRESSION_LZW);
	//TIFFSetField(tiff_img, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_INT);

	// save data
	if (TIFFWriteEncodedStrip(tiff_img, 0, image.data, line_len*height) == -1)
	{
		throw("ImageFailed to write image");
	}

	TIFFWriteDirectory(tiff_img);
	TIFFClose(tiff_img);
}






void Ximea::prepare_recording(fs::path path){



	// fs::path dir(path.filename());

	destination_path = path;


	fs::path ximea ("ximea");
	fs::path ts_file ("ximea_ts.csv");

	frames_path = path / ximea;

	timestamps_file = path / ts_file;


    fs::create_directories(frames_path);
}


void Ximea::run(){
	long long last_ts = 0;

	
	int img_size_bytes = 0;
	CE(xiGetParamInt(xiH, XI_PRM_IMAGE_PAYLOAD_SIZE, &img_size_bytes));
	// unsigned char * img_buffer = (unsigned char*)malloc(img_size_bytes);

	int width, height;
	xiGetParamInt(xiH, XI_PRM_WIDTH, &width);
	xiGetParamInt(xiH, XI_PRM_HEIGHT, &height);


	cv::Mat cv_mat_image = cv::Mat(height,width,CV_16UC1);
	cv::Mat rgb_image = cv::Mat(height, width, CV_16UC3);
	

	std::cout << "Ximea ready" << std::endl;
	while(true){


		// Wait here for recording to resume
		std::unique_lock<std::mutex> lock(mutex);
		condition.wait(lock);
		if(stopped){
			break;
		}
		lock.unlock();

		std::ofstream ts_file(timestamps_file.string());
		ts_file << "frame_id," 
				<< "ts, " 
				<< "exposure, " 
				<< "gain, " 
				<< "skip_frames"				
				<< std::endl;


		
		CE(xiStartAcquisition(xiH));
		
		long long last_ts = 0;

		int frame_id = 0;

		while(true){
			
			XI_IMG image; // image buffer
			memset(&image, 0, sizeof(image));
			image.size = sizeof(XI_IMG);

			image.bp = cv_mat_image.data;
            image.bp_size = img_size_bytes;

			CE(xiGetImage(xiH, 5000, &image)); // getting next image from the camera opened



			// We record 10bit in 16bit integer. Shift to make MSB also MSB in the two bytes.
            cv::Mat shifted = cv_mat_image * (1 << 6);



            long long current_ts = image.tsSec * 1000000 + image.tsUSec;
            long long diff_us = current_ts - last_ts;
            last_ts = current_ts;

            float fps = 1e6/(diff_us);


            // float temperature = 0.0;
            // xiSetParamInt(xiH, XI_PRM_TEMP_SELECTOR, XI_TEMP_IMAGE_SENSOR_DIE_RAW); 
            // xiGetParamFloat(xiH, XI_PRM_TEMP, &temperature); 
            // printf("%f\n", temperature);


            int number_of_skipped_frames = 0;
            xiSetParamInt(xiH, XI_PRM_COUNTER_SELECTOR, XI_CNT_SEL_API_SKIPPED_FRAMES);
            xiGetParamInt(xiH, XI_PRM_COUNTER_VALUE, &number_of_skipped_frames);
	

			ts_file << std::to_string(frame_id) << ", " 
					<<  std::to_string(current_ts) << ", " 
					<< std::to_string(image.exposure_time_us/1000.0) << ", " 
					<< std::to_string(image.gain_db) << ", " 
					<< std::to_string(number_of_skipped_frames)					
					<<std::endl;

			// unsigned char pixel = *(unsigned char*)image.bp;
			if(frame_id % 64 == 0){
				printf("\rFrame %d - ts: %d.%ds fps: %f, exposure_us: %f ms, gain %f dB, skipped: %d", 
            	frame_id, image.tsSec, image.tsUSec, fps, image.exposure_time_us/1000.0, image.gain_db, number_of_skipped_frames);
				fflush(stdout);
			}
			
			

            // cv::Mat rgb = cv::Mat(shifted.rows, shifted.cols, CV_16UC3);
            // cv::cvtColor(shifted, rgb, cv::COLOR_BayerGBRG2BGR);

            //cv::imwrite(filename, shifted);
            // cv::imshow("view",rgb);
            // char c = (char)cv::waitKey(1);
            // if( c == 27 ) 
            // break;


			char filename[100] = "";
			sprintf(filename, "frame%06d.tif", frame_id);
 

			fs::path img_path = frames_path / fs::path(filename);


			const char* c_img_path = img_path.c_str();
            
			WriteImage(shifted, c_img_path);


			{
				std::lock_guard<std::mutex> lock(frame_mutex);
				out_frame = shifted.clone();
			}


			// Check if recording interrupted
			{
				std::lock_guard<std::mutex> lock(mutex);

				if(stopped || paused){
					xiStopAcquisition(xiH);
					ts_file.close();
					// Stop Aquisition

					break;
					
				}
			}
			frame_id++;
		}
	}

	xiCloseDevice(xiH);

}



void Ximea::init() {

	try {
		CE(xiOpenDevice(0, &xiH));

		xiSetParamInt(xiH, XI_PRM_DEBUG_LEVEL, XI_DL_WARNING);
		xiSetParamInt(xiH, XI_PRM_DEBUG_LEVEL, XI_DL_DISABLED);
		
		// Target Exposure Level
		CE(xiSetParamInt(xiH, XI_PRM_AEAG_LEVEL, config.aeag_level));
		// Max exposure limit < 16.6ms
		CE(xiSetParamFloat(xiH, XI_PRM_AE_MAX_LIMIT, config.ae_max_lim));
		CE(xiSetParamFloat(xiH, XI_PRM_AG_MAX_LIMIT, config.ag_max_lim));
		CE(xiSetParamFloat(xiH, XI_PRM_EXP_PRIORITY, config.exp_priority));
		CE(xiSetParamInt(xiH, XI_PRM_EXPOSURE, config.ae_max_lim));
		CE(xiSetParamInt(xiH, XI_PRM_AEAG, XI_ON));
		

		CE(xiSetParamInt(xiH,XI_PRM_BUFFER_POLICY,XI_BP_SAFE));
		CE(xiSetParamInt(xiH, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE));
		CE(xiSetParamInt(xiH,XI_PRM_FRAMERATE, config.fps));

		// CE(xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24));
		CE(xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RAW16));

	
		// GPIO Setup 
		CE(xiSetParamInt(xiH, XI_PRM_GPO_SELECTOR, 1));
		// CE(xiSetParamInt(xiH, XI_PRM_GPO_MODE,  XI_GPO_OFF));
		CE(xiSetParamInt(xiH, XI_PRM_GPO_MODE,  XI_GPO_EXPOSURE_ACTIVE));


	}
	catch (const char* err) {
		printf("Error: %s\n", err); 
	}

} 
