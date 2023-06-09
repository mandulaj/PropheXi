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


#include "ui.hpp"


#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif
#include <opencv2/imgproc.hpp>


void UI::run(){


    cv::namedWindow("Ximea", CV_WINDOW_NORMAL);
    cv::namedWindow("Right", CV_WINDOW_NORMAL);
    cv::namedWindow("Left", CV_WINDOW_NORMAL);


    while(true){
        std::unique_lock<std::mutex> lock(mutex);
		if(stopped){
            cv::destroyAllWindows();
			break;
		}
        lock.unlock();


        cv::Mat out_frame_right;
        cv::Mat out_frame_left;
        cv::Mat out_frame_ximea;

        out_frame_ximea = cameras[0]->get_output_frame();
        out_frame_left = cameras[1]->get_output_frame();
        out_frame_right = cameras[2]->get_output_frame();


        if(!out_frame_left.empty()){
            cv::imshow("Left", out_frame_left);
        }  
        
        if(!out_frame_right.empty()){
            cv::imshow("Right", out_frame_right);
        }        
        
        if(!out_frame_ximea.empty()){

            cv::cvtColor(out_frame_ximea, out_frame_ximea, cv::COLOR_BayerGBRG2BGR);
            
            cv::Mat out_rgb;

            out_frame_ximea.convertTo(out_rgb, CV_8UC3, 1/256.0);
            cv::imshow("Ximea", out_rgb);
        }
        
        cv::waitKey(33);
    }

}
