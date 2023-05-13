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

#include "prophesee.hpp"

#include <unistd.h>
#include <stdio.h>
#include <memory.h>
#include <iostream>
#include <time.h> 
#include <fstream>
#include <sys/stat.h>

#include <thread>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <thread> 



#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif
#include <opencv2/imgproc.hpp>



#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

static const int ESCAPE = 27;
static const int SPACE  = 32;

std::string human_readable_rate(double rate) {
    std::ostringstream oss;
    if (rate < 1000) {
        oss << std::setprecision(0) << std::fixed << rate << " ev/s";
    } else if (rate < 1000 * 1000) {
        oss << std::setprecision(1) << std::fixed << (rate / 1000) << " Kev/s";
    } else if (rate < 1000 * 1000 * 1000) {
        oss << std::setprecision(1) << std::fixed << (rate / (1000 * 1000)) << " Mev/s";
    } else {
        oss << std::setprecision(1) << std::fixed << (rate / (1000 * 1000 * 1000)) << " Gev/s";
    }
    return oss.str();
}

std::string human_readable_time(Metavision::timestamp t) {
    std::ostringstream oss;
    std::array<std::string, 4> ls{":", ":", ".", ""};
    std::array<std::string, 4> vs;
    std::array<int, 4> ts;
    ts[3] = t % 1000000;
    vs[3] = cv::format("%06d", int(ts[3]));
    t /= 1000000; // s
    ts[2] = t % 60;
    vs[2] = cv::format("%02d", int(ts[2]));
    t /= 60; // m
    ts[1] = t % 60;
    vs[1] = cv::format("%02d", int(ts[1]));
    t /= 60; // h
    ts[0] = t;
    vs[0] = cv::format("%02d", int(ts[0]));

    size_t i = 0;
    // skip hour and minutes if t is not high enough, but keep s and us
    for (; i < 2; ++i) {
        if (ts[i] != 0) {
            break;
        }
    }
    for (; i < 4; ++i) {
        oss << vs[i] << ls[i];
    }
    return oss.str();
}

int process_ui_for(int delay_ms) {
    auto then = std::chrono::high_resolution_clock::now();
    int key   = cv::waitKey(delay_ms);
    auto now  = std::chrono::high_resolution_clock::now();
    // cv::waitKey will not wait if no window is opened, so we wait for it, if needed
    std::this_thread::sleep_for(std::chrono::milliseconds(
        delay_ms - std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count()));

    return key;
}

bool window_was_closed(const std::string &window_name) {
// Small hack: if the window has been closed, it is not visible anymore or property changes from the one we set
    if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) == 0) {
// #else
//     if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) != 0) {
// #endif
        return true;
    }
    return false;
}

int setup_cd_callback(Metavision::Camera &camera, cv::Mat &cd_frame, Metavision::timestamp &cd_frame_ts,
                                 Metavision::CDFrameGenerator &cd_frame_generator,
                                 Metavision::RateEstimator &cd_rate_estimator) {
    auto &geometry = camera.geometry();
    auto id = camera.cd().add_callback([&cd_frame_generator, &cd_rate_estimator](const Metavision::EventCD *ev_begin,
                                                                                 const Metavision::EventCD *ev_end) {
        cd_frame_generator.add_events(ev_begin, ev_end);
        cd_rate_estimator.add_data(std::prev(ev_end)->t, std::distance(ev_begin, ev_end));
    });
    cd_frame_generator.start(30, [&cd_frame, &cd_frame_ts](const Metavision::timestamp &ts, const cv::Mat &frame) {
        cd_frame_ts = ts;
        frame.copyTo(cd_frame);
    });

    return id;
}

namespace {
std::atomic<bool> signal_caught{false};

[[maybe_unused]] void sig_handler(int s) {
    MV_LOG_TRACE() << "Interrupt signal received." << std::endl;
    signal_caught = true;
}
} // anonymous namespace




void Prophesee::prepare_recording(fs::path path){
    std::string name;

    if(config.master){
        name =  "right.raw";
    } else {
        name =  "left.raw";
    }
    
    fs::path file(name);

    
    // biases_output = path / 

    // fs::create_directories(biases_output);


	destination_path = path / name;

}



void Prophesee::run(){

    // Get the geometry of the camera
    auto &geometry = camera.geometry(); // Get the geometry of the camera

    // Setup CD event rate estimator
    double avg_rate, peak_rate;
    Metavision::RateEstimator cd_rate_estimator(
        [&avg_rate, &peak_rate](Metavision::timestamp ts, double arate, double prate) {
            avg_rate  = arate;
            peak_rate = prate;
        },
        100000, 1000000, true);

    // Setup CD frame generator
    Metavision::CDFrameGenerator cd_frame_generator(geometry.width(), geometry.height());
    cd_frame_generator.set_display_accumulation_time_us(10000);

    std::mutex cd_frame_mutex;
    cv::Mat cd_frame;
    Metavision::timestamp cd_frame_ts{0};
    cd_frame_generator.start(
        30, [&cd_frame_mutex, &cd_frame, &cd_frame_ts](const Metavision::timestamp &ts, const cv::Mat &frame) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            cd_frame_ts = ts;
            frame.copyTo(cd_frame);
        });

    // Setup CD frame display
    
    // std::string cd_window_name;

    // if(config.master)
    //     cd_window_name = "Prophesee Master";
    // else
    //     cd_window_name = "Prophesee slave";


//     cv::namedWindow(cd_window_name, CV_GUI_EXPANDED);
//     cv::resizeWindow(cd_window_name, geometry.width(), geometry.height());
//     cv::moveWindow(cd_window_name, 0, 0);
// #if (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION * 100 + CV_SUBMINOR_VERSION) >= 408) || \
// (CV_MAJOR_VERSION == 4 && (CV_MINOR_VERSION * 100 + CV_SUBMINOR_VERSION) >= 102)
//     cv::setWindowProperty(cd_window_name, cv::WND_PROP_TOPMOST, 1);
// #endif

    // Setup camera CD callback to update the frame generator and event rate estimator
    int cd_events_cb_id =
        camera.cd().add_callback([&cd_frame_mutex, &cd_frame_generator, &cd_rate_estimator](
                                        const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            cd_frame_generator.add_events(ev_begin, ev_end);
            cd_rate_estimator.add_data(std::prev(ev_end)->t, std::distance(ev_begin, ev_end));
        });





		
    // Start the camera streaming
    camera.start();

    printf("Prophesee %s - %s ready\n", config.master? "Master" : "Slave", config.serial.c_str());


    while(camera.is_running()){
		
		// Wait here for recording to start
		std::unique_lock<std::mutex> lock(mutex);

		condition.wait(lock);
		if(stopped){
			break;
		}
		lock.unlock();

        

        camera.start_recording(destination_path.string());

		// Frame aquisition 
		while(true){
			
            if (!cd_frame.empty()) {
                    std::string text;
    
                    text = human_readable_time(cd_frame_ts);
                    
                    text += "     ";
                    text += human_readable_rate(avg_rate);
                    cv::putText(cd_frame, text, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(108, 143, 255), 1,
                                cv::LINE_AA);
                    // cv::imshow(cd_window_name, cd_frame);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));


			{
				std::lock_guard<std::mutex> lock(mutex);

				if(stopped || paused){
					// Stop Aquisition
                    camera.stop_recording();
					break;
				}
			}
		}

	}

    camera.stop();

}


void Prophesee::init(){
    bool camera_is_opened = false;
    bool do_retry = false;

        
    try {
        if (!config.serial.empty()) {
            camera = Metavision::Camera::from_serial(config.serial);
        } else {
            camera = Metavision::Camera::from_first_available();
        }

        if (config.biases_file != "") {
            camera.biases().set_from_file(config.biases_file);
        }

        if (!config.roi.empty()) {
            camera.roi().set({
                config.roi[0], 
                config.roi[1], 
                config.roi[2], 
                config.roi[3]});
        }

        Metavision::I_TriggerIn *i_trigger_in = camera.get_device().get_facility<Metavision::I_TriggerIn>();
        Metavision::I_CameraSynchronization *i_cam_sync = camera.get_device().get_facility<Metavision::I_CameraSynchronization>();

        if(i_trigger_in && i_cam_sync){
            if(config.master){
                i_cam_sync->set_mode_master();
            } else {
                i_cam_sync->set_mode_slave();
            }

            i_trigger_in->enable(Metavision::I_TriggerIn::Channel::Main);
            if(i_trigger_in->is_enabled(Metavision::I_TriggerIn::Channel::Main) != 1){
                throw "Trigger not enabled";
            }
        }

        camera_is_opened = true;
    } catch (Metavision::CameraException &e) { MV_LOG_ERROR() << e.what(); }
    

    if (!camera_is_opened) {
        std::cerr << "Opening camera failed." << std::endl;
        
    }


    // Add runtime error callback
    camera.add_runtime_error_callback([&do_retry](const Metavision::CameraException &e) {
        MV_LOG_ERROR() << e.what();
        do_retry = true;
    });
}