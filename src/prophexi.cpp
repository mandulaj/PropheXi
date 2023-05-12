/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A.                                                                                       *
 *                                                                                                                    *
 * Licensed under the Apache License, Version 2.0 (the "License");                                                    *
 * you may not use this file except in compliance with the License.                                                   *
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0                                 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed   *
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.                      *
 * See the License for the specific language governing permissions and limitations under the License.                 *
 **********************************************************************************************************************/

// Example of using Metavision SDK Driver API for visualizing events stream.

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// #include "serialib/lib/serialib.h"
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
#include <string> 
#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/base/utils/log.h>


#include "prophesee.hpp"
#include "ui.hpp"
#include "ximea.hpp"
#include "device.hpp"




namespace po = boost::program_options;
namespace fs = boost::filesystem;




fs::path prepare_new_directory(const std::string &output_dir){

    time_t     now = time(0);
    struct tm  tstruct;

    tstruct = *localtime(&now);

    char file_name[256];

    strftime(file_name, 255, "%Y_%m_%d_%H%M%S", &tstruct);

     
    fs::path dir (output_dir);
    fs::path sub_dir (file_name);
    fs::path full_path = dir / sub_dir;
    fs::create_directories(full_path);

    return full_path;
}




int main(int argc, char *argv[]) {

    Ximea_config xi_config;
    Prophesee_config proph_R_config;
    Prophesee_config proph_L_config;

    bool run_gui;
    std::string output_dir;
    std::string note;

    const std::string short_program_desc(
        "Simple Synchronour recorder of IMU and Event Camera.\n");

    po::options_description options_desc("Options");
    // clang-format off
    options_desc.add_options()
        ("help,h", "Produce help message.")

        // Prophesee Camera
        ("serial_right",     po::value<std::string>(&proph_R_config.serial)->default_value("00050338"),"Serial ID of the Right camera.")
        ("serial_left",      po::value<std::string>(&proph_L_config.serial)->default_value("00050339"),"Serial ID of the Left camera.")
        ("master_right",     po::bool_switch(&proph_R_config.master)->default_value(true), "Right camera master")
        ("master_left",      po::bool_switch(&proph_L_config.master)->default_value(false), "Left camera master")
        ("biases_right",     po::value<std::string>(&proph_R_config.biases_file), "Path to a biases file for Right camera.")
        ("biases_left",      po::value<std::string>(&proph_L_config.biases_file), "Path to a biases file For Left camera.")
        ("roi_right",        po::value<std::vector<uint16_t>>(&proph_R_config.roi)->multitoken(), "Right Hardware ROI to set on the sensor in the format [x y width height].")
        ("roi_left",         po::value<std::vector<uint16_t>>(&proph_L_config.roi)->multitoken(), "Left Hardware ROI to set on the sensor in the format [x y width height].")
        
        ("output_dir,o",    po::value<std::string>(&output_dir)->default_value("output"), "Output Destination directory")
        
        ("lenses,l",          po::value<std::string>(&note)->default_value("config/lenses.json"), "File containing inforamtion on the lenses")
        ("note,n",          po::value<std::string>(&note)->default_value(""), "Any notes to add to the recordings")
        
        
        ("run_gui,g",        po::bool_switch(&run_gui)->default_value(true), "Run Gui")

        // Ximea camera
        ("fps",             po::value<int>(&xi_config.fps)->default_value(60), "Ximea Framerate [Hz]")
        ("ae_max_lim",      po::value<int>(&xi_config.ae_max_lim)->default_value(16000), "Ximea Max Exposure Time Limit [us]")
        ("ag_max_lim",             po::value<float>(&xi_config.ag_max_lim)->default_value(5.5), "Ximea Max Gain [dB]")
        ("level",             po::value<int>(&xi_config.aeag_level)->default_value(25), "Ximea Target Level [%]")
        ("exp_pri",             po::value<float>(&xi_config.exp_priority)->default_value(0.8), "Ximea Exposure Priority 0-1.0")
        // ("imu_serial,i",          po::value<std::string>(&config_data.imu_serial),"IMU Serial device (/dev/ttyUSB0), otherwise one is picked automatically.")
        // ("biases,b",         po::value<std::string>(&proph_R_config.biases_file), "Path to a biases file. If not specified, the camera will be configured with the default biases.")
        // ("raw-out,o", po::value<std::string>(&config_data.out_raw_file_path)->default_value("events"), "Folder to output RAW file used for data recording. Default value is 'events'.")
        // ("imu-out,m", po::value<std::string>(&out_imu_file_path)->default_value("imu"), "Folder to output IMU file used for data recording. Default value is 'imu'.")
    ;
    // clang-format on

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(options_desc).run(), vm);
        po::notify(vm);
    } catch (po::error &e) {
        MV_LOG_ERROR() << short_program_desc;
        MV_LOG_ERROR() << options_desc;
        MV_LOG_ERROR() << "Parsing error:" << e.what();
        return 1;
    }

    if (vm.count("help")) {
        MV_LOG_INFO() << short_program_desc;
        MV_LOG_INFO() << options_desc;
        return 0;
    }


    MV_LOG_INFO() << short_program_desc;

    if (vm.count("roi")) {
        if (proph_R_config.roi.size() != 4) {
            MV_LOG_WARNING() << "ROI as argument must be in the format 'x y width height '. ROI has not been set.";
            proph_R_config.roi.clear();
        }
    }



    // return 0;
    Ximea xi_cam(xi_config);
    Prophesee proph_R_cam(proph_R_config);
    Prophesee proph_L_cam(proph_L_config);


    XimeaTest xi1(xi_config);
    XimeaTest xi2(xi_config);
    
    // xi1.start();
    // xi2.start();
    xi_cam.start();
    proph_R_cam.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Give Master time to turn on
    proph_L_cam.start();


    bool recording = false;

    while (true) {
        std::string input;
        std::getline(std::cin, input);
        if (input == "") {
            if (!recording) {
                fs::path new_path = prepare_new_directory(output_dir);
                // xi1.start_recording(new_path);
                // xi2.start_recording(new_path);
                proph_R_cam.start_recording(new_path);
                proph_L_cam.start_recording(new_path);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
       
                xi_cam.start_recording(new_path);
                std::cout << "Recording started." << std::endl;
                recording = true;
            } else {
                // xi1.stop_recording();
                // xi2.stop_recording();
                xi_cam.stop_recording();

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                proph_R_cam.stop_recording();
                proph_L_cam.stop_recording();  
                std::cout << "Stopped recording." << std::endl;
                recording = false;
            }
        } else if (input == "q" || input == "quit"){
            std::cout << "Quitting" << std::endl;

            xi_cam.stop();
            proph_R_cam.stop();
            proph_L_cam.stop(); 
            return 0;
        }
    }
    
    return 0;
}
