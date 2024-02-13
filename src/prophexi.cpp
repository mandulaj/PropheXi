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

#include <cstdio>
#include <thread>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <thread> 
#include <string> 
#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

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




fs::path prepare_new_directory(const std::string &output_dir, const std::string &note){

    time_t     now = time(0);
    struct tm  tstruct;

    tstruct = *localtime(&now);

    char file_name_time[256];

    strftime(file_name_time, 255, "%Y_%m_%d_%H%M%S", &tstruct);

    char file_name[256];
    snprintf(file_name, 255, "%s_%s", file_name_time, note.c_str());

     
    fs::path dir (output_dir);
    fs::path sub_dir (file_name);
    fs::path full_path = dir / sub_dir;
    fs::create_directories(full_path);

    return full_path;
}



void load_prophexi_config_file(std::string config_yaml_file, Ximea_config &xi_config, Prophesee_config &proph_R_config, Prophesee_config &proph_L_config){
    std::ifstream yaml_fstream(config_yaml_file);
    YAML::Node config = YAML::Load(yaml_fstream);

    if (config["ximea"]) {
        if (config["ximea"]["fps"])
            xi_config.fps = config["ximea"]["fps"].as<int>();
        if (config["ximea"]["ae_max_lim"])
            xi_config.ae_max_lim = config["ximea"]["ae_max_lim"].as<int>();
        if (config["ximea"]["ag_max_lim"])
            xi_config.ag_max_lim = config["ximea"]["ag_max_lim"].as<float>();
        if (config["ximea"]["aeag_level"])
            xi_config.aeag_level = config["ximea"]["aeag_level"].as<int>();
        if (config["ximea"]["exp_priority"])    
            xi_config.exp_priority = config["ximea"]["exp_priority"].as<float>();
        if (config["ximea"]["ae_manual"])
            xi_config.ae_enabled = !config["ximea"]["ae_manual"].as<bool>();
    }

    if (config["ev_right"])
        set_prophesee_config( proph_R_config, config["ev_right"]);

    if (config["ev_left"])
        set_prophesee_config( proph_L_config, config["ev_left"]);

}



int main(int argc, char *argv[]) {

    Ximea_config xi_config;
    Prophesee_config proph_R_config;
    Prophesee_config proph_L_config;

    bool run_gui;
    bool manual_ae;
    std::string output_dir;
    std::string note;
    std::string config_yaml_file;

    const std::string short_program_desc(
        "Simple Synchronour recorder of IMU and Event Camera.\n");

    po::options_description options_desc("Options");
    // clang-format off
    options_desc.add_options()
        ("help,h", "Produce help message.")

        // Prophesee Camera
        ("serial_right",     po::value<std::string>(&proph_R_config.serial)->default_value("00050963"),"Serial ID of the Right camera.")
        ("serial_left",      po::value<std::string>(&proph_L_config.serial)->default_value("00050964"),"Serial ID of the Left camera.")
        ("master_right",     po::bool_switch(&proph_R_config.master)->default_value(true), "Right camera master")
        ("master_left",      po::bool_switch(&proph_L_config.master)->default_value(false), "Left camera master")
        ("biases_right",     po::value<std::string>(&proph_R_config.biases_file), "Path to a biases file for Right camera.")
        ("biases_left",      po::value<std::string>(&proph_L_config.biases_file), "Path to a biases file For Left camera.")
        ("roi_right",        po::value<std::vector<uint16_t>>(&proph_R_config.roi)->multitoken(), "Right Hardware ROI to set on the sensor in the format [x y width height].")
        ("roi_left",         po::value<std::vector<uint16_t>>(&proph_L_config.roi)->multitoken(), "Left Hardware ROI to set on the sensor in the format [x y width height].")
        
        ("config",     po::value<std::string>(&config_yaml_file)->default_value("config/default.yaml"),"Serial ID of the Right camera.")
        
        ("output_dir,o",    po::value<std::string>(&output_dir)->default_value("output"), "Output Destination directory")
        
        ("lenses,l",          po::value<std::string>(&note)->default_value("config/lenses.json"), "File containing inforamtion on the lenses")
        ("note,n",          po::value<std::string>(&note)->default_value(""), "Any notes to add to the recordings")
        
        
        ("run_gui,g",        po::bool_switch(&run_gui)->default_value(true), "Run Gui")

        // Ximea camera
        ("fps",             po::value<int>(&xi_config.fps)->default_value(60), "Ximea Framerate [Hz]")
        ("ae_manual",        po::bool_switch(&manual_ae)->default_value(false), "Used Manual Exposure and Gain values")
	    ("ae_max_lim",      po::value<int>(&xi_config.ae_max_lim)->default_value(16000), "Ximea Max Exposure Time Limit [us]")
        ("ag_max_lim",             po::value<float>(&xi_config.ag_max_lim)->default_value(5.5), "Ximea Max Gain [dB]")
        ("level",             po::value<int>(&xi_config.aeag_level)->default_value(30), "Ximea Target Level [%]")
        ("exp_pri",             po::value<float>(&xi_config.exp_priority)->default_value(0.8), "Ximea Exposure Priority 0-1.0")
        // ("imu_serial,i",          po::value<std::string>(&config_data.imu_serial),"IMU Serial device (/dev/ttyUSB0), otherwise one is picked automatically.")
        // ("biases,b",         po::value<std::string>(&proph_R_config.biases_file), "Path to a biases file. If not specified, the camera will be configured with the default biases.")
        // ("raw-out,o", po::value<std::string>(&config_data.out_raw_file_path)->default_value("events"), "Folder to output RAW file used for data recording. Default value is 'events'.")
        // ("imu-out,m", po::value<std::string>(&out_imu_file_path)->default_value("imu"), "Folder to output IMU file used for data recording. Default value is 'imu'.")
        ("erc",              po::bool_switch(&proph_L_config.erc)->default_value(true), "ERC on prophesee cameras")
        ("erc_rate",         po::value<uint32_t>(&proph_L_config.erc_rate)->default_value(100), "ERC Rate Mev/s")
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

    // check if file exists
    if (!fs::exists(config_yaml_file)) {
        std::cerr << "Config file does not exist: " << config_yaml_file << std::endl;
        return 1;
    }

    // load Prophesee config file

    load_prophexi_config_file(config_yaml_file, xi_config, proph_R_config, proph_L_config);
   
    // ERC is the same for both cameras
    proph_R_config.erc = proph_L_config.erc;
    proph_L_config.erc_rate *= 1000000;
    proph_R_config.erc_rate = proph_L_config.erc_rate;
    xi_config.ae_enabled = !manual_ae;


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


    std::vector<Device*> cameras = {&xi_cam, &proph_L_cam, &proph_R_cam};


    xi_cam.start();
    proph_L_cam.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Give Master time to turn on
    proph_R_cam.start();


    UI ui(cameras);

    ui.start();

    bool recording = false;

    while (true) {
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "quit"){
            std::cout << "Quitting" << std::endl;

            ui.stop();

            xi_cam.stop();
            proph_R_cam.stop();
            proph_L_cam.stop(); 
            return 0;
        } else {
            if (!recording) {

                std::string note;

                if(input != ""){
                    note = input;
                    std::replace(note.begin(), note.end(), ' ', '_');
                } else {
                    note = "recording";
                }

                fs::path new_path = prepare_new_directory(output_dir, note);

                char message[2048];
                std::snprintf(message, sizeof(message), "arecord -f S32_LE -c 1 -r 44100 -t wav -d 0 -q %s/recording.wav &", new_path.c_str());
                std::system(message);
                
                

                proph_L_cam.start_recording(new_path);
                proph_R_cam.start_recording(new_path);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
       
                xi_cam.start_recording(new_path);
                std::cout << "Recording started in " << new_path.string() << std::endl;
                recording = true;
            } else {

                xi_cam.stop_recording();

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                proph_R_cam.stop_recording();
                proph_L_cam.stop_recording();

                std::system("pkill -f arecord");

                std::cout << "Stopped recording." << std::endl;
                recording = false;
            }
        }
    }
    
    return 0;
}
