#pragma once

#include "device.hpp"
#include <iostream>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

#include <opencv2/core.hpp> 
#include <m3api/xiApi.h> // Linux, OSX

void WriteImage(cv::Mat& image, char* filename);




struct Ximea_config{
    std::string serial;
    int aeag_level;
    int ae_max_lim;
    int fps;
    float exp_priority;
    float ag_max_lim;

};





class Ximea : public Device {
public:
    Ximea(Ximea_config &config):  Device(), config(config) {}

private:
    struct Ximea_config& config;

    fs::path timestamps_file;
    fs::path frames_path;

	HANDLE xiH = NULL;
    void init();
    void run();
    void prepare_recording(fs::path path);

};


class XimeaTest : public Device {
public:
    XimeaTest(Ximea_config &config):  Device(), config(config) {}

private:
    struct Ximea_config& config;

    std::string timestamps_file;
	HANDLE xiH = NULL;
    void init();
    void run();
    void prepare_recording(fs::path path);

};