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