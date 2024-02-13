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
#include <vector>


#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/core/utils/rate_estimator.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_event_rate_activity_filter_module.h>
#include <metavision/hal/facilities/i_digital_event_mask.h>
#include <yaml-cpp/yaml.h>


#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

struct PixelCoordinates{
    uint16_t x;
    uint16_t y;
};

struct Prophesee_config{
    std::string serial;
    std::string biases_file;
    std::vector<uint16_t> roi;
    bool master;
    bool erc;
    uint32_t erc_rate;
    std::vector<PixelCoordinates> crazy_pixels;
};



class Prophesee : public Device {

public:
    Prophesee(Prophesee_config &config):  Device(), config(config) {}


private:
    Prophesee_config &config;
    Metavision::Camera camera;

    fs::path biases_output;

    void init();
    void run();
    void prepare_recording(fs::path path);

};


void set_prophesee_config(Prophesee_config &config, const YAML::Node &node);