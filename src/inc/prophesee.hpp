#pragma once

#include "device.hpp"
#include <vector>


#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/core/utils/rate_estimator.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>


#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;


struct Prophesee_config{
    std::string serial;
    std::string biases_file;
    std::vector<uint16_t> roi;
    bool master;
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


