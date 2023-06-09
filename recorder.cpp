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

#include "serialib/lib/serialib.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <time.h> 
#include <fstream>
#include <sys/stat.h>

#include <thread>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <thread> 
#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/highgui/highgui_c.h>
#endif
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/core/utils/rate_estimator.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_device_control.h>


static const int ESCAPE = 27;
static const int SPACE  = 32;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

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
#if CV_MAJOR_VERSION >= 3 && CV_MINOR_VERSION >= 2
    if (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) == 0) {
#else
    if (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) != 0) {
#endif
        return true;
    }
    return false;
}

int setup_cd_callback_and_window(Metavision::Camera &camera, cv::Mat &cd_frame, Metavision::timestamp &cd_frame_ts,
                                 Metavision::CDFrameGenerator &cd_frame_generator,
                                 Metavision::RateEstimator &cd_rate_estimator, const std::string &window_name) {
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
    cv::namedWindow(window_name, CV_GUI_EXPANDED);
    cv::resizeWindow(window_name, geometry.width(), geometry.height());
    cv::moveWindow(window_name, 0, 0);
#if (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION * 100 + CV_SUBMINOR_VERSION) >= 408) || \
    (CV_MAJOR_VERSION == 4 && (CV_MINOR_VERSION * 100 + CV_SUBMINOR_VERSION) >= 102)
    cv::setWindowProperty(window_name, cv::WND_PROP_TOPMOST, 1);
#endif
    return id;
}

namespace {
std::atomic<bool> signal_caught{false};

[[maybe_unused]] void sig_handler(int s) {
    MV_LOG_TRACE() << "Interrupt signal received." << std::endl;
    signal_caught = true;
}
} // anonymous namespace


int find_free_serial_port(char *buff){
    for(int i = 0; i < 99; i++){
        snprintf(buff, 20, "/dev/ttyUSB%d", i);
        
        struct stat buffer;

        if(stat(buff, &buffer) == 0){
            return 0;
        }
    }

    return 1;
}

std::string get_current_time_file_name(const std::string &output_path, const std::string &ending){

    time_t     now = time(0);
    struct tm  tstruct;

    tstruct = *localtime(&now);

    char file_name[256];

    strftime(file_name, 255, "%Y_%m_%d_%H%M%S", &tstruct);
    strncat(file_name, ending.c_str(), 255);

     
    fs::path dir (output_path);
    fs::path file (file_name);
    fs::path full_path = dir / file;

    return full_path.string();
}


std::mutex record_m;
bool record = false;

int process_IMU(const std::string &custom_serial_port, const std::string &output_path)
{
    // Serial object
    serialib serial;
    const int SERIAL_BUFFER_SIZE = 1024;
    char buffer[SERIAL_BUFFER_SIZE];
    char serial_port[64];

    // Connection to serial port

    if(custom_serial_port.empty()){

        if(find_free_serial_port(serial_port)){
            std::cerr << "Could not find a free serial port" << std::endl;
            return 1;
        }
    } else {
        memcpy(serial_port, custom_serial_port.c_str(), custom_serial_port.size() + 1);
    }

    char errorOpening = serial.openDevice(serial_port, 115200);


    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1) {
        std::cerr << "Error openin " << serial_port << std::endl;
        return errorOpening;
    }
    std::cout <<  "Successful connection to " << serial_port << std::endl;

    // Loop forever
    while (1)
    {
          if(serial.available()){
            serial.readString(buffer, '\n', SERIAL_BUFFER_SIZE);


            if(strncmp(buffer, "RECORD_READY\r\n", SERIAL_BUFFER_SIZE) == 0){
                std::cout << "Starting Recording" << std::endl;
                record_m.lock();
                record = true;
                record_m.unlock();

                std::string file_name = get_current_time_file_name(output_path, std::string("_imu.csv"));
                std::ofstream imuFile;
                imuFile.open(file_name);

                while(1){
                    if(serial.available()){
                        int length = serial.readString(buffer, '\n', SERIAL_BUFFER_SIZE);
                        if(strncmp(buffer, "RECORD_DONE\r\n", SERIAL_BUFFER_SIZE) == 0){
                            record_m.lock();
                            record = false;
                            record_m.unlock();

                            std::cout << "Stopping Recording" << std::endl;
                            imuFile.close();
                            break;
                        } else {
                            buffer[length - 1] = '\0';
                            buffer[length - 2] = '\n';
                            imuFile << buffer;
                        }
                    }

                }   

            }
            
          } 
    }

    // Close the serial device
    serial.closeDevice();

    return 0 ;
}






int main(int argc, char *argv[]) {
    std::string serial;
    std::string imu_serial;
    std::string biases_file;
    std::string out_raw_file_path;
    std::string out_imu_file_path;
    std::vector<uint16_t> roi;

    bool do_retry = false;

    const std::string short_program_desc(
        "Simple Synchronour recorder of IMU and Event Camera.\n");

    po::options_description options_desc("Options");
    // clang-format off
    options_desc.add_options()
        ("help,h", "Produce help message.")
        ("serial,s",          po::value<std::string>(&serial),"Serial ID of the camera.")
        ("imu_serial,i",          po::value<std::string>(&imu_serial),"IMU Serial device (/dev/ttyUSB0), otherwise one is picked automatically.")
        ("biases,b",          po::value<std::string>(&biases_file), "Path to a biases file. If not specified, the camera will be configured with the default biases.")
        ("raw-out,o", po::value<std::string>(&out_raw_file_path)->default_value("events"), "Folder to output RAW file used for data recording. Default value is 'events'.")
        // ("imu-out,m", po::value<std::string>(&out_imu_file_path)->default_value("imu"), "Folder to output IMU file used for data recording. Default value is 'imu'.")
        ("roi,r",             po::value<std::vector<uint16_t>>(&roi)->multitoken(), "Hardware ROI to set on the sensor in the format [x y width height].")
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
        if (roi.size() != 4) {
            MV_LOG_WARNING() << "ROI as argument must be in the format 'x y width height '. ROI has not been set.";
            roi.clear();
        }
    }

    // Setup IMU thread



    // std::thread imu_thread (process_IMU, imu_serial, out_imu_file_path); 


    // Setup Camera

    do {
        Metavision::Camera camera;
        bool camera_is_opened = false;


        try {
            if (!serial.empty()) {
                camera = Metavision::Camera::from_serial(serial);
            } else {
                camera = Metavision::Camera::from_first_available();
            }

            if (biases_file != "") {
                camera.biases().set_from_file(biases_file);
            }

            if (!roi.empty()) {
                camera.roi().set({roi[0], roi[1], roi[2], roi[3]});
            }

            Metavision::I_TriggerIn *i_trigger_in = camera.get_device().get_facility<Metavision::I_TriggerIn>();
            Metavision::I_DeviceControl *i_dev_ctrl = camera.get_device().get_facility<Metavision::I_DeviceControl>();

            if(i_trigger_in && i_dev_ctrl){
                i_trigger_in->enable(0);
                i_dev_ctrl->set_mode_master();
                std::cout << "Trigger Enabled: " << i_trigger_in->is_enabled(0) << std::endl;
            }

            camera_is_opened = true;
        } catch (Metavision::CameraException &e) { MV_LOG_ERROR() << e.what(); }
        

        if (!camera_is_opened) {
            if (do_retry) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                MV_LOG_INFO() << "Trying to reopen camera...";
                continue;
            } else {
                return -1;
            }
        } else {
            MV_LOG_INFO() << "Camera has been opened successfully.";
        }

        // Add runtime error callback
        camera.add_runtime_error_callback([&do_retry](const Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            do_retry = true;
        });

        // Get the geometry of the camera
        auto &geometry = camera.geometry(); // Get the geometry of the camera

        // // All cameras have CDs events
        std::string cd_window_name("CD Events");
        cv::Mat cd_frame;
        Metavision::timestamp cd_frame_ts{0};
        Metavision::CDFrameGenerator cd_frame_generator(geometry.width(), geometry.height());
        cd_frame_generator.set_display_accumulation_time_us(10000);

        double avg_rate, peak_rate;
        Metavision::RateEstimator cd_rate_estimator(
            [&avg_rate, &peak_rate](Metavision::timestamp ts, double arate, double prate) {
                avg_rate  = arate;
                peak_rate = prate;
            },
            100000, 1000000, true);
        int cd_events_cb_id = setup_cd_callback_and_window(camera, cd_frame, cd_frame_ts, cd_frame_generator,
                                                           cd_rate_estimator, cd_window_name);

        // Start the camera streaming
        camera.start();

        bool record  = false;

        bool is_roi_set    = true;
        bool osc_available = false;
        bool osc_ready     = false;
        bool osd           = false;

        std::string raw_file_name;


        while (!signal_caught && camera.is_running()) {

            

            if (!cd_frame.empty()) {
                std::string text;
                if (osd) {
                    text = human_readable_time(cd_frame_ts) + " / " +
                           human_readable_time(camera.offline_streaming_control().get_duration());
                } else {
                    text = human_readable_time(cd_frame_ts);
                }
                text += "     ";
                text += human_readable_rate(avg_rate);
                cv::putText(cd_frame, text, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(108, 143, 255), 1,
                            cv::LINE_AA);
                cv::imshow(cd_window_name, cd_frame);
            }
            int key = process_ui_for(33);
            switch (key) {
                case 'q':
                case ESCAPE:
                camera.stop();
                do_retry = false;
                break;

                case SPACE:


                    if (!record) {
                        raw_file_name = get_current_time_file_name(out_raw_file_path, std::string("_events.raw"));
                        MV_LOG_INFO() << "Started recording RAW in" << raw_file_name;
                        camera.start_recording(raw_file_name);
                        record = true;
                    } else {
                        MV_LOG_INFO() << "Stopped recording RAW in" << out_raw_file_path;
                        camera.stop_recording();
                        record = false;
                    }
                    break;



            }
        }


    } while (!signal_caught && do_retry);

    return signal_caught;
}
