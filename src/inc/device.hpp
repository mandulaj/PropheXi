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

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>


#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;



class Device {

public:
    Device() : paused(true), stopped(false) {}
    
    ~Device() {
        stop();
    }

    void start() {
        init();

        thread = std::thread(&Device::run, this);
    }

    void stop_recording() {
        std::unique_lock<std::mutex> lock(mutex);
        paused = true;
        condition.notify_one();
    }

    void start_recording(fs::path path) {
        std::unique_lock<std::mutex> lock(mutex);
        
        prepare_recording(path);

        paused = false;
        condition.notify_one();
    }
    void stop() {
        std::unique_lock<std::mutex> lock(mutex);
        stopped = true;
        condition.notify_one();
        lock.unlock();

        if (thread.joinable()) {
            thread.join();
        }
    }

protected:
    std::thread thread;
    std::mutex mutex;
    std::condition_variable condition;
    std::atomic_bool paused;
    std::atomic_bool stopped;

    std::mutex frame_mutex;

    std::string path_root;
    std::string record_dir;
    std::string record_prefix;
    std::string record_suffix;
    std::string file_ext;

    fs::path destination_path;


    virtual void init() = 0;
    virtual void run() = 0;
    virtual void prepare_recording(fs::path path) = 0;
     
    /*
    void run() {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex);
            condition.wait(lock, [this]() { return !paused || stopped; });
            if (stopped) {
                return;
            }
            std::cout << "Thread " << std::this_thread::get_id() << " running..." << std::endl;
        }
    }
    */



};