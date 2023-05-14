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
#include <opencv2/core.hpp> 

#include "device.hpp"



class UI {

public:
    UI(std::vector<Device*>& cameras) : stopped(false), cameras(cameras){}
    
    ~UI() {
        stop();
    }

    void start() {
        thread = std::thread(&UI::run, this);
    }

   
    void stop() {
        std::unique_lock<std::mutex> lock(mutex);
        stopped = true;
        lock.unlock();

        if (thread.joinable()) {
            thread.join();
        }
    }


private:
    std::thread thread;
    std::mutex mutex;
    std::atomic_bool stopped;
;
    std::vector<Device*>& cameras;


    void run();

};