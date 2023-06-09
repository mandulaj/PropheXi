# Copyright (c) Prophesee S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

project(sample_recorder)


set (sample recorder)
set (common_libraries MetavisionSDK::driver MetavisionSDK::core Boost::program_options opencv_highgui)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Boost
set (boost_components_to_find program_options filesystem timer chrono thread)
find_package(Boost COMPONENTS ${boost_components_to_find} REQUIRED)

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(OpenCV COMPONENTS core highgui imgproc videoio imgcodecs calib3d objdetect REQUIRED)


add_subdirectory(serialib/lib)

add_library(serialib STATIC serialib/lib/serialib.cpp)


add_executable(${sample} ${sample}.cpp)
target_link_libraries(${sample} PRIVATE ${common_libraries})
target_link_libraries(${sample} serialib)


#install(TARGETS ${sample}
#        RUNTIME DESTINATION bin
#        COMPONENT metavision-sdk-driver-bin
#)

#install(FILES ${sample}.cpp
#        DESTINATION share/metavision/sdk/driver/apps/${sample}
#        COMPONENT metavision-sdk-driver-samples
#)

#install(FILES CMakeLists.txt.install
#        RENAME CMakeLists.txt
#        DESTINATION share/metavision/sdk/driver/apps/${sample}
#        COMPONENT metavision-sdk-driver-samples
#)
