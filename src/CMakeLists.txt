 # Copyright (c) 2023 Jakub Mandula.                                                                                       *
 #                                                                                                                    *
 # Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 # software and associated documentation files (the “Software”), to deal in the Software 
 # without restriction, including without limitation the rights to use, copy, modify, merge, 
 # publish, distribute, sublicense, and/or sell copies of the Software, and to permit 
 # persons to whom the Software is furnished to do so, subject to the following conditions:
 # 
 # The above copyright notice and this permission notice shall be included in all copies or
 # substantial portions of the Software.


set (sample prophexi)
set (common_libraries MetavisionSDK::driver MetavisionSDK::core Boost::program_options opencv_highgui)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Boost
set (boost_components_to_find program_options filesystem timer chrono thread)
find_package(Boost COMPONENTS ${boost_components_to_find} REQUIRED)

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(OpenCV 4.8.0 COMPONENTS core highgui imgproc videoio imgcodecs calib3d objdetect REQUIRED)


include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()


# Xiapi
include_directories(_libs)
include_directories(_libs/xiAPI)
link_libraries(m3api)

link_libraries(tiff)


include_directories(inc)


add_executable(${sample}
  ui.cpp
  ximea.cpp
  prophesee.cpp
  device.cpp 
  ${sample}.cpp
  )
target_link_libraries(${sample} PRIVATE ${common_libraries})
target_link_libraries(${sample} PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library
# target_link_libraries(${sample} serialib)


set_target_properties(${sample} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "../../" )


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
