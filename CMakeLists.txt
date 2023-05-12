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


project(prophexi)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# Serial
# add_subdirectory(serialib/lib)
# add_library(serialib STATIC serialib/lib/serialib.cpp)

add_subdirectory(src)
