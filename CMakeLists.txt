# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                                                 #
# Copyright (c) 2018, William C. Lenthe                                           #
# All rights reserved.                                                            #
#                                                                                 #
# Redistribution and use in source and binary forms, with or without              #
# modification, are permitted provided that the following conditions are met:     #
#                                                                                 #
# 1. Redistributions of source code must retain the above copyright notice, this  #
#    list of conditions and the following disclaimer.                             #
#                                                                                 #
# 2. Redistributions in binary form must reproduce the above copyright notice,    #
#    this list of conditions and the following disclaimer in the documentation    #
#    and/or other materials provided with the distribution.                       #
#                                                                                 #
# 3. Neither the name of the copyright holder nor the names of its                #
#    contributors may be used to endorse or promote products derived from         #
#    this software without specific prior written permission.                     #
#                                                                                 #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"     #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE       #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE    #
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL      #
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR      #
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,   #
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.            #
#                                                                                 #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

# download + build fftw
set(FFTW_VER "3.3.7")
include(ExternalProject)
if(WIN32)
	ExternalProject_add(fftwf
		PREFIX fftwf
		URL "http://www.fftw.org/fftw-${FFTW_VER}.zip"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF -DENABLE_FLOAT=ON -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/fftwf
	)
	ExternalProject_add(fftw
		PREFIX fftw
		URL "http://www.fftw.org/fftw-${FFTW_VER}.zip"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/fftw
	)
	ExternalProject_add(fftwl
		PREFIX fftwl
		URL "http://www.fftw.org/fftw-${FFTW_VER}.zip"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CMAKE_ARGS -DBUILD_SHARED_LIBS=OFF -DENABLE_LONG_DOUBLE=ON -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/fftwl
	)
elseif(UNIX) # download + build fftw from source
	ExternalProject_add(fftwf
		PREFIX fftwf
		URL "http://www.fftw.org/fftw-${FFTW_VER}.tar.gz"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/fftwf/src/fftwf/configure
			--prefix=${CMAKE_CURRENT_BINARY_DIR}/fftwf
			--enable-type-prefix
			--disable-fortran
			--enable-float
		)
	ExternalProject_add(fftw
		PREFIX fftw
		URL "http://www.fftw.org/fftw-${FFTW_VER}.tar.gz"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/fftw/src/fftw/configure
			--prefix=${CMAKE_CURRENT_BINARY_DIR}/fftw
			--disable-fortran
			--enable-float
		)
	ExternalProject_add(fftwl
		PREFIX fftwl
		URL "http://www.fftw.org/fftw-${FFTW_VER}.tar.gz"
		DOWNLOAD_NAME "fftw-${FFTW_VER}.tar.gz"
		CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/fftwl/src/fftwl/configure
			--prefix=${CMAKE_CURRENT_BINARY_DIR}/fftwl
			--enable-type-prefix
			--disable-fortran
			--enable-float
		)
endif(WIN32)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/fftw/include)
find_library(FFTW_LIBRARY_F NAMES fftw3f HINTS "${CMAKE_CURRENT_BINARY_DIR}/fftwf/lib")
find_library(FFTW_LIBRARY_F NAMES fftw3  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/fftw/lib")
find_library(FFTW_LIBRARY_F NAMES fftw3l HINTS "${CMAKE_CURRENT_SOURCE_DIR}/fftwl/lib")

# find nidaqmx
find_path(NIDAQmx_INCLUDE_DIR NIDAQmx.h ${CMAKE_CURRENT_SOURCE_DIR})
find_library(NIDAQmx_LIBRARY NIDAQmx ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${NIDAQmx_INCLUDE_DIR})

add_executable(align align.cpp)
add_dependencies(align fftwf fftw fftwl)

target_link_libraries(align ${FFTW_LIBRARY_F} ${FFTW_LIBRARY_D} ${FFTW_LIBRARY_L})

add_executable(scan scan.cpp)
target_link_libraries(scan ${NIDAQmx_LIBRARY})
