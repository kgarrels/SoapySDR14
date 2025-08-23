########################################################################
# Build Soapy SDR support module for NetSDR
########################################################################
cmake_minimum_required(VERSION 3.10)

project(SoapySDR14 CXX)

#enable c++11 features
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #disable warnings for unused parameters
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-overloaded-virtual)
endif()

if(APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

find_package ( libftdi1 NO_MODULE REQUIRED )
include ( ${LIBFTDI_USE_FILE} )

find_package(SoapySDR CONFIG)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

SOAPY_SDR_MODULE_UTIL(
    TARGET SDR14Support
    SOURCES
        SDR14_Registration.cpp
        SDR14_Settings.cpp
        SDR14_Streaming.cpp
    LIBRARIES
        ${LIBFTDI_LIBRARIES}
)
