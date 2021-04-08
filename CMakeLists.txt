# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it>

project(calib-red-ball)

find_package(OpenCV REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(ICUB REQUIRED)
find_package(YARP REQUIRED COMPONENTS os sig dev math cv)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

add_subdirectory(modules)
add_subdirectory(app)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

