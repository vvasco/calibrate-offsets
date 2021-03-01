################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB app ${CMAKE_SOURCE_DIR}/app/*.xml)
yarp_install(FILES ${app} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/${PROJECT_NAME})
