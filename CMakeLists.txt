
# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(v1)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(ICUB REQUIRED)
find_package( Threads )
find_package( Threads )

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${icub_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
#add_definitions(-D_MY_DEFINITION)
#add_definitions(-D_USE_MATH_DEFINES)


# building module
include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/src)
#controller
add_executable(${PROJECT_NAME}-controller 
               ${CMAKE_SOURCE_DIR}/src/controller.cpp)

target_link_libraries(${PROJECT_NAME}-controller
		      ${YARP_LIBRARIES}
		      iKin ctrlLib)
install(TARGETS ${PROJECT_NAME}-controller DESTINATION bin)

#controller2
add_executable(${PROJECT_NAME}-controller2 
               ${CMAKE_SOURCE_DIR}/src/controller2.cpp)

target_link_libraries(${PROJECT_NAME}-controller2
		      ${YARP_LIBRARIES}
		      iKin ctrlLib)
install(TARGETS ${PROJECT_NAME}-controller2 DESTINATION bin)

# sensors
add_executable(${PROJECT_NAME}-sensors 
	       ${CMAKE_SOURCE_DIR}/src/sensors.cpp)

target_link_libraries(${PROJECT_NAME}-sensors  
                      ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-sensors DESTINATION bin)

# learning: EKF
add_executable(${PROJECT_NAME}-learning
	       ${CMAKE_SOURCE_DIR}/src/learning.cpp)

target_link_libraries(${PROJECT_NAME}-learning  
                      ${YARP_LIBRARIES}
		      iKin ctrlLib)
install(TARGETS ${PROJECT_NAME}-learning DESTINATION bin)

# learning: NL-RLS
add_executable(${PROJECT_NAME}-learning_NLRLS
	       ${CMAKE_SOURCE_DIR}/src/learning_NLRLS.cpp)

target_link_libraries(${PROJECT_NAME}-learning_NLRLS  
                      ${YARP_LIBRARIES}
		      iKin ctrlLib)
install(TARGETS ${PROJECT_NAME}-learning_NLRLS DESTINATION bin)


# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()
