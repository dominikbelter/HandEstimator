# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Estimator of Hand Configuration
PROJECT(HandEstimator)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/dependencies")

# core libraries
option(BUILD_HANDEST "Build HandEst libraries" YES)
option(BUILD_HANDEST_MATH "Build HandEst Math library" YES)
mark_as_advanced(BUILD_HANDEST_MATH)
option(BUILD_HANDEST_GRABBER "Build HandEst Kinect Grabber library" YES)
mark_as_advanced(BUILD_HANDEST_GRABBER)
option(BUILD_HANDEST_FILTER_PCL "Build HandEst Filter library" YES)
mark_as_advanced(BUILD_HANDEST_FILTER_PCL)
option(BUILD_HANDEST_KINEMATICS "Build HandEst Kinematics library" YES)
mark_as_advanced(BUILD_HANDEST_KINEMATICS)
option(BUILD_HANDEST_OPTIMIZATION_FUNCTION "Build HandEst Optimization Function library" YES)
mark_as_advanced(BUILD_HANDEST_OPTIMIZATION_FUNCTION)
option(BUILD_HANDEST_GL_VISUALIZER "Build Handest GL Visualizer library" YES)
mark_as_advanced(BUILD_HANDEST_GL_VISUALIZER)
option(BUILD_HANDEST_OPTIMIZATION_PSO "Build HandEst PSO Optimization library" YES)
mark_as_advanced(BUILD_HANDEST_OPTIMIZATION_PSO)

# demos
option(BUILD_HANDEST_DEMO "Build Handest demonstration programs and libraries" YES)
option(BUILD_HANDEST_DEMO_BASIC "Build Handest test" YES)
mark_as_advanced(BUILD_HANDEST_DEMO_BASIC)
option(BUILD_HANDEST_OPTIMIZATION_TEST "Build Handest Optimization Test" YES)
mark_as_advanced(BUILD_HANDEST_OPTIMIZATION_TEST)
option(BUILD_HANDEST_FK_DEMO "Build Handest Forward Kinematic Demo" YES)
mark_as_advanced(BUILD_HANDEST_FK_DEMO)
option(BUILD_HANDEST_FILTER_DEMO "Build Handest Filter Demo" YES)
mark_as_advanced(BUILD_HANDEST_FILTER_DEMO)
option(BUILD_HANDEST_Grabber_DEMO "Build Handest Grabber Demo" YES)
mark_as_advanced(BUILD_HANDEST_Grabber_demo)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

###############################################################################
#
# HandEst Math library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_MATH)
	file(GLOB MATH_SOURCES
		"${CMAKE_SOURCE_DIR}/src/Core/Math/*.cpp"
	)
	file(GLOB MATH_HEADERS
		"${CMAKE_SOURCE_DIR}/include/Core/Math/*.h"
	)

	ADD_LIBRARY(Math STATIC ${MATH_SOURCES} ${MATH_HEADERS})
	#TARGET_LINK_LIBRARIES(Math)
	INSTALL(TARGETS Math RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${MATH_HEADERS} DESTINATION include/handest/Core/Math/)
endif(BUILD_HANDEST AND BUILD_HANDEST_MATH)

###############################################################################
#
# HandEst Grabber library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_GRABBER)
	# Grabber interface
	SET(GRABBER_HEADERS ./include/Grabber/grabber.h)

	# Kinect Grabber
	SET(GRABBERKINECT_SOURCES ./src/Grabber/kinect_grabber.cpp)
	SET(GRABBERKINECT_HEADERS ./include/Grabber/kinect_grabber.h)
        ADD_LIBRARY(Grabber STATIC ${GRABBERKINECT_SOURCES} ${GRABBER_HEADERS} ${GRABBERKINECT_HEADERS})
	#TARGET_LINK_LIBRARIES(GrabberKinect)
        target_link_libraries (Grabber ${PCL_LIBRARIES})
        INSTALL(TARGETS Grabber RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/handest/Grabber/)
endif(BUILD_HANDEST AND BUILD_HANDEST_GRABBER)

###############################################################################
#
# HandEst Filter library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_FILTER_PCL)

	SET(FILTER_HEADERS ./include/Filter/filter.h)

	# Filter PCL
	SET(FILTERPCL_SOURCES ./src/Filter/filterPCL.cpp)
	SET(FILTERPCL_HEADERS ./include/Filter/filterPCL.h)
	ADD_LIBRARY(Filter STATIC ${FILTERPCL_SOURCES} ${FILTER_HEADERS} ${FILTERPCL_HEADERS})
	#TARGET_LINK_LIBRARIES(FilterPCL)
	INSTALL(TARGETS Filter RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${FILTER_HEADERS} DESTINATION include/handest/Filter/)
endif(BUILD_HANDEST AND BUILD_HANDEST_FILTER_PCL)

###############################################################################
#
# HandEst Optimization Function library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_OPTIMIZATION_FUNCTION)
	# OptimizationFunction interface
	SET(OPTIMIZATIONFUNCTION_HEADERS ./include/OptimizationFunction/optimizationFunction.h)

	# OptimizationFunctionGauss
	SET(OPTIMIZATIONFUNCTIONGAUSS_SOURCES ./src/OptimizationFunction/optimizationFunctionGauss.cpp)
	SET(OPTIMIZATIONFUNCTIONGAUSS_HEADERS ./include/OptimizationFunction/optimizationFunctionGauss.h)
	SET(OPTIMIZATIONFUNCTIONPF_SOURCES ./src/OptimizationFunction/optimizationFunctionPF.cpp)
	SET(OPTIMIZATIONFUNCTIONPF_HEADERS ./include/OptimizationFunction/optimizationFunctionPF.h)
	ADD_LIBRARY(OptimizationFunction STATIC ${OPTIMIZATIONFUNCTIONGAUSS_SOURCES} ${OPTIMIZATIONFUNCTION_HEADERS} ${OPTIMIZATIONFUNCTIONGAUSS_HEADERS} ${OPTIMIZATIONFUNCTIONPF_HEADERS} ${OPTIMIZATIONFUNCTIONPF_SOURCES})
	#TARGET_LINK_LIBRARIES(OptimizationFunction)
	INSTALL(TARGETS OptimizationFunction RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
endif (BUILD_HANDEST AND BUILD_HANDEST_OPTIMIZATION_FUNCTION)

###############################################################################
#
# HandEst Visualizer library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_GL_VISUALIZER)

	SET(VISUALIZER_HEADERS ./include/Visualizer/visualizer.h)

	# OpenGL Visualizer
	SET(VISUALIZERGL_SOURCES ./src/Visualizer/visualizerGL.cpp)
	SET(VISUALIZERGL_HEADERS ./include/Visualizer/visualizerGL.h)
        # PCL Visualizer
        SET(VISUALIZERPCL_SOURCES ./src/Visualizer/visualizerPCL.cpp)
        SET(VISUALIZERPCL_HEADERS ./include/Visualizer/visualizerPCL.h)
        ADD_LIBRARY(VisualizerGL STATIC ${VISUALIZERGL_SOURCES} ${VISUALIZERPCL_SOURCES} ${VISUALIZER_HEADERS} ${VISUALIZERGL_HEADERS} ${VISUALIZERPCL_HEADERS})
	#TARGET_LINK_LIBRARIES(VisualizerGL)
	INSTALL(TARGETS VisualizerGL RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
	INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/handest/Visualizer/)

endif (BUILD_HANDEST AND BUILD_HANDEST_GL_VISUALIZER)

###############################################################################
#
# HandEst Kinematic library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_KINEMATICS)

	SET(KINEMATIC_HEADERS ./include/Kinematic/forward_kinematics.h)

	# Forward Kinematics
	#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
	#find_package(Eigen3 REQUIRED)
	SET(FK_SOURCES ./src/Kinematic/kinematic_liego.cpp)
	SET(FK_HEADERS ./include/Kinematic/kinematic_liego.h )
	ADD_LIBRARY(ForwardKinematics STATIC ${KINEMATIC_HEADERS} ${FK_SOURCES} ${FK_HEADERS})
	INSTALL(TARGETS ForwardKinematics RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
endif (BUILD_HANDEST AND BUILD_HANDEST_KINEMATICS)

###############################################################################
#
# HandEst PSO Optimization library
#
###############################################################################

if(BUILD_HANDEST AND BUILD_HANDEST_OPTIMIZATION_PSO)
	SET(OPTIMIZATION_HEADERS ./include/Optimization/optimization.h)

	SET(OPTIMIZATION_SOURCES ./src/Optimization/optimizationPSO.cpp)
	SET(OPTIMIZATIONPSO_HEADERS ./include/Optimization/optimizationPSO.h)
	ADD_LIBRARY(Optimization STATIC ${OPTIMIZATION_SOURCES} ${OPTIMIZATION_HEADERS} ${OPTIMIZATIONPSO_HEADERS})
        TARGET_LINK_LIBRARIES(Math Grabber ForwardKinematics OptimizationFunction Filter VisualizerGL ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} GLU GL glut)
	INSTALL(TARGETS Optimization RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
endif (BUILD_HANDEST AND BUILD_HANDEST_OPTIMIZATION_PSO)


###############################################################################
#
# HandEst DEMO executables
#
###############################################################################

if(BUILD_HANDEST_DEMO AND BUILD_HANDEST_DEMO_BASIC)
	SET(DEMO_SOURCES ./main.cpp)
	ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(Demo Math Grabber ForwardKinematics Optimization OptimizationFunction Filter VisualizerGL GLU GL glut)
	INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
endif (BUILD_HANDEST_DEMO AND BUILD_HANDEST_DEMO_BASIC)


###############################################################################
#
# HandEst Optimization Test
#
###############################################################################

#
if(BUILD_HANDEST_DEMO AND BUILD_HANDEST_OPTIMIZATION_TEST)
	SET(OPTIMIZATION_TEST_SOURCES ./optimizationTest.cpp)
	ADD_EXECUTABLE(OptTest ${OPTIMIZATION_TEST_SOURCES})
        TARGET_LINK_LIBRARIES(OptTest OptimizationFunction)
	INSTALL(TARGETS OptTest RUNTIME DESTINATION bin)
endif (BUILD_HANDEST_DEMO AND BUILD_HANDEST_OPTIMIZATION_TEST)
#


###############################################################################
#
# HandEst Forward Kinematic Demo
#
###############################################################################

#
if(BUILD_HANDEST_DEMO AND BUILD_HANDEST_FK_DEMO)
	SET(FK_DEMO_SOURCES ./fkdemo.cpp)
	ADD_EXECUTABLE(FKdemo ${FK_DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(FKdemo Math Grabber ForwardKinematics VisualizerGL GLU GL glut)
	INSTALL(TARGETS FKdemo DESTINATION  ${CMAKE_CURRENT_BINARY_DIR}/build)
endif (BUILD_HANDEST_DEMO AND BUILD_HANDEST_FK_DEMO)
#

###############################################################################
#
# HandEst Filter Demo
#
###############################################################################

#
if(BUILD_HANDEST_DEMO AND BUILD_HANDEST_FILTER_DEMO)
	SET(FILTER_DEMO_SOURCES ./FilterDemo.cpp)
	ADD_EXECUTABLE(FilterDemo ${FILTER_DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(FilterDemo Math Grabber ForwardKinematics VisualizerGL Filter GLU GL glut)
	INSTALL(TARGETS FilterDemo DESTINATION  ${CMAKE_CURRENT_BINARY_DIR}/build)
endif (BUILD_HANDEST_DEMO AND BUILD_HANDEST_FILTER_DEMO)
#
#
###############################################################################
#
# HandEst Grabber Demo
#
###############################################################################

#
if(BUILD_HANDEST_DEMO AND BUILD_HANDEST_Grabber_DEMO)
	SET(FK_DEMO_SOURCES ./Grabber_demo.cpp)
	ADD_EXECUTABLE(Grabber_demo ${FK_DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(Grabber_demo Math Grabber ForwardKinematics VisualizerGL ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} GLU GL glut)
	INSTALL(TARGETS Grabber_demo DESTINATION  ${CMAKE_CURRENT_BINARY_DIR}/build)
endif (BUILD_HANDEST_DEMO AND BUILD_HANDEST_Grabber_DEMO)
#
