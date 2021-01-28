cmake_minimum_required( VERSION 3.10 )
project( Calibration VERSION 0.0.1 )

set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTOUIC ON )

set ( COMMON_SOURCES
    src/common/defs.h
    src/common/precompiled.h
    src/common/image.h
    src/common/image.cpp
    src/common/colorpoint.h
    src/common/colorpoint.cpp
    src/common/imagewidget.h
    src/common/imagewidget.cpp
    src/common/vimbacamera.h
    src/common/vimbacamera.cpp
    src/common/limitedqueue.h
    src/common/limitedqueue.inl
    src/common/supportwidgets.h
    src/common/supportwidgets.cpp
    src/common/supportwidgets.inl
    src/common/functions.h
    src/common/functions.cpp
    src/common/calibrationdatabase.h
    src/common/calibrationdatabase.cpp
    src/common/templateprocessor.h
    src/common/templateprocessor.cpp
    src/common/rectificationprocessor.h
    src/common/rectificationprocessor.cpp
    src/common/supportclasses.h
    src/common/supportclasses.cpp
    src/common/supportclasses.inl
    src/common/stereoprocessor.h
    src/common/stereoprocessor.cpp
    src/common/plane.h
    src/common/plane.cpp
    src/common/projectionmatrix.h
    src/common/projectionmatrix.cpp
    src/common/markerprocessor.h
    src/common/markerprocessor.cpp
    src/common/featureprocessor.h
    src/common/featureprocessor.cpp
    src/common/ipwidget.h
    src/common/ipwidget.cpp
    src/common/documentarea.h
    src/common/documentarea.cpp
    src/common/iconswidget.h
    src/common/iconswidget.cpp
    src/common/fileslistwidget.h
    src/common/fileslistwidget.cpp
    src/common/pclwidget.h
    src/common/pclwidget.cpp
    src/common/matrix.h
    src/common/matrix.inl
    src/common/rungekutta.h
    src/common/rungekutta.inl
    src/common/xsens.h
    src/common/xsens.cpp
    src/common/tictoc.h
    src/common/tictoc.cpp
    src/superglue/extract_common.cpp
    src/superglue/super_point_detector.cpp
    src/superglue/super_glue_matcher.cpp
    src/superglue/keypoint_selector.cpp
)

set ( LIBELAS_SOURCES
    src/libelas/descriptor.h
    src/libelas/elas.h
    src/libelas/image.h
    src/libelas/filter.h
    src/libelas/matrix.h
    src/libelas/triangle.h
    src/libelas/StereoEfficientLargeScale.h
    src/libelas/descriptor.cpp
    src/libelas/elas.cpp
    src/libelas/filter.cpp
    src/libelas/matrix.cpp
    src/libelas/triangle.cpp
    src/libelas/StereoEfficientLargeScale.cpp
)

set ( ANNOTATION_SOURCES
    src/rapidxml/xmlTree.cpp
    src/annotations/annotation.h
    src/annotations/annotation.cpp
)

set ( RESOURCES
    resources/resources.qrc
)

# Find TensorRT
set ( TensorRT_ROOT /usr/local/TensorRT-7.2.2.3 )
set ( TensorRT_INCLUDE_DIR ${TensorRT_ROOT}/include )
set ( TensorRT_LIBRARY_DIR ${TensorRT_ROOT}/lib )
message ( "TensorRT at: " ${TensorRT_ROOT} )

# Find CUDNN for Torch
set ( CUDNN_ROOT /usr )
set ( CUDNN_INCLUDE_PATH ${CUDNN_ROOT}/include )
set ( CUDNN_LIBRARY_PATH ${CUDNN_ROOT}/lib/x86_64-linux-gnu/libcudnn.so )
message ( "CUDNN at " ${CUDNN_ROOT} )

find_package ( Torch REQUIRED )
find_package ( CUDA 11.1 REQUIRED )
find_package( OpenMP REQUIRED )
find_package( Qt5Widgets REQUIRED )
find_package( Qt5Charts REQUIRED )
find_package( OpenCV 4.4 REQUIRED )
find_package( Eigen3 3.1 REQUIRED )
find_package( PCL 1.11 REQUIRED )
find_package( SuiteSparse REQUIRED )
find_package( G2O REQUIRED )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS ON )

if ( CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    add_compile_options( -Wall -O3 -march=native )
endif ()

add_definitions( ${QT_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )

set( XSENS_DIR "/usr/local/xsens" )

include_directories(
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Charts_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src/vimba/include
    ${XSENS_DIR}/include )

link_directories( ${PCL_LIBRARY_DIRS} ${XSENS_DIR}/lib )

link_libraries(
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Charts_LIBRARIES}
    ${TensorRT_LIBRARY_DIR}/libnvinfer.so
    ${TensorRT_LIBRARY_DIR}/libnvparsers.so
    ${TensorRT_LIBRARY_DIR}/libnvonnxparser.so
    ${TensorRT_LIBRARY_DIR}/libnvinfer_plugin.so
    ${TORCH_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBS}
    OpenMP::OpenMP_CXX
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so
    xscontroller
    xscommon
    xstypes
)

QT5_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

add_executable( calibration ${COMMON_SOURCES} ${RES_SOURCES}
    src/calibration/application.h
    src/calibration/application.cpp
    src/calibration/mainwindow.h
    src/calibration/mainwindow.cpp
    src/calibration/calibrationdata.h
    src/calibration/calibrationdata.cpp
    src/calibration/calibrationwidget.h
    src/calibration/calibrationwidget.cpp
    src/calibration/calibrationchoicedialog.h
    src/calibration/calibrationchoicedialog.cpp
    src/calibration/calibrationiconswidget.h
    src/calibration/calibrationiconswidget.cpp
    src/calibration/camerawidget.h
    src/calibration/camerawidget.cpp
    src/calibration/previewwidget.h
    src/calibration/previewwidget.cpp
    src/calibration/taskwidget.h
    src/calibration/taskwidget.cpp
    src/calibration/threads.h
    src/calibration/threads.cpp
    src/calibration/reportwidget.h
    src/calibration/reportwidget.cpp
    src/calibration/parameterswidget.h
    src/calibration/parameterswidget.cpp
    src/calibration/documentwidget.h
    src/calibration/documentwidget.cpp
    src/calibration/main.cpp
)

add_executable( disparity ${COMMON_SOURCES} ${LIBELAS_SOURCES} ${RES_SOURCES}
    src/disparity/application.h
    src/disparity/application.cpp
    src/disparity/mainwindow.h
    src/disparity/mainwindow.cpp
    src/disparity/disparitychoicedialog.h
    src/disparity/disparitychoicedialog.cpp
    src/disparity/disparitycontrolwidget.h
    src/disparity/disparitycontrolwidget.cpp
    src/disparity/disparitypreviewwidget.h
    src/disparity/disparitypreviewwidget.cpp
    src/disparity/disparityiconswidget.h
    src/disparity/disparityiconswidget.cpp
    src/disparity/stereoresultprocessor.h
    src/disparity/stereoresultprocessor.cpp
    src/disparity/elasprocessor.h
    src/disparity/elasprocessor.cpp
    src/disparity/processorthread.h
    src/disparity/processorthread.cpp
    src/disparity/documentwidget.h
    src/disparity/documentwidget.cpp
    src/disparity/main.cpp
)

add_executable( slam ${COMMON_SOURCES} ${RES_SOURCES}
    src/slam/application.h
    src/slam/choicedialog.h
    src/slam/slamwidget.h
    src/slam/slamdocument.h
    src/slam/slamdialog.h
    src/slam/slamgeometry.h
    src/slam/slamthread.h
    src/slam/mainwindow.h
    src/slam/map.h
    src/slam/mappoint.h
    src/slam/imudata.h
    src/slam/frame.h
    src/slam/framepoint.h
    src/slam/optimizer.h
    src/slam/world.h
    src/slam/log.h
    src/slam/settings.h
    src/slam/alias.h
    src/slam/tracker.h
    src/slam/application.cpp
    src/slam/choicedialog.cpp
    src/slam/slamwidget.cpp
    src/slam/slamdocument.cpp
    src/slam/slamdialog.cpp
    src/slam/slamgeometry.cpp
    src/slam/slamthread.cpp
    src/slam/mainwindow.cpp
    src/slam/map.cpp
    src/slam/mappoint.cpp
    src/slam/imudata.cpp
    src/slam/frame.cpp
    src/slam/framepoint.cpp
    src/slam/optimizer.cpp
    src/slam/world.cpp
    src/slam/log.cpp
    src/slam/settings.cpp
    src/slam/tracker.cpp
    src/slam/main.cpp
)

add_executable( slam2 ${COMMON_SOURCES} ${RES_SOURCES}
    src/slam2/alias.h
    src/slam2/application.h
    src/slam2/application.cpp
    src/slam2/mainwindow.h
    src/slam2/mainwindow.cpp
    src/slam2/parameters.h
    src/slam2/parameters.cpp
    src/slam2/system.h
    src/slam2/system.cpp
    src/slam2/map.h
    src/slam2/map.cpp
    src/slam2/frame.h
    src/slam2/frame.cpp
    src/slam2/framepoint.h
    src/slam2/framepoint.cpp
    src/slam2/mappoint.h
    src/slam2/mappoint.cpp
    src/slam2/track.h
    src/slam2/track.cpp
    src/slam2/tracker.h
    src/slam2/tracker.cpp
    src/slam2/thread.h
    src/slam2/thread.cpp
    src/slam2/slamwidget.h
    src/slam2/slamwidget.cpp
    src/slam2/slamdocument.h
    src/slam2/slamdocument.cpp
    src/slam2/slamdialog.h
    src/slam2/slamdialog.cpp
    src/slam2/main.cpp
)

add_executable( exposure ${COMMON_SOURCES} ${RES_SOURCES}
    src/exposure/application.h
    src/exposure/mainwindow.h
    src/exposure/application.cpp
    src/exposure/mainwindow.cpp
    src/exposure/main.cpp
)

add_executable( features ${COMMON_SOURCES} ${RES_SOURCES}
    src/features/application.h
    src/features/application.cpp
    src/features/mainwindow.h
    src/features/mainwindow.cpp
    src/features/matcher.h
    src/features/matcher.cpp
    src/features/processorthread.h
    src/features/processorthread.cpp
    src/features/featuresiconswidget.h
    src/features/featuresiconswidget.cpp
    src/features/controlwidget.h
    src/features/controlwidget.cpp
    src/features/viewwidget.h
    src/features/viewwidget.cpp
    src/features/documentwidget.h
    src/features/documentwidget.cpp
    src/features/main.cpp
)

add_executable( lidar ${COMMON_SOURCES} ${RES_SOURCES}
    src/lidar/application.h
    src/lidar/application.cpp
    src/lidar/mainwindow.h
    src/lidar/mainwindow.cpp
    src/lidar/main.cpp
)

target_include_directories( slam PRIVATE ${G2O_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )
target_include_directories( slam2 PRIVATE ${G2O_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )

target_link_libraries( slam PRIVATE
                            ${G2O_CORE_LIBRARY}
                            ${G2O_STUFF_LIBRARY}
                            ${G2O_CLI_LIBRARY}
                            ${G2O_TYPES_SBA}
                            ${G2O_TYPES_SLAM3D}
                            ${G2O_SOLVER_EIGEN}
                            ${CHOLMOD_LIBRARIES}
)

target_link_libraries( slam2 PRIVATE
                            ${G2O_CORE_LIBRARY}
                            ${G2O_STUFF_LIBRARY}
                            ${G2O_CLI_LIBRARY}
                            ${G2O_TYPES_SBA}
                            ${G2O_TYPES_SLAM3D}
                            ${G2O_SOLVER_EIGEN}
                            ${CHOLMOD_LIBRARIES}
)

