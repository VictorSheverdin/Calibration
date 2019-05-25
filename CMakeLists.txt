cmake_minimum_required( VERSION 2.6 )
project ( Calibration )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set ( CALIBRATION_SOURCES
    "src/common/defs.h"
    "src/common/precompiled.h"
    "src/common/image.h"
    "src/common/image.cpp"
    "src/common/imagewidget.h"
    "src/common/imagewidget.cpp"
    "src/common/vimbacamera.h"
    "src/common/vimbacamera.cpp"
    "src/common/limitedqueue.h"
    "src/common/limitedqueue.inl"
    "src/common/supportwidgets.h"
    "src/common/supportwidgets.cpp"
    "src/common/supportwidgets.inl"
    "src/common/functions.h"
    "src/common/functions.cpp"
    "src/common/calibrationdata.h"
    "src/common/calibrationdata.cpp"
    "src/common/ipwidget.h"
    "src/common/ipwidget.cpp"
    "src/common/documentarea.h"
    "src/common/documentarea.cpp"
    "src/common/iconswidget.h"
    "src/common/iconswidget.cpp"
    "src/common/fileslistwidget.h"
    "src/common/fileslistwidget.cpp"
    "src/calibration/application.h"
    "src/calibration/application.cpp"
    "src/calibration/mainwindow.h"
    "src/calibration/mainwindow.cpp"
    "src/calibration/calibrationwidget.h"
    "src/calibration/calibrationwidget.cpp"
    "src/calibration/calibrationiconswidget.h"
    "src/calibration/calibrationiconswidget.cpp"
    "src/calibration/camerawidget.h"
    "src/calibration/camerawidget.cpp"
    "src/calibration/previewwidget.h"
    "src/calibration/previewwidget.cpp"
    "src/calibration/taskwidget.h"
    "src/calibration/taskwidget.cpp"
    "src/calibration/templateprocessor.h"
    "src/calibration/templateprocessor.cpp"
    "src/calibration/reportwidget.h"
    "src/calibration/reportwidget.cpp"
    "src/calibration/parameterswidget.h"
    "src/calibration/parameterswidget.cpp"
    "src/calibration/documentwidget.h"
    "src/calibration/documentwidget.cpp"
    "src/calibration/main.cpp"
)

set ( DISPARITY_SOURCES
    "src/common/defs.h"
    "src/common/precompiled.h"
    "src/common/image.h"
    "src/common/image.cpp"
    "src/common/imagewidget.h"
    "src/common/imagewidget.cpp"
    "src/common/calibrationdata.h"
    "src/common/calibrationdata.cpp"
    "src/common/vimbacamera.h"
    "src/common/vimbacamera.cpp"
    "src/common/limitedqueue.h"
    "src/common/limitedqueue.inl"
    "src/common/functions.h"
    "src/common/functions.cpp"
    "src/common/supportwidgets.h"
    "src/common/supportwidgets.cpp"
    "src/common/supportwidgets.inl"
    "src/common/ipwidget.h"
    "src/common/ipwidget.cpp"
    "src/common/documentarea.h"
    "src/common/documentarea.cpp"
    "src/common/videopreviewwidget.h"
    "src/common/videopreviewwidget.cpp"
    "src/common/iconswidget.h"
    "src/common/iconswidget.cpp"
    "src/common/fileslistwidget.h"
    "src/common/fileslistwidget.cpp"
    "src/disparity/application.h"
    "src/disparity/application.cpp"
    "src/disparity/mainwindow.h"
    "src/disparity/mainwindow.cpp"
    "src/disparity/disparitycontrolwidget.h"
    "src/disparity/disparitycontrolwidget.cpp"
    "src/disparity/disparitypreviewwidget.h"
    "src/disparity/disparitypreviewwidget.cpp"
    "src/disparity/disparityiconswidget.h"
    "src/disparity/disparityiconswidget.cpp"
    "src/disparity/stereoprocessor.h"
    "src/disparity/stereoprocessor.cpp"
    "src/disparity/pclwidget.h"
    "src/disparity/pclwidget.cpp"
    "src/disparity/documentwidget.h"
    "src/disparity/documentwidget.cpp"
    "src/disparity/main.cpp"
)

set ( COMPARE_SOURCES
    "src/rapidxml/xmlTree.cpp"
    "src/annotations/annotation.h"
    "src/annotations/annotation.cpp"
    "src/compare/main.cpp"
)

set ( ODOMETRY_SOURCES
    "src/common/documentarea.h"
    "src/common/documentarea.cpp"
    "src/common/supportwidgets.h"
    "src/common/supportwidgets.cpp"
    "src/common/supportwidgets.inl"
    "src/common/fileslistwidget.h"
    "src/common/fileslistwidget.cpp"
    "src/odometry/System.cc"
    "src/odometry/Tracking.cc"
    "src/odometry/LocalMapping.cc"
    "src/odometry/LoopClosing.cc"
    "src/odometry/ORBextractor.cc"
    "src/odometry/ORBmatcher.cc"
    "src/odometry/FrameDrawer.cc"
    "src/odometry/Converter.cc"
    "src/odometry/MapPoint.cc"
    "src/odometry/KeyFrame.cc"
    "src/odometry/Map.cc"
    "src/odometry/MapDrawer.cc"
    "src/odometry/Optimizer.cc"
    "src/odometry/PnPsolver.cc"
    "src/odometry/Frame.cc"
    "src/odometry/KeyFrameDatabase.cc"
    "src/odometry/Sim3Solver.cc"
    "src/odometry/Initializer.cc"
    "src/odometry/Viewer.cc"
    "src/annotations/annotation.h"
    "src/annotations/annotation.cpp"
    "src/rapidxml/xmlTree.cpp"
    "src/odometry/main.cpp"
)

set ( RESOURCES
    "resources/resources.qrc"
)

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(VTK 7.1 REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(Pangolin REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/vimba/include )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

QT5_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

add_executable( calibration ${CALIBRATION_SOURCES} ${RES_SOURCES} )
add_executable( disparity ${DISPARITY_SOURCES} ${RES_SOURCES} )
add_executable( compare ${COMPARE_SOURCES} )
add_executable( odometry ${ODOMETRY_SOURCES} ${RES_SOURCES} )

target_include_directories( odometry PRIVATE ${Pangolin_INCLUDE_DIRS} )

target_link_libraries( calibration ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
target_link_libraries( disparity ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
target_link_libraries( compare ${Qt5Widgets_LIBRARIES} )
target_link_libraries( odometry ${Qt5Widgets_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${PROJECT_SOURCE_DIR}/src/DBoW2/lib/libDBoW2.so ${PROJECT_SOURCE_DIR}/src/g2o/lib/libg2o.so )
