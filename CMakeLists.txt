cmake_minimum_required(VERSION 2.6)
project ( Calibration )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set ( CALIBRATION_SOURCES
    "src/precompiled.h"
    "src/calibrationwidget.h"
    "src/calibrationwidget.cpp"
    "src/image.h"
    "src/image.cpp"
    "src/imagewidget.h"
    "src/imagewidget.cpp"
    "src/mainwindow.h"
    "src/mainwindow.cpp"
    "src/iconswidget.h"
    "src/iconswidget.cpp"
    "src/defs.h"
    "src/application.h"
    "src/application.cpp"
    "src/camerawidget.h"
    "src/camerawidget.cpp"
    "src/previewwidget.h"
    "src/previewwidget.cpp"
    "src/parameterswidget.h"
    "src/parameterswidget.cpp"
    "src/taskwidget.h"
    "src/taskwidget.cpp"
    "src/supportwidgets.h"
    "src/supportwidgets.cpp"
    "src/templateprocessor.h"
    "src/templateprocessor.cpp"
    "src/reportwidget.h"
    "src/reportwidget.cpp"
    "src/vizwidget.h"
    "src/vizwidget.cpp"
    "src/disparitycontrolwidget.h"
    "src/disparitycontrolwidget.cpp"
    "src/calibrationapp.cpp"
)

set ( DISPARITY_PREVIEW_SOURCES
    "src/precompiled.h"
    "src/defs.h"
    "src/image.h"
    "src/image.cpp"
    "src/imagewidget.h"
    "src/imagewidget.cpp"
    "src/supportwidgets.h"
    "src/supportwidgets.cpp"
    "src/disparitycontrolwidget.h"
    "src/disparitycontrolwidget.cpp"
    "src/disparitypreviewwidget.h"
    "src/disparitypreviewwidget.cpp"
    "src/disparitypreviewapp.cpp"
)

set ( RESOURCES
    "resources/resources.qrc"
)

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK 8 REQUIRED)


add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

QT5_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

add_executable( ${PROJECT_NAME} ${CALIBRATION_SOURCES} ${RES_SOURCES} )
add_executable( DisparityPreview ${DISPARITY_PREVIEW_SOURCES} ${RES_SOURCES} )

target_link_libraries( ${PROJECT_NAME} ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} )
target_link_libraries( DisparityPreview ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} )









