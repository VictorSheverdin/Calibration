cmake_minimum_required(VERSION 2.6)
project ( Calibration )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

set ( SOURCES
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
    "src/main.cpp"
)

set ( RESOURCES
    "resources/resources.qrc"
)

find_package (Qt5Widgets)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

# Next, using precompiler, compiler and linker

# using Qt meta-system (precompiler)
QT5_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )
QT5_WRAP_UI( UI_HEADERS ${UIS} )
QT5_WRAP_CPP( MOC_SRCS ${MOC_HEADERS} )

# compile
add_executable( ${PROJECT_NAME} ${SOURCES} ${MOC_SRCS} ${RES_SOURCES} ${UI_HEADERS} )

# build it (link libraries)
target_link_libraries( ${PROJECT_NAME} ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} )










