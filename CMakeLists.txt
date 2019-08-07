cmake_minimum_required( VERSION 2.6 )
project ( Calibration )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set ( COMMON_SOURCES
    src/common/defs.h
    src/common/precompiled.h
    src/common/image.h
    src/common/image.cpp
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
    src/common/ipwidget.h
    src/common/ipwidget.cpp
    src/common/documentarea.h
    src/common/documentarea.cpp
    src/common/iconswidget.h
    src/common/iconswidget.cpp
    src/common/fileslistwidget.h
    src/common/fileslistwidget.cpp
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

set ( ORB_SOURCES
    src/ORB/Settings.h
    src/ORB/Settings.cpp
    src/ORB/System.h
    src/ORB/System.cc
    src/ORB/Tracking.h
    src/ORB/Tracking.cc
    src/ORB/LocalMapping.h
    src/ORB/LocalMapping.cc
    src/ORB/LoopClosing.h
    src/ORB/LoopClosing.cc
    src/ORB/ORBVocabulary.h
    src/ORB/ORBextractor.h
    src/ORB/ORBextractor.cc
    src/ORB/ORBmatcher.h
    src/ORB/ORBmatcher.cc
    src/ORB/FrameDrawer.h
    src/ORB/FrameDrawer.cc
    src/ORB/Converter.h
    src/ORB/Converter.cc
    src/ORB/MapPoint.h
    src/ORB/MapPoint.cc
    src/ORB/KeyFrame.h
    src/ORB/KeyFrame.cc
    src/ORB/Map.h
    src/ORB/Map.cc
    src/ORB/MapDrawer.h
    src/ORB/MapDrawer.cc
    src/ORB/Optimizer.h
    src/ORB/Optimizer.cc
    src/ORB/PnPsolver.h
    src/ORB/PnPsolver.cc
    src/ORB/Frame.h
    src/ORB/Frame.cc
    src/ORB/KeyFrameDatabase.h
    src/ORB/KeyFrameDatabase.cc
    src/ORB/Sim3Solver.h
    src/ORB/Sim3Solver.cc
    src/ORB/Initializer.h
    src/ORB/Initializer.cc
    src/ORB/Viewer.h
    src/ORB/Viewer.cc
)

set ( ANNOTATION_SOURCES
    src/rapidxml/xmlTree.cpp
    src/annotations/annotation.h
    src/annotations/annotation.cpp
)

set ( RESOURCES
    resources/resources.qrc
)

find_package( OpenMP REQUIRED )
find_package( Qt5Widgets REQUIRED )
find_package( OpenCV 4 REQUIRED )
find_package( Eigen3 3.1.0 REQUIRED )
find_package( VTK 8 REQUIRED )
find_package( PCL 1.9 REQUIRED )
find_package( Pangolin REQUIRED )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/vimba/include )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

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
    src/calibration/templateprocessor.h
    src/calibration/templateprocessor.cpp
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
    src/disparity/stereoprocessor.h
    src/disparity/stereoprocessor.cpp
    src/disparity/pclwidget.h
    src/disparity/pclwidget.cpp
    src/disparity/documentwidget.h
    src/disparity/documentwidget.cpp
    src/disparity/main.cpp
)

add_executable( imagesodometry ${COMMON_SOURCES} ${ANNOTATION_SOURCES} ${ORB_SOURCES} ${RES_SOURCES}
    src/imagesodometry/main.cpp
)

add_executable( cameraodometry ${COMMON_SOURCES} ${ANNOTATION_SOURCES} ${ORB_SOURCES} ${RES_SOURCES}
    src/cameraodometry/main.cpp
)

add_executable( writer ${COMMON_SOURCES} ${RES_SOURCES}
    src/writer/writer.h
    src/writer/writer.cpp
    src/writer/main.cpp
)

target_include_directories( imagesodometry PRIVATE ${Pangolin_INCLUDE_DIRS} )
target_include_directories( cameraodometry PRIVATE ${Pangolin_INCLUDE_DIRS} )

target_link_libraries( calibration ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
target_link_libraries( disparity ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
target_link_libraries( imagesodometry ${Qt5Widgets_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${PROJECT_SOURCE_DIR}/src/DBoW2/lib/libDBoW2.so ${PROJECT_SOURCE_DIR}/src/g2o/lib/libg2o.so )
target_link_libraries( cameraodometry ${Qt5Widgets_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${PROJECT_SOURCE_DIR}/src/DBoW2/lib/libDBoW2.so ${PROJECT_SOURCE_DIR}/src/g2o/lib/libg2o.so )
target_link_libraries( writer ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so )
