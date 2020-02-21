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
    src/common/rectificationprocessor.h
    src/common/rectificationprocessor.cpp
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

set ( VISO_SOURCES
    src/viso2/timer.h
    src/viso2/filter.h
    src/viso2/filter.cpp
    src/viso2/matrix.h
    src/viso2/matrix.cpp
    src/viso2/matcher.h
    src/viso2/matcher.cpp
    src/viso2/triangle.h
    src/viso2/triangle.cpp
    src/viso2/reconstruction.h
    src/viso2/reconstruction.cpp
    src/viso2/viso.h
    src/viso2/viso.cpp
    src/viso2/viso_mono.h
    src/viso2/viso_mono.cpp
    src/viso2/viso_stereo.h
    src/viso2/viso_stereo.cpp
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
find_package( RTABMap REQUIRED )
find_package( SuiteSparse REQUIRED )
find_package( G2O REQUIRED )

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/vimba/include )
link_directories( ${PCL_LIBRARY_DIRS} )
link_libraries( ${OpenCV_LIBS} ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so )
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
    src/disparity/elasprocessor.h
    src/disparity/elasprocessor.cpp
    src/disparity/stereoresultprocessor.h
    src/disparity/stereoresultprocessor.cpp
    src/disparity/documentwidget.h
    src/disparity/documentwidget.cpp
    src/disparity/main.cpp
)

add_executable( imagesorbodometry ${COMMON_SOURCES} ${ANNOTATION_SOURCES} ${ORB_SOURCES} ${RES_SOURCES}
    src/imagesodometry/main.cpp
)

add_executable( cameraorbodometry ${COMMON_SOURCES} ${ANNOTATION_SOURCES} ${ORB_SOURCES} ${RES_SOURCES}
    src/cameraodometry/main.cpp
)

add_executable( visoodometry ${COMMON_SOURCES} ${VISO_SOURCES}
    src/visoodometry/main.cpp
)

#add_executable( rtabmapslam ${COMMON_SOURCES} ${RES_SOURCES}
#    src/rtabmapslam/main.cpp
#)

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
    src/slam/frame.h
    src/slam/framepoint.h
    src/slam/optimizer.h
    src/slam/world.h
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
    src/slam/frame.cpp
    src/slam/framepoint.cpp
    src/slam/optimizer.cpp
    src/slam/world.cpp
    src/slam/main.cpp
)

add_executable( exposure ${COMMON_SOURCES} ${RES_SOURCES}
    src/exposure/application.h
    src/exposure/mainwindow.h
    src/exposure/application.cpp
    src/exposure/mainwindow.cpp
    src/exposure/main.cpp
)

add_executable( lidar ${COMMON_SOURCES} ${RES_SOURCES}
    src/lidar/application.h
    src/lidar/mainwindow.h
    src/lidar/application.cpp
    src/lidar/mainwindow.cpp
    src/lidar/main.cpp
)

target_include_directories( imagesorbodometry PRIVATE ${Pangolin_INCLUDE_DIRS} )
target_include_directories( cameraorbodometry PRIVATE ${Pangolin_INCLUDE_DIRS} )
#target_include_directories( rtabmapslam PRIVATE ${RTABMap_INCLUDE_DIRS} )
target_include_directories( slam PRIVATE ${G2O_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )

target_link_libraries( calibration ${Qt5Widgets_LIBRARIES} ${YAML_CPP_LIBRARIES} )
target_link_libraries( disparity ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} ${YAML_CPP_LIBRARIES} )
target_link_libraries( imagesorbodometry ${Qt5Widgets_LIBRARIES} ${Pangolin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/DBoW2/lib/libDBoW2.so ${PROJECT_SOURCE_DIR}/src/g2o/lib/libg2o.so )
target_link_libraries( cameraorbodometry ${Qt5Widgets_LIBRARIES} ${Pangolin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/DBoW2/lib/libDBoW2.so ${PROJECT_SOURCE_DIR}/src/g2o/lib/libg2o.so )
target_link_libraries( visoodometry ${Qt5Widgets_LIBRARIES} png )
#target_link_libraries( rtabmapslam ${RTABMap_LIBRARIES} )

target_link_libraries( slam PRIVATE
                            ${Qt5Widgets_LIBRARIES}
                            ${VTK_LIBRARIES}
                            ${G2O_CORE_LIBRARY}
                            ${G2O_STUFF_LIBRARY}
                            ${G2O_CLI_LIBRARY}
                            ${G2O_TYPES_SBA}
                            ${G2O_TYPES_SLAM3D}
                            ${G2O_SOLVER_EIGEN}
                            ${CHOLMOD_LIBRARIES}
)

target_link_libraries( exposure PRIVATE ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} )

target_link_libraries( lidar PRIVATE ${Qt5Widgets_LIBRARIES} ${VTK_LIBRARIES} )
