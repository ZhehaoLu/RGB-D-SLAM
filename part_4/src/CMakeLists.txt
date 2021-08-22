ADD_EXECUTABLE(main_test main.cpp)
TARGET_LINK_LIBRARIES( main_test
slambase
${OpenCV_LIBS} 
${PCL_LIBRARIES} )



FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

FIND_PACKAGE(OpenCV REQUIRED)
include_directories("/usr/include/Eigen")

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

list( APPEND CMAKE_MODULE_PATH /home/john/3rdparty/g2o/cmake_modules ) 
SET( G2O_ROOT /usr/local/include/g2o )

FIND_PACKAGE( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )

    ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
    TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

    ADD_LIBRARY( slambase slamBase.cpp )
    TARGET_LINK_LIBRARIES( slambase
        ${OpenCV_LIBS} 
        ${PCL_LIBRARIES} )
    
    ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
    TARGET_LINK_LIBRARIES( detectFeatures 
        slambase
        ${OpenCV_LIBS} 
        ${PCL_LIBRARIES} )

    ADD_EXECUTABLE( joinPointCloud joinPointCloud.cpp)
    TARGET_LINK_LIBRARIES( joinPointCloud
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

    ADD_EXECUTABLE( visualOdometry visualOdometry.cpp)
    TARGET_LINK_LIBRARIES( visualOdometry
        slambase
        ${OpenCV_LIBS} 
        ${PCL_LIBRARIES} )
        
    ADD_EXECUTABLE( slamEnd slamEnd.cpp )
        TARGET_LINK_LIBRARIES( slamEnd
            slambase
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${G2O_STUFF_LIBRARY}
            g2o_core g2o_types_slam3d g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})