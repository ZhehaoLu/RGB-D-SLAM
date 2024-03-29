ADD_EXECUTABLE(main_test main.cpp)
TARGET_LINK_LIBRARIES( main_test
slambase
${OpenCV_LIBS} 
${PCL_LIBRARIES} )

FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

FIND_PACKAGE(OpenCV REQUIRED)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

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