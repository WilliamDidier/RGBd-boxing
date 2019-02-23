cmake_minimum_required(VERSION 3.9)
project(TEST)

set (SRC Main.cpp)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall")
set (PARENTWD ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(CWD ${CMAKE_CURRENT_SOURCE_DIR})

function(AddSourceInGroup MainFolder cpp_folder GroupName)
    # Add cpp data
    # Message( STATUS ${cpp_folder} )
    file(GLOB_RECURSE CPP_SRCS RELATIVE ${MainFolder} "${cpp_folder}/*.cpp")
    # source_group(${GroupName} FILES ${CPP_SRCS})
    set(${GroupName} ${${GroupName}} ${CPP_SRCS} PARENT_SCOPE)
endfunction(AddSourceInGroup)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

add_definitions(-DKINECT_2 -DACTIVATE_KINECT_DRAWING -D_FILE_OFFSET_BITS=64)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUD_DIRS})


include_directories("${PARENTWD}/Omiscid/Json/")
include_directories("${PARENTWD}/Omiscid/System/")
include_directories("${PARENTWD}/Omiscid/Messaging/")
include_directories("${PARENTWD}/Map/")
include_directories(${PARENTWD})

# Add main files from the project
file(GLOB SRCS RELATIVE ${CWD} ${CWD})

# Add Omiscid files
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/System" "OMISCID_SYSTEM")
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/Json" "OMISCID_JSON")
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/Messaging" "OMISCID_MESSAGING")


# Add Drawing and DataManagement files
AddSourceInGroup(${CWD} "${PARENTWD}/Drawing" "DRAWING")
AddSourceInGroup(${CWD} "${PARENTWD}/DataManagement" "DATAMANAGEMENT")

# Add mandatory files for the Kinect folder
file(GLOB_RECURSE KINECT RELATIVE ${CWD}
        "${PARENTWD}/Kinect/KinectBasics.cpp"
        "${PARENTWD}/Kinect/KinectBody.cpp"
        "${PARENTWD}/Kinect/KinectDataAsMemoryBuffer.cpp"
        "${PARENTWD}/Kinect/KinectFace.cpp"
        "${PARENTWD}/Kinect/KinectImageConverter.cpp"
        )


# And for Geometric map
AddSourceInGroup(${CWD} "${PARENTWD}/Map" "MAP")

file(GLOB_RECURSE HDRS RELATIVE ${CWD}
        "${PARENTWD}/Omiscid/System/System/*.h"
        "${PARENTWD}/Omiscid/Messaging/Messaging/*.h"
        "${PARENTWD}/Omiscid/Json/Json/*.h"
        "${PARENTWD}/Drawing/*.h"
        "${PARENTWD}/Kinect/*.h"
        "${PARENTWD}/DataManagement/*.h"
        "${PARENTWD}/Map/*.hpp"
        )

#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(main Main.cpp ${SRCS} ${HDRS} ${OMISCID_SYSTEM} ${OMISCID_JSON} ${OMISCID_MESSAGING} ${DRAWING}  ${DATAMANAGEMENT} ${KINECT} ${MAP} planeManagement.cpp planeManagement.h bodyProcessing.cpp bodyProcessing.h sceneClustering.h sceneClustering.cpp)
target_link_libraries(main ${OpenCV_LIBS})
#target_link_libraries(main ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${PCL_VISUALISATION_LIBRARIES})
target_link_libraries(main ${PCL_LIBRARIES})
