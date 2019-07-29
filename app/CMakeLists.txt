cmake_minimum_required(VERSION 3.4.1)


include_directories(src/main/cpp/include)



#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__OPENCV_BUILD -DCERES_NO_LAPACK -DCERES_NO_SUITESPARSE -DCERES_NO_CXSPARSE -DCERES_STD_UNORDERED_MAP -DCERES_NO_THREADS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCERES_NO_LAPACK -DCERES_NO_SUITESPARSE -DCERES_NO_CXSPARSE -DCERES_STD_UNORDERED_MAP -DCERES_NO_THREADS")
#####**************Ceres library*****************#####
file(GLOB CERES_SRCS
        src/main/jni/ceres/*.cc
        src/main/jni/ceres/generated/*.cc
        src/main/jni/ceres/miniglog/glog/*.cc)
#add_library(ceres STATIC ${CERES_SRCS})
add_library(ceres STATIC ${CERES_SRCS})
SET_TARGET_PROPERTIES(ceres PROPERTIES COMPILE_FLAGS "-O2 -DNDEBUG")
target_include_directories(ceres PRIVATE src/main/cpp src/main/jni src/main/jni/ceres/miniglog)


set(OpenCV_DIR "/home/ale/AndroidStudioProjects/opencv3-android-sdk-with-contrib/OpenCV-android-sdk/sdk/native/jni")
#/home/ale/AndroidStudioProjects/OpenCV-android-sdk/sdk/native/jni")
find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_I)
#find_library(log-lib log)


#####**************Finding libraries*****************#####
find_library(zlib z)
#find_library(libdl dl)
find_library(log-lib log)
#####**************i3d library*****************#####
file(GLOB I3D_SRCS
        src/main/cpp/src/readData.cpp
        src/main/cpp/src/initInputData.cpp
        src/main/cpp/src/genFeatures.cpp
#        src/main/cpp/src/genRelativePoses.cpp
        src/main/cpp/src/genInitialGraph.cpp
        src/main/cpp/src/genGlobalByMst.cpp

        src/main/cpp/src/DProblem1.cpp

        src/main/cpp/src/WarperEglManager.cpp
        src/main/cpp/src/WarperGlesInitializer.cpp
        src/main/cpp/src/Warper4Android.cpp
        #src/Pers2PanoWarper.cpp
        #src/stitchAllPanos.cpp

        #src/genCompactTri.cpp
        )
add_library(i3d STATIC ${I3D_SRCS})
SET_TARGET_PROPERTIES(i3d PROPERTIES COMPILE_FLAGS "-O2 -DNDEBUG")
target_include_directories(i3d PRIVATE src/main/cpp/include src/main/jni src/main/jni/ceres/miniglog)
target_link_libraries(i3d ${OpenCV_LIBS} ceres ${zlib} ${log-lib} GLESv3 EGL)



#####**************target link *****************#####
add_library(native-lib SHARED src/main/cpp/native-lib.cpp)
target_include_directories(native-lib PRIVATE src/main/cpp src/main/cpp/include src/main/jni src/main/jni/ceres/miniglog)
target_link_libraries(native-lib jnigraphics i3d android)