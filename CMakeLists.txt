
cmake_minimum_required(VERSION 2.8)
Project (pcl_multi_object_recognition CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release) 

set(INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(GRABBER_PATH ${INCLUDE_PATH}/grabber)
set(SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(IMPL_PATH ${INCLUDE_PATH}/impl)


#######find Openmp
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DHAVE_OMP)
endif()
#######


#######find libusb
if (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  # in cache already
  set(LIBUSB_FOUND TRUE)
else (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  find_path(LIBUSB_1_INCLUDE_DIR
    NAMES
  libusb.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  PATH_SUFFIXES
    libusb-1.0
  )

  find_library(LIBUSB_1_LIBRARY
    NAMES
      usb-1.0 usb
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(LIBUSB_1_INCLUDE_DIRS
    ${LIBUSB_1_INCLUDE_DIR}
  )
  set(LIBUSB_1_LIBRARIES
    ${LIBUSB_1_LIBRARY}
)

  if (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)
     set(LIBUSB_1_FOUND TRUE)
  endif (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)

  if (LIBUSB_1_FOUND)
    if (NOT libusb_1_FIND_QUIETLY)
      message(STATUS "Found libusb-1.0:")
    message(STATUS " - Includes: ${LIBUSB_1_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${LIBUSB_1_LIBRARIES}")
    endif (NOT libusb_1_FIND_QUIETLY)
  else (LIBUSB_1_FOUND)
    if (libusb_1_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libusb")
    endif (libusb_1_FIND_REQUIRED)
  endif (LIBUSB_1_FOUND)

  # show the LIBUSB_1_INCLUDE_DIRS and LIBUSB_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBUSB_1_INCLUDE_DIRS LIBUSB_1_LIBRARIES)

endif (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
#######

#######find openni2
  find_path(OPENNI2_INCLUDE_DIRS NAMES OpenNI.h HINTS /usr/local/include/ni2)
  find_library(OPENNI2_LIBRARY NAMES OpenNI2 libOpenNI2 HINTS /usr/local/lib/ni2 )
  include_directories(${OPENNI2_INCLUDE_DIRS})
#######

#######find pcl
  find_package(PCL 1.7 REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
#######

#######find freenect
  IF(NOT FREENECT_ROOT)
    IF(EXISTS "/usr/include/libfreenect")
      SET(FREENECT_ROOT "/usr")
    ELSEIF(EXISTS "/usr/local/include/libfreenect")
      SET(FREENECT_ROOT "/usr/local")
    ELSE()
      MESSAGE("FREENECT_ROOT not set. Continuing anyway..")
    ENDIF()
  ENDIF()

  # Include dir
  find_path(Freenect_INCLUDE_DIR
    NAMES libfreenect.h
    PATHS ${FREENECT_ROOT}/include/libfreenect 
  )

  include_directories(${Freenect_INCLUDE_DIR} )

  # Finally the library itself
  find_library(Freenect_LIBRARY
    NAMES freenect
    PATHS ${FREENECT_ROOT}/lib  /usr/local/lib64/
  )
#######

include_directories(${INCLUDE_PATH})
include_directories(${GRABBER_PATH})
include_directories(${IMPL_PATH})
include_directories(${LIBUSB_1_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS}/libusb-1.0)


  add_executable (pcl_multi_object_recognition pcl_multi_object_recognition.cpp  ${SOURCE_PATH}/Visualizer.cpp  ${SOURCE_PATH}/Cluster.cpp ${SOURCE_PATH}/Keypoints.cpp ${SOURCE_PATH}/Sampling.cpp ${SOURCE_PATH}/Icp.cpp  ${SOURCE_PATH}/Ppfe.cpp ${SOURCE_PATH}/OpenniStreamer.cpp ${SOURCE_PATH}/Error.cpp ${SOURCE_PATH}/find_function.cpp ${SOURCE_PATH}/define.cpp ${SOURCE_PATH}/help.cpp ${SOURCE_PATH}/utils.cpp ${SOURCE_PATH}/KeyDes.cpp ${GRABBER_PATH}/registration.cpp)

target_link_libraries( pcl_multi_object_recognition  ${OPENNI2_LIBRARY} ${PCL_LIBRARIES} ${Freenect_LIBRARY} )