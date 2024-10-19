cmake_minimum_required(VERSION 3.5.0)
project(CarControl LANGUAGES CXX)

add_definitions(-Wno-psabi)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

set(SFML_DIR "/usr/lib/arm-linux-gnueabihf/cmake/SFML")

find_package(SFML 2.5.1 COMPONENTS audio REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${SFML_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

find_library(PIGPIO_LIBRARY NAMES libpigpio.so HINT /usr/local/lib)

link_libraries(${OpenCV_LIBS})
link_libraries(${PIGPIO_LIBRARY})


aux_source_directory(src DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PIGPIO_LIBRARY} sfml-audio)