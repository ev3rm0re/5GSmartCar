cmake_minimum_required(VERSION 3.5.0)
project(CarControl LANGUAGES CXX)

add_definitions(-Wno-psabi)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++17 -pthread")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_library(YAMLCPP_LIBRARIES NAMES libyaml-cpp.so HINTS /usr/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/include/yaml-cpp)


find_library(PIGPIO_LIBRARY NAMES libpigpio.so HINT /usr/local/lib)

link_libraries(${OpenCV_LIBS})
link_libraries(${PIGPIO_LIBRARY})
link_libraries(${YAMLCPP_LIBRARIES})


aux_source_directory(src DIR_SRCS)
add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PIGPIO_LIBRARY} ${YAMLCPP_LIBRARIES} atomic)