cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)

project(ground_removal_experiment)

find_package(PCL 1.2 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS}, ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND COMMON_SOURCE src/point_categories.cpp)

list(APPEND POINT_CATEGORIZATION_SOURCE src/point_categorization.cpp ${COMMON_SOURCE})

add_executable(point_categorization src/point_categorization.cpp src/point_categories.hpp)
#add_executable(algorithm_simulation src/algorithm_simulation.cpp)

set_property(TARGET point_categorization PROPERTY CXX_STANDARD 20)

target_link_libraries(point_categorization ${PCL_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(algorithm_simulation ${PCL_LIBRARIES} ${Boost_LIBRARIES})
