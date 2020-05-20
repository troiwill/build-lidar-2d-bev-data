find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(OpenCV 4.2 REQUIRED)

find_package(Boost 1.72.0 COMPONENTS program_options filesystem REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(KITTI_SOURCES build_kitti_bev_data.cpp kitti_helpers.cpp bev2d_helpers.cpp)
set(KITTI_LINK_LIBS ${Boost_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)
add_executable(build_kitti_bev_data ${KITTI_SOURCES})
target_link_libraries(build_kitti_bev_data ${KITTI_LINK_LIBS})

set(GEN_KITTI_DATA_SRCS generate_learn_data.cpp kitti_helpers.cpp)
set(GEN_KITTI_DATA_LINK_LIBS ${Boost_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)
add_executable(generate_learn_data ${GEN_KITTI_DATA_SRCS})
target_link_libraries(generate_learn_data ${GEN_KITTI_DATA_LINK_LIBS})
