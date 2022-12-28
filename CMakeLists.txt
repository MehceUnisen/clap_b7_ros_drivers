cmake_minimum_required(VERSION 2.8.3)
project(rbf_clap_b7_driver)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        )

add_message_files(FILES
        ClapData.msg)
generate_messages(DEPENDENCIES
        std_msgs
        )
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
        include/rbf_clap_b7_driver
        ${catkin_INCLUDE_DIRS})

set(NTRIP_LIB_HEADER
        include/rbf_clap_b7_driver/ntrip/cmake_definition.h
        include/rbf_clap_b7_driver/ntrip/ntrip_client.h
        include/rbf_clap_b7_driver/ntrip/ntrip_util.h
        include/rbf_clap_b7_driver/ntrip/thread_raii.h
        )

set(CLAP_B7_DRIVER_LIB_HEADERS
        include/rbf_clap_b7_driver/TermiosSerial.h
        include/rbf_clap_b7_driver/AsyncSerial.h
        include/rbf_clap_b7_driver/ClapB7BinaryParser.h
        include/rbf_clap_b7_driver/ClapB7Driver.hpp
        include/rbf_clap_b7_driver/AgricInfoInterface.h
        )

add_executable(clap_b7_node
        src/AsyncSerial.cpp
        src/ClapB7BinaryParser.cpp
        src/ClapB7Driver.cpp
        src/ClapB7DriverNode.cpp
        src/ntrip_client.cpp
        src/ntrip_util.cpp
        src/TermiosSerial.cpp
        ${NTRIP_LIB_HEADER}
        ${CLAP_B7_DRIVER_LIB_HEADERS}
        )

target_link_libraries(clap_b7_node ${catkin_LIBRARIES})

add_dependencies(clap_b7_node
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS clap_b7_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
