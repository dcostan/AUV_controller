cmake_minimum_required(VERSION 3.5)

project(AUV_controller VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets Network LinguistTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network LinguistTools)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz-transport14 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

set(TS_FILES AUV_controller_it_IT.ts)

set(PROJECT_SOURCES
        main.cpp
        controlwindow.cpp
        controlwindow.h
        controlwindow.ui
        ${TS_FILES}
)

add_executable(AUV_controller
    ${PROJECT_SOURCES}
    rosnode.h rosnode.cpp
    gazebonode.h gazebonode.cpp
)

target_link_libraries(AUV_controller Qt${QT_VERSION_MAJOR}::Widgets)

target_link_libraries(AUV_controller Qt${QT_VERSION_MAJOR}::Network)

target_link_libraries(AUV_controller gz-transport${GZ_TRANSPORT_VER}::core)

ament_target_dependencies(AUV_controller rclcpp std_msgs)

include(GNUInstallDirs)
install(TARGETS AUV_controller
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(AUV_controller)
endif()
