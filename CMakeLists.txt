cmake_minimum_required(VERSION 3.5)
project(TryCelluarPathCompose)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
else()
    add_definitions(-D_USE_MATH_DEFINES)  # Для MinGW
endif()

set(PROG_INCLUDE    ${CMAKE_CURRENT_SOURCE_DIR}/src
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${QtQml_INCLUDE_DIRS})

set(Qt_RESOURCES "${CMAKE_CURRENT_SOURCE_DIR}/qresource.qrc")

find_package(Qt5 COMPONENTS Core Quick QuickControls2 REQUIRED)

add_subdirectory(src)

add_executable(TryCelluarPathCompose main.cpp
        ${Qt_RESOURCES})

foreach (name ${Qt_RESOURCES})
    message(STATUS "${name}")
endforeach ()


target_link_libraries(TryCelluarPathCompose
        PUBLIC
        Qt5::Core
        Qt5::Quick
        Qt5::QuickControls2
        PRIVATE
        src
)

include_directories(${PROG_INCLUDE})