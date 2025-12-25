
set(CMAKE_CXX_STANDARD 17)

add_library(src
        decomposer.cpp
        path_generator.cpp
        PathFinderCalculator.cc
)

target_link_libraries(src
        PUBLIC
        Qt5::Core
        Qt5::Gui
)

target_include_directories(src
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
