
set(CMAKE_CXX_STANDARD 17)

add_library(src
        decomposer.cpp
        path_generator.cpp
        PathFinderCalculator.cc
        PolyBuilder.cpp
)


find_package(Clipper2 CONFIG REQUIRED)

target_link_libraries(src
        PUBLIC
        Qt5::Core
        Qt5::Gui
        Clipper2::Clipper2
)

target_include_directories(src
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
