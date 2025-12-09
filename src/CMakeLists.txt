
add_library(src
        decomposer.cpp
        decomposer.h
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
