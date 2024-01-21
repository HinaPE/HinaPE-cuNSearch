set(TARGET SIM_cuNSearch)
# Find Houdini
if (MSVC)
    set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.590")
elseif (APPLE)
    set(Houdini_PATH "/Applications/Houdini/Houdini20.0.590/Frameworks/Houdini.framework/Versions/20.0/Resources")
endif ()

set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Load Source
add_library(
        ${TARGET}
        SHARED
        hdk/Entrance.cpp

        hdk/GAS_CUDA_BuildNeighborList.cpp
        hdk/GAS_CUDA_BuildNeighborList.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${TARGET}
        PUBLIC
        Houdini
        cuNSearch
)
target_link_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        hdk
)
target_compile_options(
        ${TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${TARGET})


# Load Source
add_executable(Test
        test.cpp
)
add_dependencies(Test cuNSearch)
target_link_libraries(Test PRIVATE cuNSearch)
