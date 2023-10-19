cmake_minimum_required(VERSION 3.17)

project(CalibrationSolver)

set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_LIBDIR "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Matlab REQUIRED )

add_library(calibration_solver STATIC  )

target_sources(calibration_solver PUBLIC
src/cpp_calibration_solver.cpp
)

target_include_directories(calibration_solver
PUBLIC
# where the library itself will look for its internal headers
${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(calibration_solver PUBLIC
Ceres::ceres
Eigen3::Eigen
)

if (Matlab_FOUND)
    message(STATUS "MATLAB root directory found: ${Matlab_ROOT_DIR}.")
    matlab_add_mex(
        NAME CalibrationSolver
        SRC src/mex_calibration_solver.cpp
        LINK_TO Matlab::mex calibration_solver
        )


    #build to bin
    set_target_properties( CalibrationSolver
        PROPERTIES
        COMPILE_FLAGS "-fvisibility=default"
        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

    message(STATUS "Mex file can be found in ${PROJECT_SOURCE_DIR}/bin")
else ()
    message(WARNING "MATLAB root directory not found. Failed to build MATLAB bindings.")
    set(BUILD_MATLAB_BINDINGS OFF)
endif ()



