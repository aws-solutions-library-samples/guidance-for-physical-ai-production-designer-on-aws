cmake_minimum_required(VERSION 3.14)
project(pinocchio_wrapper C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pinocchio REQUIRED)

# Collision support is gated at compile time so that non-collision builds are
# entirely unaffected (build.rs always invokes this CMake project). The
# `collision` Cargo feature flips `ENABLE_COLLISION` to ON via build.rs; when
# OFF we neither probe for nor link the Coal collision backend.
option(ENABLE_COLLISION "Build collision support and link the Coal (hpp-fcl) backend" OFF)

if(ENABLE_COLLISION)
    # Pinocchio delegates collision/distance queries to Coal (formerly
    # HPP-FCL). Try the modern `coal` package first, then fall back to
    # `hpp-fcl` on older installs. Fail fast at configure time with a clear
    # message naming the missing dependency when neither is available.
    find_package(coal QUIET)
    if(coal_FOUND)
        set(COLLISION_BACKEND_TARGET coal::coal)
    else()
        find_package(hpp-fcl QUIET)
        if(hpp-fcl_FOUND)
            set(COLLISION_BACKEND_TARGET hpp-fcl::hpp-fcl)
        else()
            message(FATAL_ERROR
                "Collision support is enabled (ENABLE_COLLISION=ON) but the Coal "
                "collision dependency could not be found. Install the Coal "
                "development package (CMake package `coal`, imported target "
                "`coal::coal`) or the legacy `hpp-fcl` package (imported target "
                "`hpp-fcl::hpp-fcl`), or build without the `collision` feature.")
        endif()
    endif()
endif()

add_library(pinocchio_wrapper STATIC pinocchio_wrapper.cpp)
target_include_directories(pinocchio_wrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pinocchio_wrapper PUBLIC pinocchio::pinocchio)

if(ENABLE_COLLISION)
    target_compile_definitions(pinocchio_wrapper PUBLIC PINOCCHIO_WRAPPER_COLLISION)
    target_link_libraries(pinocchio_wrapper PUBLIC ${COLLISION_BACKEND_TARGET})
endif()

install(TARGETS pinocchio_wrapper ARCHIVE DESTINATION lib)
