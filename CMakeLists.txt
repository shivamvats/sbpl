cmake_minimum_required(VERSION 2.8)
PROJECT( sbpl )

# version (e.g. for packaging)
set(SBPL_MAJOR_VERSION 1)
set(SBPL_MINOR_VERSION 3)
set(SBPL_PATCH_VERSION 0)
set(SBPL_VERSION ${SBPL_MAJOR_VERSION}.${SBPL_MINOR_VERSION}.${SBPL_PATCH_VERSION})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(src/include)

if(MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(SBPL_SOURCE_FILES 
    src/discrete_space_information/environment_nav2D.cpp
    src/discrete_space_information/environment_navxythetalat.cpp
    src/discrete_space_information/environment_navxythetamlevlat.cpp
    src/discrete_space_information/environment_nav2Duu.cpp
    src/discrete_space_information/environment_XXX.cpp
    src/discrete_space_information/environment_robarm.cpp
    src/heuristics/embedded_heuristic.cpp
    src/planners/adplanner.cpp
    src/planners/ANAplanner.cpp
    src/planners/araplanner.cpp
    src/planners/lazyARA.cpp
    src/planners/mhaplanner.cpp
    src/planners/ppcpplanner.cpp
    src/planners/rstarplanner.cpp
    src/planners/viplanner.cpp
    src/utils/heap.cpp
    src/utils/mdp.cpp
    src/utils/utils.cpp
    src/utils/2Dgridsearch.cpp
    src/utils/config.cpp)

set(CMAKE_DEBUG_POSTFIX "_d")

add_library(sbpl SHARED ${SBPL_SOURCE_FILES})
set_target_properties(
    sbpl
    PROPERTIES
        VERSION ${SBPL_VERSION}
        SOVERSION ${SBPL_MAJOR_VERSION}.${SBPL_MINOR_VERSION})

add_library(sbpl-static STATIC ${SBPL_SOURCE_FILES})
set_target_properties(sbpl-static PROPERTIES OUTPUT_NAME sbpl)
if(MSVC)
    set_target_properties(sbpl-static PROPERTIES PREFIX lib)
endif()

set(SBPL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(SBPL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

# support for pkg-config
set(PKG_NAME "sbpl")
set(PKG_DESC "The Search Based Planning Library")
set(PKG_SBPL_LIBS "-lsbpl")
set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/sbpl.pc.in")
set(pkg_conf_file "${PROJECT_BINARY_DIR}/InstallFiles/sbpl.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file}" @ONLY)
install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

# support for cmake-config:
configure_file(
    sbpl-config.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake"
    @ONLY)
configure_file(
    sbpl-config-version.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake"
    @ONLY)
install(
    FILES
        "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake"
        "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake"
    DESTINATION share/sbpl/)

install(
    DIRECTORY src/include/sbpl
    DESTINATION include
	FILES_MATCHING PATTERN "*.h")
install(
    TARGETS sbpl sbpl-static
    EXPORT sbpl-targets
	LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(EXPORT sbpl-targets DESTINATION share/sbpl/ COMPONENT dev)

add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

