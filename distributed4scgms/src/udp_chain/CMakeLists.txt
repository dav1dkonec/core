CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
SET(PROJ "tchain_udp")

file(GLOB SRC_FILES "./*.cpp" "./*.c" "./*.h")

IF(WIN32)
    FILE(GLOB SRC_WIN_FILES "./win/*.cpp" "./win/*.h" "./win/*.def")
    SET(SRC_FILES "${SRC_FILES};${SRC_WIN_FILES}")
ENDIF()

SCGMS_ADD_LIBRARY(${PROJ} SHARED ${SRC_FILES})

# Same as tproblem_udp, plus scgms itself - this UDP builds and runs a filter chain
target_link_libraries(${PROJ} distributed pagmo)

SCGMS_LINK_COMMON_LIBRARIES(${PROJ})
APPLY_SCGMS_LIBRARY_BUILD_SETTINGS(${PROJ})

set_target_properties(${PROJ}
        PROPERTIES
        PREFIX ""
        OUTPUT_NAME ${PROJ}
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/compiled/"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/compiled/"
)

target_include_directories(${PROJ} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
