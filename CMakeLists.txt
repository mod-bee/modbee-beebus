cmake_minimum_required(VERSION 3.14)

project(BeeBus
    VERSION 0.1.0
    DESCRIPTION "Peer-to-peer token-passing protocol for serial and UDP links"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Examples and tests default on only when BeeBus is the top-level project, so a
# parent that pulls it in with add_subdirectory() gets just the library.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(BEEBUS_TOP_LEVEL ON)
else()
    set(BEEBUS_TOP_LEVEL OFF)
endif()

option(BEEBUS_BUILD_EXAMPLES "Build the plain C++ examples" ${BEEBUS_TOP_LEVEL})
option(BEEBUS_BUILD_TESTS    "Build the tests, when a test/ directory is present" ${BEEBUS_TOP_LEVEL})
option(BEEBUS_SANITIZE       "Build with the address and undefined behaviour sanitizers" OFF)
option(BEEBUS_SANITIZE_THREAD "Build with the thread sanitizer (excludes BEEBUS_SANITIZE)" OFF)

# The library parses bytes it did not produce, so out-of-bounds reads are the
# failure mode that matters most and the one least likely to show as a wrong
# answer.
if(BEEBUS_SANITIZE)
    if(MSVC)
        add_compile_options(/fsanitize=address)
    else()
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# A host runtime may drive one node from two threads -- a fast task servicing it
# while the application blocks call from a slower one. The address sanitizer says
# nothing about that; only the thread sanitizer does, and the two cannot be built
# together.
if(BEEBUS_SANITIZE_THREAD)
    if(BEEBUS_SANITIZE)
        message(FATAL_ERROR "BEEBUS_SANITIZE and BEEBUS_SANITIZE_THREAD cannot be combined")
    endif()
    if(NOT MSVC)
        add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=thread)
    endif()
endif()

# The host build serialises a node's API against its own loop() with a mutex,
# so it needs the threading library.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# =============================================================================
# LIBRARY
# =============================================================================

# Listed explicitly rather than globbed, so a new file is a deliberate change
# and CMake never silently misses one.
set(BEEBUS_SOURCES
    src/BeeBusPlatform.cpp
    src/BeeBusAPI.cpp
    src/BeeBusDebug.cpp
    src/BeeBusFrame.cpp
    src/BeeBusIO.cpp
    src/BeeBusOperations.cpp
    src/BeeBusProtocol.cpp
    src/BeeBusRing.cpp
    src/BeeBusDiscovery.cpp
    src/BeeBusTopicFrame.cpp
    src/BeeBusTopicMap.cpp
    src/BeeBusTopicHandler.cpp
    src/BeeBusTopics.cpp
    src/BeeBusTopicAPI.cpp
    src/BeeBusStreams.cpp
    src/BeeBusBinding.cpp
    src/BeeBusModbusDataMap.cpp
    src/BeeBusModbusFrame.cpp
    src/BeeBusModbusHandler.cpp
    src/transport/BeeBusHostSerialTransport.cpp
    src/transport/BeeBusHostUdpTransport.cpp
)

add_library(beebus STATIC ${BEEBUS_SOURCES})
add_library(BeeBus::beebus ALIAS beebus)

target_include_directories(beebus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/beebus>
)

target_link_libraries(beebus PUBLIC Threads::Threads)

if(WIN32)
    target_link_libraries(beebus PUBLIC ws2_32)
endif()

if(MSVC)
    target_compile_options(beebus PRIVATE /W4)
else()
    target_compile_options(beebus PRIVATE -Wall -Wextra)
endif()

# =============================================================================
# EXAMPLES
# =============================================================================

if(BEEBUS_BUILD_EXAMPLES)
    add_executable(beebus_serial_node examples/cpp/serial_node/main.cpp)
    target_link_libraries(beebus_serial_node PRIVATE beebus)

    add_executable(beebus_udp_node examples/cpp/udp_node/main.cpp)
    target_link_libraries(beebus_udp_node PRIVATE beebus)

    add_executable(beebus_topic_node examples/cpp/topic_node/main.cpp)
    target_link_libraries(beebus_topic_node PRIVATE beebus)

    add_executable(beebus_gateway_node examples/cpp/gateway_node/main.cpp)
    target_link_libraries(beebus_gateway_node PRIVATE beebus)
endif()

# =============================================================================
# TESTS
# =============================================================================

# Only built when a test/ directory is present.
if(BEEBUS_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt)
    enable_testing()
    add_subdirectory(test)
endif()
