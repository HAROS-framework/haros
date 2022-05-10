# Copyright (c) 2020 by Robert Bosch GmbH. All rights reserved.
# Copyright (c) 2020 - 2021 by Apex.AI Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

set(IOX_VERSION_STRING "2.90.0")

project(iceoryx_dds VERSION ${IOX_VERSION_STRING})

find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)

include(IceoryxPackageHelper)
include(IceoryxPlatform)

option(MAKE_UNIQUE_INCLUDEDIR
  "When ON headers are installed to a path ending with folders called \
  iceoryx/vX.Y.Z/ . This avoids include directory search order issues when \
  overriding this package from a merged catkin, ament, or colcon workspace."
  ON)

if(MAKE_UNIQUE_INCLUDEDIR)
  set(_unique_dir "iceoryx/v${IOX_VERSION_STRING}")
  if(PREFIX STREQUAL "")
    set(PREFIX "${_unique_dir}")
  else()
    set(PREFIX "${PREFIX}/${_unique_dir}")
  endif()
endif()

if(CMAKE_SYSTEM_NAME MATCHES Linux OR CMAKE_SYSTEM_NAME MATCHES Darwin)
    option(BUILD_SHARED_LIBS "Create shared libraries by default" ON)
endif()

#
########## set variables for export ##########
#
setup_package_name_and_create_files(
    NAME ${PROJECT_NAME}
    NAMESPACE iceoryx_dds
    PROJECT_PREFIX ${PREFIX}
)

#
########## feature flags ##########
#
option(USE_CYCLONE_DDS "Bind to CycloneDDS implementation" ON)

if(USE_CYCLONE_DDS)
    message(INFO " Using CycloneDDS stack")
    find_package(CycloneDDS CONFIG REQUIRED)
    find_package(CycloneDDS-CXX CONFIG REQUIRED)
endif()

#
########## build building-block library ##########
#
add_library(iceoryx_dds
    source/iceoryx_dds/log/logging.cpp
)
add_library(${PROJECT_NAMESPACE}::iceoryx_dds ALIAS iceoryx_dds)

set_target_properties(iceoryx_hoofs PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_STANDARD ${ICEORYX_CXX_STANDARD}
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(iceoryx_dds
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/dependencies/install/include>
    $<INSTALL_INTERFACE:include/${PREFIX}>
)
target_link_libraries(iceoryx_dds
    PRIVATE
    iceoryx_posh::iceoryx_posh
    iceoryx_posh::iceoryx_posh_config
    iceoryx_posh::iceoryx_posh_gateway
    iceoryx_hoofs::iceoryx_hoofs
)

if(USE_CYCLONE_DDS)
    target_sources(iceoryx_dds
        PRIVATE
        source/iceoryx_dds/dds/cyclone_context.cpp
        source/iceoryx_dds/dds/cyclone_data_reader.cpp
        source/iceoryx_dds/dds/cyclone_data_writer.cpp
        source/iceoryx_dds/dds/iox_chunk_datagram_header.cpp
    )

    # Generate IDL at configure time
    set(MESSAGE_DEFINITION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
    get_filename_component(MEMPOOL_IDL "${MESSAGE_DEFINITION_DIR}/Mempool.idl" ABSOLUTE)

    add_custom_command(
        OUTPUT "Mempool.hpp"
        COMMAND CycloneDDS::idlc
        ARGS -l $<TARGET_FILE:CycloneDDS-CXX::idlcxx> ${MEMPOOL_IDL}
        DEPENDS CycloneDDS::idlc CycloneDDS-CXX::idlcxx
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating serialization for Mempool bytestream"
        VERBATIM
    )

    add_custom_target(mempool_header ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Mempool.hpp)

    target_include_directories(iceoryx_dds PUBLIC
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
        $<INSTALL_INTERFACE:include/${PREFIX}>
    )

    target_compile_definitions(iceoryx_dds PUBLIC -DUSE_CYCLONE_DDS)
    target_link_libraries(iceoryx_dds
        PUBLIC
        CycloneDDS-CXX::ddscxx
    )
endif()

target_compile_options(iceoryx_dds PRIVATE ${ICEORYX_WARNINGS} ${ICEORYX_SANITIZER_FLAGS})

#
########## build gateway app ##########
#
add_executable(iox-dds-gateway
    source/gateway/main.cpp
)
target_link_libraries(iox-dds-gateway
    iceoryx_posh::iceoryx_posh
    iceoryx_posh::iceoryx_posh_gateway
    iceoryx_posh::iceoryx_posh_config
    ${PROJECT_NAMESPACE}::iceoryx_dds
)

target_compile_options(iox-dds-gateway PRIVATE ${ICEORYX_WARNINGS} ${ICEORYX_SANITIZER_FLAGS})

set_target_properties(iox-dds-gateway iceoryx_dds
  PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_STANDARD ${ICEORYX_CXX_STANDARD}
)

#
########## build test executables ##########
#
if(BUILD_TEST)
    add_subdirectory(test)
endif()

#
########## export library ##########
#
setup_install_directories_and_export_package(
    TARGETS iceoryx_dds iox-dds-gateway
    INCLUDE_DIRECTORY include/
)
