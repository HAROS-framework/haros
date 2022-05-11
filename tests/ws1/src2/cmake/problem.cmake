if(MAKE_UNIQUE_INCLUDEDIR)
  set(_unique_dir "iceoryx/v${IOX_VERSION_STRING}")
  if(PREFIX STREQUAL "")
    set(PREFIX "${_unique_dir}")
  else()
    set(PREFIX "${PREFIX}/${_unique_dir}")
  endif()
endif()

if(USE_CYCLONE_DDS)
    add_custom_command(
        OUTPUT "Mempool.hpp"
        COMMAND CycloneDDS::idlc
        ARGS -l $<TARGET_FILE:CycloneDDS-CXX::idlcxx> ${MEMPOOL_IDL}
        DEPENDS CycloneDDS::idlc CycloneDDS-CXX::idlcxx
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating serialization for Mempool bytestream"
        VERBATIM
    )
