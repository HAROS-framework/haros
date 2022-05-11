add_custom_command(
    OUTPUT "Mempool.hpp"
    COMMAND CycloneDDS::idlc
    ARGS -l $<TARGET_FILE:CycloneDDS-CXX::idlcxx> ${MEMPOOL_IDL}
    DEPENDS CycloneDDS::idlc CycloneDDS-CXX::idlcxx
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating serialization for Mempool bytestream"
    VERBATIM
)
