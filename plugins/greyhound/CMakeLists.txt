#
# Greyhound plugin CMake configuration
#

if(JSONCPP_FOUND)
    # add JSONCPP stuff
    include_directories(${JSONCPP_INCLUDE_DIR})
    add_definitions(-DHAVE_JSONCPP=1)

    set(srcs
        io/CompressionStream.cpp
        io/Exchanges.cpp
        io/GreyhoundReader.cpp
    )

    set(incs
        io/CompressionStream.hpp
        io/Exchanges.hpp
        io/GreyhoundReader.hpp
    )

    PDAL_ADD_PLUGIN(libname reader greyhound
        FILES "${srcs}" "${incs}"
        LINK_WITH ${JSONCPP_LIBRARY})
endif()
