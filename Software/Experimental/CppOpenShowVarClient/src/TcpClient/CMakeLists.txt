set(LIBRARY_SOURCES 
    "TcpClient.cpp" )


add_library(${TCPLIBRARY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
target_link_libraries(${TCPLIBRARY_NAME} ${BOOST_LIBRARIES})
