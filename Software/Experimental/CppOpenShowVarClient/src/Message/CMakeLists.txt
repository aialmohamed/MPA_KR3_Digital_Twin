set(LIBRARY_SOURCES 
    "Message.cpp" )


add_library(${MESSAGELIBRARY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
#target_link_libraries(${MESSAGELIBRARY_NAME} ${BOOST_LIBRARIES})
