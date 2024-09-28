set(LIBRARY_SOURCES 
    "ReadMessage.cpp" )


add_library(${READMESSAGELIBARAY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
target_link_libraries(${READMESSAGELIBARAY_NAME} ${MESSAGELIBRARY_NAME})
