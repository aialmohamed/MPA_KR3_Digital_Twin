set(LIBRARY_SOURCES 
    "WriteMessage.cpp" )


add_library(${WRTIEMESSAGELIBARAY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
target_link_libraries(${WRTIEMESSAGELIBARAY_NAME} ${MESSAGELIBRARY_NAME})
