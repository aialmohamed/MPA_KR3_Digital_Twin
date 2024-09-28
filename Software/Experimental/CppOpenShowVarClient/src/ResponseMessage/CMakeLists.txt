set(LIBRARY_SOURCES 
    "ResponseMessage.cpp" )


add_library(${RESPONSEMESSAGELIBARAY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
target_link_libraries(${RESPONSEMESSAGELIBARAY_NAME} ${MESSAGELIBRARY_NAME})
