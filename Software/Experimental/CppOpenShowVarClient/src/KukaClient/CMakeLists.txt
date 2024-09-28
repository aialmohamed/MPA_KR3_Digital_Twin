set(LIBRARY_SOURCES 
    "KukaClient.cpp" )


add_library(${KUKACLIENTLIBARAY_NAME} STATIC
    ${LIBRARY_SOURCES})

# Link Boost libraries if required by TcpClient
target_link_libraries(${KUKACLIENTLIBARAY_NAME} 
                        ${BOOST_LIBRARIES}
                        ${TCPLIBRARY_NAME}
                        ${MESSAGELIBRARY_NAME}
                        ${WRTIEMESSAGELIBARAY_NAME}
                        ${READMESSAGELIBARAY_NAME}
                        ${RESPONSEMESSAGELIBARAY_NAME})