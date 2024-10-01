# doc/CMakeLists.txt

# Require minimum version of CMake
cmake_minimum_required(VERSION 3.10)

# Project name
project(CppOpenShowVarClientDocs)

# Find Doxygen package
find_package(Doxygen REQUIRED)

# Set input and output files for Doxygen
set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../Doxyfile)
set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/DoxygenDocs)

# Configure Doxygen to use the Doxyfile
add_custom_target(Doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_IN}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
)

# Specify that the Doxygen documentation depends on the library
add_dependencies(Doxygen ${TCPLIBRARY_NAME}
                        ${MESSAGELIBRARY_NAME}
                        ${WRTIEMESSAGELIBARAY_NAME}
                        ${READMESSAGELIBARAY_NAME}
                        ${RESPONSEMESSAGELIBARAY_NAME})