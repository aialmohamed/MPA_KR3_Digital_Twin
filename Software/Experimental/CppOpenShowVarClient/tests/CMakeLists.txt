Include(FetchContent)


# For unit tests
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)


# For Integration test 
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(TestReadMessage)
add_subdirectory(TestWriteMessage)
add_subdirectory(TestResponseMessage)
add_subdirectory(TestTcpClient)
add_subdirectory(TestKukaClient)
add_subdirectory(TestMYAXISType)