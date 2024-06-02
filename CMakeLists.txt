cmake_minimum_required(VERSION 3.6.0)
project(test_httplib)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17) # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON) # ...is required...
set(CMAKE_CXX_EXTENSIONS OFF) # ...without compiler extensions like gnu++11
find_package(Threads REQUIRED)

add_subdirectory(lib/cpp-httplib)

# add_subdirectory(lib/json)
include_directories(${PROJECT_NAME}
    lib/cpp-httplib

    # lib/json/include
)

set(SRC
    src/main.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -pedantic")

# # Define the executable
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME}
    Threads::Threads
    httplib
)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

# サブディレクトリを登録
# add_subdirectory(src)
# add_subdirectory(test)
