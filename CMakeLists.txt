cmake_minimum_required(VERSION 3.0.0)
project(test1)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17) # C++17...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
find_package(Threads REQUIRED)

set(SRC
    main.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -pedantic")

## Define the executable
add_executable(test1 ${SRC})
target_link_libraries(test1 Threads::Threads)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

# サブディレクトリを登録
# add_subdirectory(src)
# add_subdirectory(test)