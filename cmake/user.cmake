# 递归收集人工维护的 C++ 源文件；新增文件时自动触发 CMake 重配置。
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/../User/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../Program/*.cpp"
)

add_library(UserLib)

target_sources(UserLib PRIVATE ${USER_SOURCES})

include_directories(
        "User"
        "Program"
)

target_link_libraries(UserLib PRIVATE stm32cubemx)

