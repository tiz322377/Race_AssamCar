# 方式1：使用 GLOB_RECURSE（不推荐，但简单）
file(GLOB_RECURSE USER_SOURCES
        "User/*.cpp"
        "Program/*.cpp"
)

add_library(UserLib)

target_sources(UserLib PRIVATE ${USER_SOURCES})

include_directories(
        "User"
        "Program"
)

target_link_libraries(UserLib PRIVATE stm32cubemx)

