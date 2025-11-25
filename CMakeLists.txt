cmake_minimum_required(VERSION 3.28)

# Library target
add_library(ADS1262 STATIC)
add_library(ADS1262::ADS1262 ALIAS ADS1262)

# Languages
enable_language(C ASM)

# Require C11 at the target level
target_compile_features(ADS1262 PUBLIC c_std_11)

# Include directories (for this target and consumers)
target_include_directories(ADS1262
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Inc
    PRIVATE
        ${CMAKE_SOURCE_DIR}/App/Inc
)

# Sources
target_sources(ADS1262 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Src/ADS1262.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ADS1262_port.c
)

# Optional: expose headers as a file set (useful for IDEs/installs)
if(POLICY CMP0135)
    target_sources(ADS1262 PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/Inc
        FILES
            ${CMAKE_CURRENT_LIST_DIR}/Inc/ADS1262.h
            ${CMAKE_CURRENT_LIST_DIR}/Inc/ADS1262_port.h
    )
endif()

# Inherit STM32/HAL/FreeRTOS include paths and macros
target_link_libraries(ADS1262 PUBLIC stm32cubemx)
