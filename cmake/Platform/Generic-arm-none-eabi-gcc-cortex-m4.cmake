
message(STATUS "arm-none-eabi cortex-m4")

set(cpu_flags "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE INTERNAL "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__ " CACHE INTERNAL "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${cpu_flags}" CACHE INTERNAL "" FORCE)
