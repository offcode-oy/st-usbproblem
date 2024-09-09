# Cmake root file to build the Boot and Appli projects in lift-devkit folder

cmake_minimum_required(VERSION 3.27)

include(toolchain.cmake)
include(utils.cmake)

project(lift-devkit C ASM)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_C_COMPILER_TARGET arm-none-eabi)
set(CMAKE_CXX_COMPILER_TARGET arm-none-eabi)

# execute generate_cmake.py to generate the cmake files, do it only for configuration step
execute_process(COMMAND python3 ${CMAKE_SOURCE_DIR}/generate_cmake.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# find all files with pattern generated_*.cmake and include them
file(GLOB GENERATED_CMAKE_FILES "generated_*.cmake")
foreach(GENERATED_CMAKE_FILE ${GENERATED_CMAKE_FILES})
    include(${GENERATED_CMAKE_FILE})
endforeach()

# read the content of generated_all_projects.cmake and include it
include(generated_all_projects.cmake)
message(STATUS "ALL_PROJECTS: ${ALL_PROJECTS}")

# Some defaults if not provided
set(DISPLAY_SPI_MODE "normal" CACHE STRING "Set the SPI mode for the display") # normal or bitbang or qspi

# Common configuration for all projects
set(CPU_FLAGS -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb)
set(COMPILER_FLAGS -ffunction-sections -fdata-sections -Wall -fstack-usage -Wno-int-to-pointer-cast)
set(LINKER_FLAGS --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -Wl,--start-group -lc -lm -Wl,--end-group)

# Set an override to keep the StorageInfo symbol in the binary
set(LINKER_FLAGS_usbprob_ExtMemLoader --specs=nosys.specs  -Wl,--require-defined=StorageInfo -Wl,--gc-sections -static --specs=nano.specs -Wl,--start-group -lc -lm -Wl,--end-group)
# if debug build
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
# Set debugging for the boot image
set(COMPILER_FLAGS_usbprob_Boot -DEXTMEM_DRIVER_NOR_SFDP_DEBUG_LEVEL=1 -DEXTMEM_MACRO_DEBUG=printf)
endif()

foreach(PROJECT ${ALL_PROJECTS})
# check if override exists for the project LINKER_FLAGS_<PROJECT> and CPU_FLAGS_<PROJECT> and use them
if(DEFINED "LINKER_FLAGS_${PROJECT}")
set(LINKER_FLAGS_PROJ ${LINKER_FLAGS_${PROJECT}})
else()
set(LINKER_FLAGS_PROJ ${LINKER_FLAGS})
endif()
if(DEFINED "CPU_FLAGS_${PROJECT}")
set(CPU_FLAGS_PROJ ${CPU_FLAGS_${PROJECT}})
else()
set(CPU_FLAGS_PROJ ${CPU_FLAGS})
endif()
if(DEFINED "COMPILER_FLAGS_${PROJECT}")
set(COMPILER_FLAGS_PROJ ${COMPILER_FLAGS_${PROJECT}})
else()
set(COMPILER_FLAGS_PROJ ${COMPILER_FLAGS})
endif()
target_link_options(${PROJECT} PRIVATE ${CPU_FLAGS_PROJ} ${LINKER_FLAGS_PROJ} -Wl,-Map=${PROJECT}.map )
target_compile_options(${PROJECT} PRIVATE ${CPU_FLAGS_PROJ} ${COMPILER_FLAGS_PROJ})
generate_object(${PROJECT} .bin binary)
generate_object(${PROJECT} .hex ihex)
endforeach()

#### Customization for some projects ####

# append to the usbprob_Appli project the DISPLAY_SPI_MODE flag
target_compile_definitions(usbprob_Appli PRIVATE DISPLAY_SPI_MODE_${DISPLAY_SPI_MODE})

# Add the new remote_peripherals project, uses default FLAGS
# add_subdirectory(remote_peripherals)
# target_compile_options(remote_peripherals PRIVATE ${CPU_FLAGS} ${COMPILER_FLAGS})

# Add the remote_peripherals library to the usbprob_Appli project
# target_link_libraries(usbprob_Appli PUBLIC remote_peripherals)
# target_include_directories(usbprob_Appli PRIVATE $<TARGET_PROPERTY:remote_peripherals,INTERFACE_INCLUDE_DIRECTORIES>)

# g_pfnVectors array is variable and unknown size, so we need to disable the warning
set_source_files_properties(Middlewares/ST/STM32_ExtMem_Loader/STM32Cube/stm32_loader_api.c TARGET_DIRECTORY usbprob_ExtMemLoader PROPERTIES COMPILE_FLAGS "-Wno-pedantic -Wno-vla")

# After the usbprob_ExtMemLoader project is built, copy the elf file as usbprob.stldr, use file copy
add_custom_command(TARGET usbprob_ExtMemLoader POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:usbprob_ExtMemLoader> ${CMAKE_BINARY_DIR}/usbprob.stldr
)

# Will print a summary of the build, send the CMAKE_SIZE_UTIL variable to the print_summary.cmake file
add_custom_target(print_summary ALL
    COMMAND ${CMAKE_COMMAND} -E env CMAKE_SIZE_UTIL=${CMAKE_SIZE_UTIL} ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/print_summary.cmake
    DEPENDS ${ALL_PROJECTS}
)