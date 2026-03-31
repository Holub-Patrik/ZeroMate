CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(kiv_os_rpios CXX C ASM)

# specifikujeme linker skript - potrebujeme vlastni definici pametoveho layoutu
SET(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/kernel/link.ld")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT} -nostdlib")

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/kernel/include/")

# budeme vyvijet jen pro RPi0, takze tady proste natvrdo napiseme cesty k hlavickam a pridame define
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/kernel/include/board/rpi0/")
ADD_DEFINITIONS(-DRPI0)

# rekurzivne pridame vsechny zdrojaky z kernel podslozky
FILE(GLOB_RECURSE kernel_src "kernel/src/*.cpp" "kernel/src/*.c" "kernel/src/*.h" "kernel/src/*.hpp" "kernel/src/*.s")
FILE(GLOB_RECURSE local_src "local/*.cpp" "local/*.c" "local/*.h" "local/*.hpp" "local/*.s")
FILE(GLOB_RECURSE remote_src "remote/*.cpp" "remote/*.c" "remote/*.h" "remote/*.hpp" "remote/*.s")

# pridame cil - sestavujeme jen kernel
ADD_EXECUTABLE(kernel_local ${local_src} ${kernel_src})
TARGET_LINK_LIBRARIES(kernel_local gcc)

ADD_EXECUTABLE(kernel_remote ${remote_src} ${kernel_src})
TARGET_LINK_LIBRARIES(kernel_remote gcc)

# po dokonceni sestaveni musime jeste zkonvertovat vystupy
# 1) zkompilovany ELF soubor na surovou binarni mapu pro RPi bootloader (kernel.img)
# 2) pro pruzkumne ucely i disassembly vysledne binarky
# 3) SREC dump pro UART bootloader
ADD_CUSTOM_COMMAND(
    TARGET kernel_local POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ./kernel_local${CMAKE_EXECUTABLE_SUFFIX} -O binary ./kernel_local.img
    COMMAND ${CMAKE_OBJDUMP} -l -S -D ./kernel_local${CMAKE_EXECUTABLE_SUFFIX} > ./kernel_local.asm
	COMMAND ${CMAKE_OBJCOPY} --srec-forceS3 ./kernel_local${CMAKE_EXECUTABLE_SUFFIX} -O srec kernel_local.srec
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

ADD_CUSTOM_COMMAND(
    TARGET kernel_remote POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ./kernel_remote${CMAKE_EXECUTABLE_SUFFIX} -O binary ./kernel_remote.img
    COMMAND ${CMAKE_OBJDUMP} -l -S -D ./kernel_remote${CMAKE_EXECUTABLE_SUFFIX} > ./kernel_remote.asm
	COMMAND ${CMAKE_OBJCOPY} --srec-forceS3 ./kernel_remote${CMAKE_EXECUTABLE_SUFFIX} -O srec kernel_remote.srec
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
