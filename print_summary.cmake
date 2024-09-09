message("=== Build Summary ===")
file(GLOB_RECURSE ELF_FILES "${CMAKE_BINARY_DIR}/*.elf")
#sort ELF_FILES
list(SORT ELF_FILES)
# iterate over all the files
foreach(file ${ELF_FILES})
# invoke the size utility to get the size of the file
  execute_process(COMMAND $ENV{CMAKE_SIZE_UTIL} -B ${file})
endforeach()
