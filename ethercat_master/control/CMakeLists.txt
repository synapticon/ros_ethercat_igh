project(test_master_lib)

file(GLOB src "*.cpp")
file(GLOB inc "*.h")

set(test_master_src ${test_master_src} ${src} CACHE INTERNAL "")
set(test_master_inc ${test_master_inc} ${inc} CACHE INTERNAL "")
