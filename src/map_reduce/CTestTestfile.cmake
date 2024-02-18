# CMake generated Testfile for 
# Source directory: /home/gy/DAGSfM_merge_v3/src/map_reduce
# Build directory: /home/gy/DAGSfM_merge_v3/src/map_reduce
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(map_reduce/master_test "master_test")
set_tests_properties(map_reduce/master_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;14;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;0;")
add_test(map_reduce/worker_test "worker_test")
set_tests_properties(map_reduce/worker_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;15;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;0;")
add_test(map_reduce/msgpack_adaptor_test "msgpack_adaptor_test")
set_tests_properties(map_reduce/msgpack_adaptor_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;16;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/map_reduce/CMakeLists.txt;0;")
