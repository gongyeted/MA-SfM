# CMake generated Testfile for 
# Source directory: /home/gy/DAGSfM_merge_v3/src/rotation_estimation
# Build directory: /home/gy/DAGSfM_merge_v3/src/rotation_estimation
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(rotation_estimation/align_rotations_test "align_rotations_test")
set_tests_properties(rotation_estimation/align_rotations_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;12;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;0;")
add_test(rotation_estimation/robust_rotation_estimator_test "robust_rotation_estimator_test")
set_tests_properties(rotation_estimation/robust_rotation_estimator_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;13;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;0;")
add_test(rotation_estimation/lagrange_dual_rotation_estimator_test "lagrange_dual_rotation_estimator_test")
set_tests_properties(rotation_estimation/lagrange_dual_rotation_estimator_test PROPERTIES  _BACKTRACE_TRIPLES "/home/gy/DAGSfM_merge_v3/cmake/CMakeHelper.cmake;153;add_test;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;14;COLMAP_ADD_TEST;/home/gy/DAGSfM_merge_v3/src/rotation_estimation/CMakeLists.txt;0;")
