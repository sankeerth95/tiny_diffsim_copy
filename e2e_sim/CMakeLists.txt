


add_executable(e2e_diff 
e2e_diff.cpp 
${TDS_HDRS})
target_include_directories(e2e_diff PRIVATE
./
../src
../third_party/tinyobjloader
../third_party/tinyrenderer 
../third_party/nlohmann/include 
../third_party 
)

target_compile_definitions(e2e_diff PRIVATE USE_CPPAD USE_CPPAD_CODEGEN)
# target_link_libraries(e2e_diff ..)
target_link_libraries(e2e_diff differentiation Eigen3::Eigen tinyxml2 ${CMAKE_DL_LIBS} crossguid cpp_base64 zmq ${CMAKE_DL_LIBS})



