add_executable(sample_ad sample_ad.cpp  ${TDS_HDRS})
target_link_libraries(sample_ad differentiation Eigen3::Eigen ${CMAKE_DL_LIBS})
target_include_directories(sample_ad PRIVATE ../src ./)# ../build_cmake/CppADCodeGen/include)
target_compile_definitions(sample_ad PRIVATE USE_CPPAD USE_TINY USE_CPPAD_CODEGEN)


add_executable(my_diffrew_sim my_diffrew_sim.cpp  ${TDS_HDRS})
target_link_libraries(my_diffrew_sim differentiation Eigen3::Eigen ${CMAKE_DL_LIBS})
target_include_directories(my_diffrew_sim PRIVATE ../src ./)# ../build_cmake/CppADCodeGen/include)
target_compile_definitions(my_diffrew_sim PRIVATE USE_CPPAD USE_TINY USE_CPPAD_CODEGEN)


add_executable(my_diffrew_ant my_diffrew_ant.cpp  
../third_party/tinyrenderer/tinyrenderer.cpp 
../third_party/tinyrenderer/geometry.cpp 
../third_party/tinyrenderer/our_gl.cpp 
../third_party/tinyrenderer/model.cpp
../third_party/tinyrenderer/tgaimage.cpp
../third_party/stb_image/stb_image.cpp
../third_party/tinyobjloader/tiny_obj_loader.cc
${TDS_HDRS})
target_include_directories(my_diffrew_ant PRIVATE 
    ../src 
    ../third_party/tinyobjloader
    ../third_party/tinyrenderer 
    ../third_party/nlohmann/include 
    ../third_party 
)
target_link_libraries(my_diffrew_ant differentiation Eigen3::Eigen tinyxml2 ${CMAKE_DL_LIBS} crossguid cpp_base64 zmq )
target_compile_definitions(my_diffrew_ant PRIVATE USE_CPPAD USE_TINY USE_CPPAD_CODEGEN)


# add_executable(my_diffrew_vecenv my_diffrew_vecenv.cpp  ${TDS_HDRS})
# target_link_libraries(my_diffrew_vecenv differentiation Eigen3::Eigen ${CMAKE_DL_LIBS})
# target_include_directories(my_diffrew_vecenv PRIVATE ../src)# ../build_cmake/CppADCodeGen/include)
# target_compile_definitions(my_diffrew_vecenv PRIVATE USE_CPPAD USE_TINY USE_CPPAD_CODEGEN)




