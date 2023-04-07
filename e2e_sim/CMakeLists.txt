find_package(Enzyme REQUIRED)

message("LLVM Version: " ${Enzyme_LLVM_VERSION_MAJOR})
message("Found LLVM at: " ${Enzyme_LLVM_BINARY_DIR})

# C and C++ Compilers can only be configured before project(...)
# set(CMAKE_C_COMPILER "${Enzyme_LLVM_BINARY_DIR}/bin/clang")
# set(CMAKE_CXX_COMPILER "${Enzyme_LLVM_BINARY_DIR}/bin/clang")

# project(EnzymeExample)

find_package(Enzyme REQUIRED COMPONENTS ClangEnzyme)

get_target_property(ClangEnzyme ClangEnzyme-${Enzyme_LLVM_VERSION_MAJOR} LOCATION)

message("Found Enzyme at: " ${ClangEnzyme})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=lld -flto")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fuse-ld=lld -flto")
set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=lld -flto -Wl,-mllvm=-load=${ClangEnzyme}")
set(CMAKE_SHARED_LINKER_FLAGS "-fuse-ld=lld -flto -Wl,-mllvm=-load=${ClangEnzyme}")

add_executable(e2e_diff 
e2e_diff.cpp 
${TDS_HDRS})
target_include_directories(e2e_diff PRIVATE
./
../src
../examples
../third_party/tinyobjloader
../third_party/tinyrenderer 
../third_party/nlohmann/include 
../third_party 
)

target_compile_definitions(e2e_diff PRIVATE USE_CPPAD USE_CPPAD_CODEGEN)
# target_link_libraries(e2e_diff ..)
target_link_libraries(e2e_diff differentiation Eigen3::Eigen tinyxml2 ${CMAKE_DL_LIBS} crossguid cpp_base64 opengl_window zmq ${CMAKE_DL_LIBS})

# add_executable(e2e_sim_laikago 
# e2e_sim_laikago.cpp 
# ${TDS_HDRS})
# target_include_directories(e2e_sim_laikago PRIVATE
# ./
# ../src
# ../third_party/tinyobjloader
# ../third_party/tinyrenderer 
# ../third_party/nlohmann/include 
# ../third_party 
# )

# target_compile_definitions(e2e_sim_laikago PRIVATE USE_CPPAD USE_CPPAD_CODEGEN)
# # target_link_libraries(e2e_diff ..)
# target_link_libraries(e2e_sim_laikago differentiation Eigen3::Eigen tinyxml2 ${CMAKE_DL_LIBS} crossguid cpp_base64 zmq ${CMAKE_DL_LIBS})




