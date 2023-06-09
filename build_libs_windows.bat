@set ROOT=%cd%
@echo root= %ROOT%

git submodule update --init --recursive

pushd third_party\gflags
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%/third_party/gflags/build_cmake/local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug

popd
cd %ROOT%



pushd third_party\glog
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -Dgflags_DIR=%ROOT%/third_party/gflags/build_cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%\third_party\glog\build_cmake\local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  Release local_install\lib\Release
copy  Debug local_install\lib\Debug

popd
cd %ROOT%


pushd third_party\cppadcodegen
git apply ..\patches\CppADCodeGen.diff
popd
cd %ROOT%




pushd third_party\eigen3
mkdir build_cmake
cd build_cmake
cmake -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install   ..
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%

rem pushd third_party\bullet3
rem mkdir build_cmake
rem cd build_cmake

rem cmake -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DCMAKE_CXX_FLAGS="/MP" -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..

rem cmake  --build .  --target ALL_BUILD  --config Debug
rem cmake  --build .  --target INSTALL  --config Release
rem mkdir local_install\lib\Release
rem mkdir local_install\lib\Debug

rem copy  lib\Release local_install\lib\Release
rem copy  lib\Debug local_install\lib\Debug

rem popd
rem cd %ROOT%


rem pushd third_party\ceres-solver
rem mkdir build_cmake
rem cd build_cmake
rem cmake  -DCMAKE_CXX_FLAGS="/MP" -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TESTING=OFF -Dgflags_DIR=%ROOT%/third_party/gflags/build_cmake -Dglog_DIR=%ROOT%/third_party/glog/build_cmake -DEigen3_DIR=%ROOT%/third_party/eigen3/build_cmake  -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%/third_party/ceres-solver/build_cmake/local_install ..
rem cmake  --build .  --target ALL_BUILD  --config Debug
rem cmake  --build .  --target INSTALL  --config Release
rem mkdir local_install\lib\Release
rem mkdir local_install\lib\Debug

rem copy  lib\Release local_install\lib\Release
rem copy  lib\Debug local_install\lib\Debug
rem popd
rem cd %ROOT%

rem del third_party\ceres-solver\build_cmake\local_install\lib\*.lib
rem del third_party\bullet3\build_cmake\local_install\lib\*.lib
del third_party\glog\build_cmake\local_install\lib\*.lib
del third_party\gflags\build_cmake\local_install\lib\*.lib



pushd third_party\boost
call bootstrap.bat
b2 --with-serialization
popd
cd %ROOT%


pushd third_party\osqp
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="d" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%\third_party\osqp\build_cmake\local_install ..
cmake  --build .  --target INSTALL  --config Debug
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%

pushd third_party\qpoases
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="d" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%\third_party\qpoases\build_cmake\local_install ..
cmake  --build .  --target INSTALL  --config Debug
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%



mkdir build_cmake
cd build_cmake

cmake  -DCMAKE_CXX_FLAGS="/MP" -DUSE_CERES=OFF -DUSE_OSQP=ON -Dosqp_DIR=%ROOT%\third_party\osqp\build_cmake -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -Dgflags_DIR=%ROOT%\third_party\gflags\build_cmake -Dglog_DIR=%ROOT%\third_party\glog\build_cmake -DEigen3_DIR=%ROOT%\third_party\eigen3\build_cmake -DUSE_CPPAD=ON -DPAGMO_WITH_EIGEN3=ON -DBoost_DIR:PATH=%ROOT%/third_party/boost/stage/lib/cmake/Boost-1.75.0 -DTBB_VERSION=2021.1.0 ..

rem cmake  --build .  --target INSTALL  --config Release
rem start DIFF_PHYSICS.sln
