// ======================================================================================
// CSF.i — SWIG 接口文件（Python 绑定）
//
// 这个文件定义了如何把 C++ 的 CSF 类暴露给 Python：
//   1. 包含必要的 C++ 头文件
//   2. 注册 std::vector 模板（VecInt, VecFloat, VecDouble）
//   3. 应用 numpy 数组 typemap（让 Python 可以传入 numpy 数组）
//   4. 包含 CSF.h 定义要暴露的类
// ===========================================================================

%module CSF
%{
  #define SWIG_FILE_WITH_INIT
  #include <exception>
  #include "../src/CSF.h"
  #include "../src/Cloth.h"
%}

%include "std_string.i"
%include "std_vector.i"
%include "numpy.i"

%exception {
    try {
        $action
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "unknown C++ exception");
    }
}

%init %{
import_array();
%}

namespace std
{
    %template(VecInt) vector<int>;
    %template(VecFloat) vector<float>;
    %template(VecVecFloat) vector< vector<float> >;
    %template(VecDouble) vector<double>;
}

// 应用 numpy 数组 typemap：让 Python 可以传入 (double* points, int rows, int cols)
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double *points, int rows, int cols)};
%include "../src/CSF.h"
