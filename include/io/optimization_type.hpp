#ifndef OPTLIB_INCLUDE_IO_OPTIMIZATION_TYPE_HPP_
#define OPTLIB_INCLUDE_IO_OPTIMIZATION_TYPE_HPP_

#include <array>

#include <Eigen/Dense>

namespace optlib_io {
  enum class OptimizationType {
    GradientDescent = 0,
    GaussSeidl = 1
  };
} //  namespace optlib_io

#endif  //  OPTLIB_INCLUDE_IO_OPTIMIZATION_TYPE_HPP_
