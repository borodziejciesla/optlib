#ifndef OPTLIB_INCLUDE_IO_OPTIMIZATION_OUTPUT_HPP_
#define OPTLIB_INCLUDE_IO_OPTIMIZATION_OUTPUT_HPP_

#include <Eigen/Dense>

namespace optlib_io {
  template <unsigned int size, typename T = float>
  struct OptimizationOutput {
    Eigen::Vector<T, size> optimized_argument;
    T optimized_function_value = T(0);
  };
} //  namespace optlib_io

#endif  //  OPTLIB_INCLUDE_IO_OPTIMIZATION_OUTPUT_HPP_
