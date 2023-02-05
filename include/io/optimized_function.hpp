#ifndef OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_
#define OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_

#include <Eigen/Dense>

namespace optlib_io {
  template <unsigned int size, typename T = float>
  using StateVector = Eigen::Vector<T, size>;

  template <unsigned int size, typename T = float>
  using StateMatrix = Eigen::Matrix<T, size, size>;

  template <unsigned int size, typename T = float>
  class OptimizedFunction {
    public:
      OptimizedFunction(void) = default;
      virtual ~OptimizedFunction(void) {}

      virtual T FunctionValue(const StateVector<size, T> & argument) = 0;
      virtual StateVector<size, T> FunctionGradient(const StateVector<size, T> & argument) = 0;
      virtual StateMatrix<size, T> FunctionHessian(const StateVector<size, T> & argument) = 0;
  };
} //  namespace optlib_io

#endif  //  OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_
