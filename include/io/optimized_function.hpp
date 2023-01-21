#ifndef OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_
#define OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_

#include <Eigen/Dense>

namespace optlib_io {
  template <unsigned int size, typename T = float>
  class OptimizedFunction {
    public:
      OptimizedFunction(void) = default;
      virtual ~OptimizedFunction(void) {}

      using StateVector = Eigen::Vector<T, size>;
      using StateMatrix = Eigen::Matrix<T, size, size>;

      virtual T FunctionValue(const StateVector & argument) = 0;
      virtual StateVector FunctionGradient(const StateVector & argument) = 0;
      virtual StateMatrix FunctionHessian(const StateVector & argument) = 0;
  };
} //  namespace optlib_io

#endif  //  OPTLIB_INCLUDE_IO_OPTIMIZED_FUNCTION_HPP_
