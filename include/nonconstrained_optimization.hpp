#ifndef OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

#include "base_unconstrained_optimization.hpp"

namespace optlib {
  template <unsigned int size, typename T = float>
  class NonconstrainedOptimizer {
    public:
      explicit NonconstrainedOptimizer(const optlib_io::OptimizationType optimization_type) {
        //
      }

      ~NonconstrainedOptimizer(void) = default;

      optlib_io::OptimizationOutput<size, T> Run(const optlib_io::OptimizedFunction<size, T>::StateVector & initial_argument) {
        auto f = optlib_io::OptimizationOutput<size, T>{};
        return f;
      }
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
