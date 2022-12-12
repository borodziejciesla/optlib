#ifndef OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

namespace optlib {
  template <unsigned int size, typename T = float>
  class NonconstrainedOptimizer {
    public:
      explicit NonconstrainedOptimizer(const optlib_io::OptimizationType optimization_type) {
        //
      }

      ~NonconstrainedOptimizer(void) = default;

      optlib_io::OptimizationOutput Run(const optlib_io::OptimizedFunction::StateVector & initial_argument) {
        //
      }
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
