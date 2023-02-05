#ifndef OPTLIB_INCLUDE_CONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_CONSTRAINED_OPTIMIZATION_HPP_

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

namespace optlib {
  template <unsigned int size, typename T = float>
  class ConstrainedOptimizer {
    public:
      explicit ConstrainedOptimizer(const optlib_io::OptimizationType optimization_type) {
        //
      }

      ~ConstrainedOptimizer(void) = default;

      optlib_io::OptimizationOutput Run(const optlib_io::StateVector<T, size> & initial_argument) {
        //
      }
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CONSTRAINED_OPTIMIZATION_HPP_
