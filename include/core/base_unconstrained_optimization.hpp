#ifndef OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_

#include "linesearch.hpp"

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

namespace optlib {
  template <unsigned int size, typename T = float>
  class BaseUnconstrainedOptimizer {
    public:
      explicit BaseUnconstrainedOptimizer(const optlib_io::OptimizationType optimization_type) {
        //
      }

      virtual ~BaseUnconstrainedOptimizer(void) {}

      optlib_io::OptimizationOutput<size, T> Run(const optlib_io::OptimizedFunction<size, T>::StateVector & initial_argument) {
        //
      }

    protected:
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_
