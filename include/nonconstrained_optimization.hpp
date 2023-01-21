#ifndef OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_

#include <memory>

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

#include "gradient_descent_algorithm.hpp"
#include "newton_rapson_algorithm.hpp"

namespace optlib {
  template <unsigned int size, typename T, typename OptimizedFunctionClass>
  class NonconstrainedOptimizer {
    public:
      explicit NonconstrainedOptimizer(const optlib_io::OptimizationType optimization_type) {
        switch (optimization_type) {
          case optlib_io::OptimizationType::GradientDescent: {
            optimizer_ = std::make_shared<GradientDescentAlgorithm<size, T, OptimizedFunctionClass>>(optimization_type);
            break;
          }
          case optlib_io::OptimizationType::NewtonRapson: {
            optimizer_ = std::make_shared<NewtonRapsonAlgorithm<size, T, OptimizedFunctionClass>>(optimization_type);
            break;
          }
        }
      }

      ~NonconstrainedOptimizer(void) = default;

      optlib_io::OptimizationOutput<size, T> Run(const optlib_io::OptimizedFunction<size, T>::StateVector & initial_argument) {
        const auto output = optimizer_->Run(initial_argument);
        
        return output;
      }

    private:
      std::shared_ptr<BaseUnconstrainedOptimizer<size, T, OptimizedFunctionClass>> optimizer_ = nullptr;
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_NONCONSTRAINED_OPTIMIZATION_HPP_
