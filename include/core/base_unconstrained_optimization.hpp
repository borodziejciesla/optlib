#ifndef OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_
#define OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_

#include <memory>

#include "linesearch.hpp"

#include "optimized_function.hpp"
#include "optimization_output.hpp"
#include "optimization_type.hpp"

namespace optlib {
  template <unsigned int size, typename T, typename OptimizedFunctionClass>
  class BaseUnconstrainedOptimizer {
    public:
      explicit BaseUnconstrainedOptimizer(const optlib_io::OptimizationType optimization_type)
        : optimized_function_{std::make_shared<OptimizedFunctionClass>()} {}

      virtual ~BaseUnconstrainedOptimizer(void) {}

      optlib_io::OptimizationOutput<size, T> Run(const optlib_io::OptimizedFunction<size, T>::StateVector & initial_argument) {
        auto iteration_start_point = initial_argument;
        auto best_value = optimized_function_->FunctionValue(initial_argument);

        // Start iterations
        do {
          const auto best_on_direction = Iteration(iteration_start_point);
          iteration_start_point = best_on_direction;
          current_best_point_ = iteration_start_point;
          best_value = optimized_function_->FunctionValue(iteration_start_point);
        } while (IsStopCondition());

        // Set output
        output_.optimized_argument = iteration_start_point;
        output_.optimized_function_value = best_value;

        return output_;
      }

    protected:
      virtual optlib_io::OptimizedFunction<size, T>::StateVector FindMinimzationDirection(const optlib_io::OptimizedFunction<size, T>::StateVector & start_point) = 0;
      virtual optlib_io::OptimizedFunction<size, T>::StateVector FindMinimumOnSearchDirection(const optlib_io::OptimizedFunction<size, T>::StateVector & start_point, const optlib_io::OptimizedFunction<size, T>::StateVector & search_direction) = 0;

      std::shared_ptr<OptimizedFunctionClass> optimized_function_ = nullptr;

    private:
      optlib_io::OptimizedFunction<size, T>::StateVector Iteration(const optlib_io::OptimizedFunction<size, T>::StateVector & start_point) {
        // Increase iteration counter
        iteration_index_++;

        // Find search direction and minimum
        const auto search_direction = FindMinimzationDirection(start_point);
        const auto minimum_on_direction = FindMinimumOnSearchDirection(start_point, search_direction);

        return minimum_on_direction;
      }

      bool IsStopCondition(void) const {
        return (iteration_index_ < 100u) 
          && (optimized_function_->FunctionGradient(current_best_point_).norm() > 0.0001);
      }

      size_t iteration_index_ = 0u;
      optlib_io::OptimizedFunction<size, T>::StateVector current_best_point_;
      optlib_io::OptimizationOutput<size, T> output_;
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CORE_BASE_UNCONSTRAINED_OPTIMIZATION_HPP_
