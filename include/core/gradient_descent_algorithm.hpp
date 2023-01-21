#ifndef OPTLIB_INCLUDE_CORE_GRADIENT_DESCENT_ALGORITHM_HPP_
#define OPTLIB_INCLUDE_CORE_GRADIENT_DESCENT_ALGORITHM_HPP_

#include <memory>
#include <tuple>

#include "base_unconstrained_optimization.hpp"
#include "linesearch.hpp"

namespace optlib {
  template <unsigned int size, typename T, typename OptimizedFunctionClass>
  class GradientDescentAlgorithm : public BaseUnconstrainedOptimizer<size, T, OptimizedFunctionClass> {
    public:
      explicit GradientDescentAlgorithm(const optlib_io::OptimizationType optimization_type)
        : BaseUnconstrainedOptimizer<size, T, OptimizedFunctionClass>(optimization_type)
        , optimized_function2_{std::make_shared<OptimizedFunctionClass>()} {}

      virtual ~GradientDescentAlgorithm(void) {}

    protected:
      optlib_io::OptimizedFunction<size, T>::StateVector FindMinimzationDirection(const optlib_io::OptimizedFunction<size, T>::StateVector & start_point) {
        const auto search_direction = -optimized_function2_->FunctionGradient(start_point);
        return search_direction;
      }

      optlib_io::OptimizedFunction<size, T>::StateVector FindMinimumOnSearchDirection(const optlib_io::OptimizedFunction<size, T>::StateVector & start_point, const optlib_io::OptimizedFunction<size, T>::StateVector & search_direction) {
        const auto [minimum_on_direction, value] = optlib::Linesearch<size, T>(start_point, search_direction, optimized_function2_);
        std::ignore = value;

        return minimum_on_direction;
      }

      std::shared_ptr<OptimizedFunctionClass> optimized_function2_ = nullptr;
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CORE_GRADIENT_DESCENT_ALGORITHM_HPP_
