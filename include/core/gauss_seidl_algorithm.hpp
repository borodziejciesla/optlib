#ifndef OPTLIB_INCLUDE_CORE_GAUSS_SEIDL_ALGORITHM_HPP_
#define OPTLIB_INCLUDE_CORE_GAUSS_SEIDL_ALGORITHM_HPP_

#include <memory>
#include <tuple>

#include "base_unconstrained_optimization.hpp"
#include "linesearch.hpp"

namespace optlib {
  template <unsigned int size, typename T, typename OptimizedFunctionClass>
  class GaussSeidlAlgorithm : public BaseUnconstrainedOptimizer<size, T, OptimizedFunctionClass> {
    public:
      explicit GaussSeidlAlgorithm(const optlib_io::OptimizationType optimization_type)
        : BaseUnconstrainedOptimizer<size, T, OptimizedFunctionClass>(optimization_type)
        , optimized_function2_{std::make_shared<OptimizedFunctionClass>()} {}

      virtual ~GaussSeidlAlgorithm(void) {}

    protected:
      optlib_io::StateVector<size, T> FindMinimzationDirection(const optlib_io::StateVector<size, T> & start_point) {
        std::ignore = start_point;

        optlib_io::StateVector<size, T> search_direction = optlib_io::StateVector<size, T>::Zero();
        search_direction(n) = T(1);

        n = (n + 1u) % size;
        
        return search_direction;
      }

      optlib_io::StateVector<size, T> FindMinimumOnSearchDirection(const optlib_io::StateVector<size, T> & start_point, const optlib_io::StateVector<size, T> & search_direction) {
        const auto [minimum_on_direction, value] = optlib::Linesearch<size, T>(start_point, search_direction, optimized_function2_);
        std::ignore = value;

        return minimum_on_direction;
      }

      std::shared_ptr<OptimizedFunctionClass> optimized_function2_ = nullptr;
      unsigned int n = 0u;
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CORE_GAUSS_SEIDL_ALGORITHM_HPP_
