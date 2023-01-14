#ifndef OPTLIB_INCLUDE_CORE_BASE_LINESEARCH_HPP_
#define OPTLIB_INCLUDE_CORE_BASE_LINESEARCH_HPP_

#include "optimized_function.hpp"

#include <array>
#include <cmath>
#include <memory>

namespace optlib {
  template <typename T = float>
  struct PointWithValue {
    T alpha = T(0);
    T value = T(0);
  };

  template <unsigned int size, typename T = float>
  using OptimizedFunctionPtr = std::shared_ptr<optlib_io::OptimizedFunction<size, T>>;

  constexpr auto maximum_iterations_number = 10u;

  /*****************************************************/
  /*                  Local functions                  */
  /*****************************************************/
  template <unsigned int size, typename T = float>
  static void Expansion(const Eigen::Vector<T, size> & initial_point, const Eigen::Vector<T, size> & search_direction, const OptimizedFunctionPtr<size, T> cost_function, std::array<PointWithValue<T>, 3u> & points, T multiplier) {
    // Check
    auto check_condition = [](const std::array<PointWithValue<T>, 3u> & points) {
      return (points.at(0).value > points.at(1).value) && (points.at(1).value > points.at(2).value);
    };
    
    // Iteration
    points.at(2)  = {multiplier, cost_function->FunctionValue(initial_point + search_direction * multiplier)};

    auto counter = 0u;
    while((counter < maximum_iterations_number) && check_condition(points)) {
      multiplier *= multiplier;
      counter++;

      points.at(0) = points.at(1);
      points.at(1) = points.at(2);
      points.at(2)  = {multiplier, cost_function->FunctionValue(initial_point + search_direction * multiplier)};
    };
  };

  template <unsigned int size, typename T = float>
  static void Contraction(const Eigen::Vector<T, size> & initial_point, const Eigen::Vector<T, size> & search_direction, const OptimizedFunctionPtr<size, T> cost_function, std::array<PointWithValue<T>, 3u> & points, T multiplier) {
    constexpr auto epsilon = 0.01;

    auto counter = 0u;
    while((counter < maximum_iterations_number)) {
      if ((points.at(1).alpha - points.at(0).alpha) < epsilon) {
        points.at(2) = points.at(1);
        points.at(1)  = {multiplier, cost_function->FunctionValue(initial_point + search_direction * multiplier)};

        if (points.at(1).value < points.at(0).value) {
          points.at(1)  = {multiplier, cost_function->FunctionValue(initial_point + search_direction * multiplier)};
          break;
        }
      } else {
        break;
      }
    }
  };

  template <typename T = float>
  static T EstimateLineseaerchMinimum(std::array<PointWithValue<T>, 3u> & points) {
    const auto q1 = points.at(0).value;
    const auto z1 = points.at(0).alpha;
    const auto q2 = points.at(1).value;
    const auto z2 = points.at(1).alpha;
    const auto q3 = points.at(2).value;
    const auto z3 = points.at(2).alpha;

    const auto z1_sqr = static_cast<T>(std::pow(z1, 2));
    const auto z2_sqr = static_cast<T>(std::pow(z2, 2));
    const auto z3_sqr = static_cast<T>(std::pow(z3, 2));

    const auto numerator = q1 * (z2_sqr - z3_sqr) + q2 * (z3_sqr - z1_sqr) + q3 * (z1_sqr - z2_sqr);
    const auto denumerator = q1 * (z2 - z3) + q2 * (z3 - z1) + q3 * (z1 - z2);

    const auto estimated_optimum = T(0.5) * numerator / denumerator;

    return estimated_optimum;
  }

  /*****************************************************/
  /*                 Exported function                 */
  /*****************************************************/
  template <unsigned int size, typename T = float>
  extern std::pair<Eigen::Vector<T, size>, T> Linesearch(const Eigen::Vector<T, size> & initial_point, const Eigen::Vector<T, size> & search_direction, const OptimizedFunctionPtr<size, T> cost_function) {
    const auto step_length = T(1.0);
    const auto alpha = T(2.0);
    std::array<PointWithValue<T>, 3u> points;

    // First step
    const auto current_value = cost_function->FunctionValue(initial_point);
    const auto first_step_value = cost_function->FunctionValue(initial_point + search_direction);

    // Decide if make Expansion or Contraction
    if (current_value >= first_step_value) {
      Expansion<size, T>(initial_point, search_direction, cost_function, points, alpha);
    } else {
      Contraction<size, T>(initial_point, search_direction, cost_function, points, T(1.0) / alpha);
    }

    // Estimation
    const auto  estimated_optimum_step = EstimateLineseaerchMinimum<T>(points);

    // Check if really best point
    const auto estimated_step_value = cost_function->FunctionValue(initial_point + search_direction * estimated_optimum_step);

    if (points.at(0).value < estimated_step_value)
      return std::make_pair((initial_point + search_direction * points.at(0).alpha), points.at(0).value);
    else
      return std::make_pair((initial_point + search_direction * estimated_step_value), estimated_step_value);
  };
} //  namespace optlib

#endif  //  OPTLIB_INCLUDE_CORE_BASE_LINESEARCH_HPP_
