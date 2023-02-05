#include "gtest/gtest.h"

#include <cmath>

#include "nonconstrained_optimization.hpp"

/* Quadratic form */
class QuadraticForm : public optlib_io::OptimizedFunction<2, double> {
  public:
    QuadraticForm(void) {
      qf_ << 1.0, 0.0, 0.0, 1.0;
    }

    double FunctionValue(const optlib_io::StateVector<2, double> & argument) {
      const auto value = argument.transpose() * qf_ * argument;
      return value(0);
    }

    optlib_io::StateVector<2, double> FunctionGradient(const optlib_io::StateVector<2, double> & argument) {
      return static_cast<optlib_io::StateVector<2, double>>(qf_ * argument);
    }

    optlib_io::StateMatrix<2, double> FunctionHessian(const optlib_io::StateVector<2, double> & argument) {
      return qf_;
    }

  private:
    optlib_io::StateMatrix<2, double> qf_;
};

/* ROsenbrock function */
class Rosenbrock : public optlib_io::OptimizedFunction<2, double> {
  public:
    Rosenbrock(void) { }

    double FunctionValue(const optlib_io::StateVector<2, double> & argument) {
      const auto x = argument(0);
      const auto y = argument(1);

      return std::pow(a_ - x, 2) + b_ * std::pow(y - x, 2);
    }

    optlib_io::StateVector<2, double> FunctionGradient(const optlib_io::StateVector<2, double> & argument) {
      const auto x = argument(0);
      const auto y = argument(1);

      const auto grad_x = -2.0 * (a_ - x) - 2.0 * b_ * (y - x);
      const auto grad_y = 2.0 * b_ * (y - x);

      optlib_io::StateVector<2, double> grad;
      grad << grad_x , grad_y; 
      
      return grad;
    }

    optlib_io::StateMatrix<2, double> FunctionHessian(const optlib_io::StateVector<2, double> & argument) {
      const auto x = argument(0);
      const auto y = argument(1);

      const auto h_xx = 2.0 * (1.0 + b_);
      const auto h_xy = -2.0 * b_;
      const auto h_yx = -2.0 * b_;
      const auto h_yy = 2.0 * b_;

      optlib_io::StateMatrix<2, double> hessian;
      hessian << h_xx, h_xy, h_yx, h_yy;

      return hessian;
    }

  private:
    const double a_ = 1.0;
    const double b_ = 100.0;
};


/******************* Tests *******************/
class UnconstrainedOptimizerTest : public ::testing::TestWithParam<optlib_io::OptimizationType>
{
  protected:
    void SetUp(void) override
    {}

    QuadraticForm qf_;
};

TEST_P(UnconstrainedOptimizerTest, SimpleQuadraticTest)
{
  const auto optimizer_type = GetParam();

  optlib_io::StateVector<2, double> x0;
  x0 << -25.0, -34.0;

  optlib::NonconstrainedOptimizer<2, double, QuadraticForm> optimizer(optimizer_type);

  const auto output = optimizer.Run(x0);

  EXPECT_TRUE(output.optimized_function_value < qf_.FunctionValue(x0));
  EXPECT_TRUE(output.optimized_argument.norm() < 0.01);
}

TEST_P(UnconstrainedOptimizerTest, SimpleQuadraticTest2)
{
  const auto optimizer_type = GetParam();

  optlib_io::StateVector<2, double> x0;
  x0 << -34.0, -534.0;

  optlib::NonconstrainedOptimizer<2, double, QuadraticForm> optimizer(optimizer_type);

  const auto output = optimizer.Run(x0);

  EXPECT_TRUE(output.optimized_function_value < qf_.FunctionValue(x0));
  EXPECT_TRUE(output.optimized_argument.norm() < 0.01);
}

// TEST_P(UnconstrainedOptimizerTest, SimpleRosenbrockTest)
// {
//   const auto optimizer_type = GetParam();

//   optlib_io::OptimizedFunction<2, double>::StateVector x0;
//   x0 << 0.0, 0.0;

//   optlib_io::OptimizedFunction<2, double>::StateVector x_opt;
//   x_opt << 1.0, 1.0;

//   optlib::NonconstrainedOptimizer<2, double, Rosenbrock> optimizer(optimizer_type);

//   const auto output = optimizer.Run(x0);

//   EXPECT_TRUE(output.optimized_function_value < qf_.FunctionValue(x0));
//   EXPECT_TRUE((output.optimized_argument - x_opt).norm() < 0.01);
// }

// TEST_P(UnconstrainedOptimizerTest, SSimpleRosenbrockTest2)
// {
//   const auto optimizer_type = GetParam();

//   optlib_io::OptimizedFunction<2, double>::StateVector x0;
//   x0 << 2.0, 2.0;

//   optlib_io::OptimizedFunction<2, double>::StateVector x_opt;
//   x_opt << 1.0, 1.0;

//   optlib::NonconstrainedOptimizer<2, double, Rosenbrock> optimizer(optimizer_type);

//   const auto output = optimizer.Run(x0);

//   EXPECT_TRUE(output.optimized_function_value < qf_.FunctionValue(x0));
//   EXPECT_TRUE((output.optimized_argument - x_opt).norm() < 0.01);
// }


INSTANTIATE_TEST_CASE_P(
  UnconstrainedOptimizerTestsOptions,
  UnconstrainedOptimizerTest,
  ::testing::Values(
    optlib_io::OptimizationType::GradientDescent,
    optlib_io::OptimizationType::GaussSeidl,
    optlib_io::OptimizationType::NewtonRapson
  )
);
