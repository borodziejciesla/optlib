#include "gtest/gtest.h"

#include <cmath>
#include <memory>

#include "linesearch.hpp"

class QuadraticForm : public optlib_io::OptimizedFunction<2, double> {
  public:
    QuadraticForm(void) {
      qf_ << 1.0, 0.0, 0.0, 1.0;
    }

    double FunctionValue(const StateVector & argument) {
      const auto value = argument.transpose() * qf_ * argument;
      return value(0);
    }

    StateVector FunctionGradient(const optlib_io::OptimizedFunction<2, double>::StateVector & argument) {
      return argument;
    }

    optlib_io::OptimizedFunction<2, double>::StateMatrix FunctionHessian(const optlib_io::OptimizedFunction<2, double>::StateVector & argument) {
      return qf_;
    }

  private:
    optlib_io::OptimizedFunction<2, double>::StateMatrix qf_;;
};

class SquareSquare : public optlib_io::OptimizedFunction<2, double> {
  public:
    SquareSquare(void) {
      qf_ << 1.0, 0.0, 0.0, 1.0;
    }

    double FunctionValue(const StateVector & argument) {
      const auto value = std::pow(argument(0), 4) + std::pow(argument(1), 4);
      return value;
    }

    StateVector FunctionGradient(const optlib_io::OptimizedFunction<2, double>::StateVector & argument) {
      return argument;
    }

    optlib_io::OptimizedFunction<2, double>::StateMatrix FunctionHessian(const optlib_io::OptimizedFunction<2, double>::StateVector & argument) {
      return qf_;
    }

  private:
    optlib_io::OptimizedFunction<2, double>::StateMatrix qf_;;
};

/* Test */
class LineseatchTests : public ::testing::Test
{
  protected:
    void SetUp(void) override
    {}
};

TEST_F(LineseatchTests, SimpleExpansion1Test)
{
    Eigen::Vector<double, 2> initial_point;
    initial_point << 1.0, 1.0; 
    Eigen::Vector<double, 2> search_direction;
    search_direction << -0.5, -0.5;
    auto qf = std::make_shared<QuadraticForm>();

    const auto initial_value = qf->FunctionValue(initial_point);

    const auto [point, value] = optlib::Linesearch<2, double>(initial_point, search_direction, qf);

    EXPECT_TRUE(initial_value > value);
}

TEST_F(LineseatchTests, SimpleExpansion2Test)
{
    Eigen::Vector<double, 2> initial_point;
    initial_point << 100.0, 100.0; 
    Eigen::Vector<double, 2> search_direction;
    search_direction << -0.5, -0.5;
    auto qf = std::make_shared<QuadraticForm>();

    const auto initial_value = qf->FunctionValue(initial_point);

    const auto [point, value] = optlib::Linesearch<2, double>(initial_point, search_direction, qf);

    EXPECT_TRUE(initial_value > value);
}

TEST_F(LineseatchTests, SimpleExpansion3Test)
{
    Eigen::Vector<double, 2> initial_point;
    initial_point << 100.0, 100.0; 
    Eigen::Vector<double, 2> search_direction;
    search_direction << -0.005, -0.005;
    auto qf = std::make_shared<QuadraticForm>();

    const auto initial_value = qf->FunctionValue(initial_point);

    const auto [point, value] = optlib::Linesearch<2, double>(initial_point, search_direction, qf);

    EXPECT_TRUE(initial_value > value);
}

TEST_F(LineseatchTests, SquareSquareExpansion1Test)
{
    Eigen::Vector<double, 2> initial_point;
    initial_point << 1.0, 1.0; 
    Eigen::Vector<double, 2> search_direction;
    search_direction << -0.5, -0.5;
    auto ss = std::make_shared<SquareSquare>();

    const auto initial_value = ss->FunctionValue(initial_point);

    const auto [point, value] = optlib::Linesearch<2, double>(initial_point, search_direction, ss);

    EXPECT_TRUE(initial_value > value);
}
