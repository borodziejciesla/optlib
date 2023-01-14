#include "gtest/gtest.h"

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

    StateVector FunctionJacobian(const optlib_io::OptimizedFunction<2, double>::StateVector & argument) {
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

TEST_F(LineseatchTests, SimpleTest)
{
    Eigen::Vector<double, 2> initial_point;
    initial_point << 1.0, 1.0; 
    Eigen::Vector<double, 2> search_direction;
    search_direction << -1.0, -1.0;
    auto qf = std::make_shared<QuadraticForm>();

    const auto initial_value = qf->FunctionValue(initial_point);

    const auto [point, value] = optlib::Linesearch<2, double>(initial_point, search_direction, qf);

    EXPECT_TRUE(initial_value > value);
}
