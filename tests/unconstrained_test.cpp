#include "gtest/gtest.h"

#include "nonconstrained_optimization.hpp"

class QuadraticForm : public optlib_io::OptimizedFunction<2, double> {
  public:
    QuadraticForm(void) {
      qf_ << 1.0, 0.0, 0.0, 1.0;
    }

    double FunctionValue(const StateVector & argument) {
      const auto value = argument.transpose() * qf_ * argument; 
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


class UnconstrainedQuadraticFormTests : public ::testing::Test
{
  protected:
    void SetUp(void) override
    {}
};

TEST_F(UnconstrainedQuadraticFormTests, GaussSeidlTest)
{
    EXPECT_TRUE(true);
}