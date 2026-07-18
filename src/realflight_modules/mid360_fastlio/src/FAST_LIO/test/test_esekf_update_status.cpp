#include <omp.h>

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

namespace
{

using Vector12 = MTK::vect<12, double>;
using Vector1 = MTK::vect<1, double>;

MTK_BUILD_MANIFOLD(TestState, ((Vector12, value)));
MTK_BUILD_MANIFOLD(TestInput, ((Vector1, value)));

using Filter = esekfom::esekf<TestState, 1, TestInput>;

Eigen::Matrix<double, TestState::DIM, 1> processModel(TestState &, const TestInput &)
{
    return Eigen::Matrix<double, TestState::DIM, 1>::Zero();
}

Eigen::Matrix<double, TestState::DIM, TestState::DOF> processStateJacobian(
    TestState &, const TestInput &)
{
    return Eigen::Matrix<double, TestState::DIM, TestState::DOF>::Zero();
}

Eigen::Matrix<double, TestState::DIM, 1> processNoiseJacobian(TestState &, const TestInput &)
{
    return Eigen::Matrix<double, TestState::DIM, 1>::Zero();
}

int invalidCallbackCount = 0;

void invalidMeasurement(TestState &, esekfom::dyn_share_datastruct<double> &measurement)
{
    ++invalidCallbackCount;
    measurement.valid = false;
}

void validZeroResidualMeasurement(
    TestState &, esekfom::dyn_share_datastruct<double> &measurement)
{
    measurement.valid = true;
    measurement.h_x = Eigen::MatrixXd::Zero(1, 12);
    measurement.h_x(0, 0) = 1.0;
    measurement.h = Eigen::VectorXd::Zero(1);
}

int partiallyValidCallbackCount = 0;
double stateValueSeenAfterPartialUpdate = 0.0;

void validThenInvalidMeasurement(
    TestState &state, esekfom::dyn_share_datastruct<double> &measurement)
{
    if (partiallyValidCallbackCount++ == 0)
    {
        measurement.valid = true;
        measurement.h_x = Eigen::MatrixXd::Zero(1, 12);
        measurement.h_x(0, 0) = 1.0;
        measurement.h = Eigen::VectorXd::Constant(1, 0.5);
        return;
    }

    stateValueSeenAfterPartialUpdate = state.value[0];
    measurement.valid = false;
}

Filter makeFilter(Filter::measurementModel_dyn_share *measurementModel)
{
    Filter filter;
    double limits[TestState::DOF];
    std::fill(limits, limits + TestState::DOF, 1.0e-6);
    filter.init_dyn_share(
        processModel,
        processStateJacobian,
        processNoiseJacobian,
        measurementModel,
        2,
        limits);
    return filter;
}

void expectStateAndCovarianceUnchanged(
    const Filter &filter,
    const TestState &stateBefore,
    const Filter::cov &covarianceBefore)
{
    Filter::vectorized_state delta;
    filter.get_x().boxminus(delta, stateBefore);
    EXPECT_TRUE(delta.isZero(1.0e-12));
    EXPECT_TRUE(filter.get_P().isApprox(covarianceBefore, 1.0e-12));
}

TEST(EsekfUpdateStatus, RejectsMeasurementWhenEveryIterationIsInvalid)
{
    invalidCallbackCount = 0;
    Filter filter = makeFilter(invalidMeasurement);
    const TestState stateBefore = filter.get_x();
    const Filter::cov covarianceBefore = filter.get_P();
    double solveTime = 0.0;

    EXPECT_FALSE(filter.update_iterated_dyn_share_modified(1.0, solveTime));
    EXPECT_GT(invalidCallbackCount, 0);
    expectStateAndCovarianceUnchanged(filter, stateBefore, covarianceBefore);
}

TEST(EsekfUpdateStatus, ReportsACommittedMeasurementUpdate)
{
    Filter filter = makeFilter(validZeroResidualMeasurement);
    const Filter::cov covarianceBefore = filter.get_P();
    double solveTime = 0.0;

    EXPECT_TRUE(filter.update_iterated_dyn_share_modified(1.0, solveTime));
    EXPECT_FALSE(filter.get_P().isApprox(covarianceBefore, 1.0e-12));
}

TEST(EsekfUpdateStatus, RollsBackAnUpdateThatCannotBeCommitted)
{
    partiallyValidCallbackCount = 0;
    stateValueSeenAfterPartialUpdate = 0.0;
    Filter filter = makeFilter(validThenInvalidMeasurement);
    const TestState stateBefore = filter.get_x();
    const Filter::cov covarianceBefore = filter.get_P();
    double solveTime = 0.0;

    EXPECT_FALSE(filter.update_iterated_dyn_share_modified(1.0, solveTime));
    EXPECT_GE(partiallyValidCallbackCount, 2);
    EXPECT_GT(std::abs(stateValueSeenAfterPartialUpdate), 1.0e-6);
    expectStateAndCovarianceUnchanged(filter, stateBefore, covarianceBefore);
}

}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
