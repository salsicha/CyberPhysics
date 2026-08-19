#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> y_position(y);
    Eigen::Map<const Eigen::Vector3d> x_position(x);
    Eigen::Map<Eigen::Vector3d> position_delta(y_minus_x);
    position_delta = y_position - x_position;

    Eigen::Map<const Eigen::Quaterniond> y_orientation(y + 3);
    Eigen::Map<const Eigen::Quaterniond> x_orientation(x + 3);
    Eigen::Quaterniond orientation_delta = x_orientation.conjugate() * y_orientation;
    if (orientation_delta.w() < 0.0)
    {
        orientation_delta.coeffs() *= -1.0;
    }

    Eigen::Map<Eigen::Vector3d> rotation_delta(y_minus_x + 3);
    if (std::abs(orientation_delta.w()) > 1e-12)
    {
        rotation_delta = 2.0 * orientation_delta.vec() / orientation_delta.w();
    }
    else
    {
        const Eigen::AngleAxisd angle_axis(orientation_delta);
        rotation_delta = angle_axis.angle() * angle_axis.axis();
    }
    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.leftCols<6>().setIdentity();
    return true;
}
