#include "camodocal/gpl/EigenQuaternionParameterization.h"

#include <cmath>

namespace camodocal
{

bool
EigenQuaternionParameterization::Plus(const double* x,
                                      const double* delta,
                                      double* x_plus_delta) const
{
    const double norm_delta =
        sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    if (norm_delta > 0.0)
    {
        const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
        double q_delta[4];
        q_delta[0] = sin_delta_by_delta * delta[0];
        q_delta[1] = sin_delta_by_delta * delta[1];
        q_delta[2] = sin_delta_by_delta * delta[2];
        q_delta[3] = cos(norm_delta);
        EigenQuaternionProduct(q_delta, x, x_plus_delta);
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            x_plus_delta[i] = x[i];
        }
    }
    return true;
}

bool
EigenQuaternionParameterization::PlusJacobian(const double* x,
                                              double* jacobian) const
{
    jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
    jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
    jacobian[6] =  x[1]; jacobian[7] = -x[0]; jacobian[8] =  x[3];  // NOLINT
    jacobian[9] = -x[0]; jacobian[10]  = -x[1]; jacobian[11]  = -x[2];  // NOLINT
    return true;
}

bool
EigenQuaternionParameterization::Minus(const double* y,
                                       const double* x,
                                       double* y_minus_x) const
{
    // Plus left-multiplies x by [sin(|d|) d/|d|, cos(|d|)].
    // Therefore the relative quaternion is y * inverse(x).
    double inverse_x[4] = {-x[0], -x[1], -x[2], x[3]};
    double relative[4];
    EigenQuaternionProduct(y, inverse_x, relative);

    if (relative[3] < 0.0)
    {
        for (double& value : relative)
        {
            value = -value;
        }
    }

    const double vector_norm =
        sqrt(relative[0] * relative[0] + relative[1] * relative[1] +
             relative[2] * relative[2]);
    if (vector_norm > 0.0)
    {
        const double scale = atan2(vector_norm, relative[3]) / vector_norm;
        for (int i = 0; i < 3; ++i)
        {
            y_minus_x[i] = scale * relative[i];
        }
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            y_minus_x[i] = 0.0;
        }
    }
    return true;
}

bool
EigenQuaternionParameterization::MinusJacobian(const double* x,
                                               double* jacobian) const
{
    // PlusJacobian has orthonormal columns, so its transpose is a left inverse.
    double plus_jacobian[12];
    PlusJacobian(x, plus_jacobian);
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            jacobian[row * 4 + column] = plus_jacobian[column * 3 + row];
        }
    }
    return true;
}

}
