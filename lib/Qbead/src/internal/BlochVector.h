#ifndef BLOCH_VECTOR_H
#define BLOCH_VECTOR_H

#include <math.h>
#include "QbeadUtils.h"

namespace Qbead
{
    class BlochVector
    {
    public:
        BlochVector() : theta(0), phi(0), x(0), y(0), z(1) {}

        BlochVector(const float theta_in, const float phi_in)
        {
            const float theta_mod360 = modulo(theta_in, 360);
            theta = (theta_mod360 < 180) ? theta_mod360 : 360 - theta_mod360;
            phi = modulo(phi_in + (theta_mod360 > 180) * 180, 360);
            x = cos_deg(phi_in) * sin_deg(theta_in);
            y = sin_deg(phi_in) * sin_deg(theta_in);
            z = cos_deg(theta_in);
        }

        BlochVector(const float x_in, const float y_in, const float z_in)
        {
            theta = toDegrees(atan2(sqrt(x_in * x_in + y_in * y_in), z_in));
            phi = modulo(toDegrees(atan2(y_in, x_in)), 360);
            const float r = sqrt(x_in * x_in + y_in * y_in + z_in * z_in);
            x = x_in / r;
            y = y_in / r;
            z = z_in / r;
        }

        float theta; // in degrees
        float phi;   // in degrees
        float x;
        float y;
        float z;

        BlochVector &operator=(const BlochVector &other)
        {
            theta = other.theta;
            phi = other.phi;
            x = other.x;
            y = other.y;
            z = other.z;
            return *this;
        }

        BlochVector operator-() const
        {
            return BlochVector(-x, -y, -z);
        }

        BlochVector rotatedAround(const BlochVector &axis, const float angle) const
        {
            const float axis_x = axis.x;
            const float axis_y = axis.y;
            const float axis_z = axis.z;
            const float dot_product = x * axis_x + y * axis_y + z * axis_z;
            const float new_x = x * cos_deg(angle) + (axis_y * z - axis_z * y) * sin_deg(angle) + axis_x * dot_product * (1 - cos_deg(angle));
            const float new_y = y * cos_deg(angle) + (axis_z * x - axis_x * z) * sin_deg(angle) + axis_y * dot_product * (1 - cos_deg(angle));
            const float new_z = z * cos_deg(angle) + (axis_x * y - axis_y * x) * sin_deg(angle) + axis_z * dot_product * (1 - cos_deg(angle));

            BlochVector result(new_x, new_y, new_z);
            return result;
        }

        BlochVector &rotateAround(const BlochVector &axis, const float angle)
        {
            *this = this->rotatedAround(axis, angle);
            return *this;
        }

        float centralAngle(const BlochVector &other) const
        { // TODO this is quite suboptimal
            return toDegrees(acos(
                cos_deg(theta) * cos_deg(other.theta) + sin_deg(theta) * sin_deg(other.theta) * cos_deg(phi - other.phi)));
        }

        // Absolute value of inner product between the quantum states (as kets in a Hilbert space) represented by the two Bloch vectors
        float innerProductAbs(const BlochVector &other) const
        { // TODO this is quite suboptimal // maybe rename to innerProductKet
            float angle = centralAngle(other);
            float cosval = cos_deg(angle / 2.0f);
            return cosval;
        }

        // Inner product between the 3D vectors
        float innerProductGeom(const BlochVector &other) const
        { // TODO this is quite suboptimal
            return x * other.x + y * other.y + z * other.z;
        }

        void setXYZ(const float x, const float y, const float z)
        {
            BlochVector new_vector(x, y, z);
            *this = new_vector;
        }

        void setAngles(const float theta, const float phi)
        {
            BlochVector new_vector(theta, phi);
            *this = new_vector;
        }
    };

    float centralAngle(const BlochVector &v, const BlochVector &u)
    {
        return v.centralAngle(u);
    }

    float innerProductAbs(const BlochVector &v, const BlochVector &u)
    {
        return v.innerProductAbs(u);
    }

    float innerProductGeom(const BlochVector &v, const BlochVector &u)
    {
        return v.innerProductGeom(u);
    }
}
#endif