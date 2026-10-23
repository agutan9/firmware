/**
 * @file BlochVector.h
 * @brief Single-qubit state represented as a point on the Bloch sphere.
 *
 * Implementations live in BlochVector.cpp.
 */
#ifndef BLOCH_VECTOR_H
#define BLOCH_VECTOR_H

#include <math.h>
#include "QbeadUtils.h"

namespace Qbead
{
    /**
     * @brief A unit vector on the Bloch sphere, stored both as angles and as Cartesian components.
     *
     * Angles are in degrees: theta in [0, 180] from the +z axis (|0⟩), phi in [0, 360).
     * The Cartesian components (x, y, z) are kept consistent with the angles by every
     * constructor and setter.
     */
    class BlochVector
    {
    public:
        /** @brief Constructs the north-pole state |0⟩ (theta = 0, phi = 0, z = 1). */
        BlochVector();

        /**
         * @brief Constructs a vector from spherical angles in degrees.
         *
         * theta is wrapped into [0, 180]; if it lies in (180, 360) the vector is
         * reflected through the pole and phi is shifted by 180 degrees. phi is
         * wrapped into [0, 360).
         *
         * @param theta_in Polar angle in degrees (any value).
         * @param phi_in   Azimuthal angle in degrees (any value).
         */
        BlochVector(const float theta_in, const float phi_in);

        /**
         * @brief Constructs a vector from Cartesian components, normalising to unit length.
         * @param x_in x component.
         * @param y_in y component.
         * @param z_in z component.
         * @warning A zero vector gives NaN components; no guard is applied.
         */
        BlochVector(const float x_in, const float y_in, const float z_in);

        float theta; /**< Polar angle in degrees, [0, 180]. */
        float phi;   /**< Azimuthal angle in degrees, [0, 360). */
        float x;     /**< Unit-vector x component. */
        float y;     /**< Unit-vector y component. */
        float z;     /**< Unit-vector z component. */

        /**
         * @brief Copies all angle and Cartesian fields from @p other.
         * @param other Vector to copy.
         * @return Reference to this vector.
         */
        BlochVector &operator=(const BlochVector &other);

        /**
         * @brief Antipodal (orthogonal) state.
         * @return The vector (-x, -y, -z).
         */
        BlochVector operator-() const;

        /**
         * @brief Returns a copy rotated about an axis (Rodrigues' rotation formula).
         * @param axis  Rotation axis; must be a unit vector.
         * @param angle Rotation angle in degrees (right-hand rule).
         * @return The rotated vector; this vector is unchanged.
         */
        BlochVector rotatedAround(const BlochVector &axis, const float angle) const;

        /**
         * @brief Rotates this vector in place about an axis.
         * @param axis  Rotation axis; must be a unit vector.
         * @param angle Rotation angle in degrees (right-hand rule).
         * @return Reference to this vector.
         */
        BlochVector &rotateAround(const BlochVector &axis, const float angle);

        /**
         * @brief Great-circle angle between this vector and @p other.
         * @param other Other vector.
         * @return Central angle in degrees, [0, 180].
         * @todo This is quite suboptimal.
         */
        float centralAngle(const BlochVector &other) const;

        /**
         * @brief Absolute value of the inner product between the quantum states (as kets in a Hilbert space) represented by the two Bloch vectors.
         *
         * Equal to cos(alpha / 2), where alpha is the central angle between the vectors.
         *
         * @param other Other state.
         * @return |⟨this|other⟩|, in [0, 1].
         * @todo This is quite suboptimal; maybe rename to innerProductKet.
         */
        float innerProductAbs(const BlochVector &other) const;

        /**
         * @brief Inner product between the 3D vectors.
         * @param other Other vector.
         * @return Geometric dot product x·x' + y·y' + z·z', in [-1, 1].
         * @todo This is quite suboptimal.
         */
        float innerProductGeom(const BlochVector &other) const;

        /**
         * @brief Replaces this vector with one built from Cartesian components (normalised).
         * @param x x component.
         * @param y y component.
         * @param z z component.
         */
        void setXYZ(const float x, const float y, const float z);

        /**
         * @brief Replaces this vector with one built from spherical angles in degrees.
         * @param theta Polar angle in degrees.
         * @param phi   Azimuthal angle in degrees.
         */
        void setAngles(const float theta, const float phi);
    };

    /**
     * @brief Great-circle angle between two vectors.
     * @param v First vector.
     * @param u Second vector.
     * @return Central angle in degrees, [0, 180].
     * @see BlochVector::centralAngle
     */
    float centralAngle(const BlochVector &v, const BlochVector &u);

    /**
     * @brief Absolute ket inner product between two states.
     * @param v First state.
     * @param u Second state.
     * @return |⟨v|u⟩|, in [0, 1].
     * @see BlochVector::innerProductAbs
     */
    float innerProductAbs(const BlochVector &v, const BlochVector &u);

    /**
     * @brief Geometric dot product between two Bloch vectors.
     * @param v First vector.
     * @param u Second vector.
     * @return v · u, in [-1, 1].
     * @see BlochVector::innerProductGeom
     */
    float innerProductGeom(const BlochVector &v, const BlochVector &u);
}
#endif // BLOCH_VECTOR_H
