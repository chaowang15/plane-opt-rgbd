#include "qemquadrics.h"

QEMQuadrics::QEMQuadrics()
{
    reset();
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)
{
    Vector3d e1 = v2 - v1, e2 = v3 - v1;
    Vector3d e = e1.cross(e2);
    if (e.norm() > 1e-8)
    {  // Ensure the three vertices are non-colinear
        e1.normalize();
		Vector3d p = (v1 + v2 + v3) / 3;
        double d = e2.dot(e1);
        e2 -= (e2.dot(e1)) * e1;  // make e2 orthogonal to e1
        e2.normalize();
        A_ = Matrix3d::Identity() - e1 * e1.transpose() - e2 * e2.transpose();
        b_ = -A_ * p;
        c_ = p.dot(-b_);
    }
	else
		reset();
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1, const Vector3d& v2)
{
    Vector3d e1 = v2 - v1, p = (v1 + v2) / 2;
    e1.normalize();
    A_ = Matrix3d::Identity() - e1 * e1.transpose();
    b_ = -A_ * p;
    c_ = p.dot(-b_);
}

QEMQuadrics::QEMQuadrics(const Vector3d& v1)
{
    A_ = Matrix3d::Identity();
    b_ = -v1;
    c_ = v1.dot(v1);
}

bool QEMQuadrics::optimize(Vector3d& v, double& energy)
{
    if (fabs(A_.determinant()) < 1e-12)
	{
		// If A's determinant is near 0, then target vertex v would be infinity.
		// This may occur, for instance, when all the planes in Q are parallel.
		return false;
	}
    v = -A_.inverse() * b_;
    energy = b_.dot(v) + c_;
    return true;
}

//! This is used as some energy evaluation.
double QEMQuadrics::evaluate(const Vector3d& v) const
{
    return v.dot(A_ * v) + 2 * b_.dot(v) + c_;
}
