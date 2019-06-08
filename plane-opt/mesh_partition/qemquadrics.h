/*!
    This is an implementation of 1D, 2D and 3D QEM-based mesh simplification.

    Reference:
    Garland, Michael. Quadric-based polygonal surface simplification. PhD thesis, 1999.
*/

#ifndef QEMQUADRICS_H
#define QEMQUADRICS_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace Eigen;

class QEMQuadrics
{
public:
    Matrix3d A_;
    Vector3d b_;
    double c_;
    int n_;  // not-in-use currently

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    QEMQuadrics();
    QEMQuadrics(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3);  // triangle
    QEMQuadrics(const Vector3d& v1, const Vector3d& v2);                      // edge
    QEMQuadrics(const Vector3d& v1);                                          // point
    bool optimize(Vector3d& v, double& energy);
    double evaluate(const Vector3d& v) const;
    double operator()(const Vector3d& v) const { return evaluate(v); }
    void reset()
    {
        A_.setZero();
        b_.setZero();
        c_ = 0.0;
        n_ = 0;
    }

    QEMQuadrics& operator+=(const QEMQuadrics& Q)
    {
        A_ += Q.A_;
        b_ += Q.b_;
        c_ += Q.c_;
        n_ += Q.n_;
        return *this;
    }
    QEMQuadrics& operator-=(const QEMQuadrics& Q)
    {
        A_ -= Q.A_;
        b_ -= Q.b_;
        c_ -= Q.c_;
        n_ -= Q.n_;
        return *this;
    }
    QEMQuadrics& operator=(const QEMQuadrics& Q)
    {
        A_ = Q.A_;
        b_ = Q.b_;
        c_ = Q.c_;
        n_ = Q.n_;
        return *this;
    }
    QEMQuadrics& operator*=(const double s)
    {
        A_ *= s;
        b_ *= s;
        c_ *= s;
        return *this;
    }
};

#endif
