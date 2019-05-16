#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "covariance.h"

CovObj::CovObj()
{
    clearCov();
}

CovObj::CovObj(const Vector3d &v1, const Vector3d &v2, const Vector3d &v3)
{
    area_ = 0.5 * (v2 - v1).cross(v3 - v1).norm();
    center_ = 1 / 3.0 * (v1 + v2 + v3);
    Matrix3d vm, c;
    vm(0, 0) = v1(0);
    vm(1, 0) = v1(1);
    vm(2, 0) = v1(2);
    vm(0, 1) = v2(0);
    vm(1, 1) = v2(1);
    vm(2, 1) = v2(2);
    vm(0, 2) = v3(0);
    vm(1, 2) = v3(1);
    vm(2, 2) = v3(2);
    c(0, 0) = c(1, 1) = c(2, 2) = 2;
    c(0, 1) = c(0, 2) = c(1, 0) = c(1, 2) = c(2, 0) = c(2, 1) = -1;
    cov_ = area_ / 36.0 * vm * c * vm.transpose();
    size_ = 1;
}

void CovObj::clearCov()
{
    cov_.setZero();
    area_ = 0;
    size_ = 0;
    center_ = normal_ = Vector3d::Zero();
}

CovObj &CovObj::operator+=(const CovObj &Q)
{
    // some models may contain corrupted null faces, ignore them
    if (Q.area_ < 1e-18)
        return *this;
    CovObj Q_old = *this;
    area_ = Q_old.area_ + Q.area_;
    center_ = (Q_old.area_ * Q_old.center_ + Q.area_ * Q.center_) / (area_);
    Vector3d ktoi, ktoj;
    ktoi = Q_old.center_ - center_;
    ktoj = Q.center_ - center_;
    cov_ = Q_old.cov_ + Q.cov_ + Q_old.area_ * ktoi * ktoi.transpose() + Q.area_ * ktoj * ktoj.transpose();
    size_ += Q.size_;
    return *this;
}

CovObj &CovObj::operator-=(const CovObj &Q)
{
    // some models may contain corrupted null faces, ignore them
    if (Q.area_ < 1e-18)
        return *this;
    CovObj Q_old = *this;
    area_ = Q_old.area_ - Q.area_;
    center_ = (Q_old.area_ * Q_old.center_ - Q.area_ * Q.center_) / (area_);
    Vector3d ktoi, ktoj;
    ktoi = Q_old.center_ - center_;
    ktoj = Q_old.center_ - Q.center_;
    cov_ = Q_old.cov_ - Q.cov_ - area_ * ktoi * ktoi.transpose() - Q.area_ * ktoj * ktoj.transpose();
    size_ -= Q.size_;
    return *this;
}

bool CovObj::operator==(const CovObj &Q)
{
    return cov_ == Q.cov_ && normal_ == Q.normal_ && center_ == Q.center_ && area_ == Q.area_ && size_ == Q.size_;
}

CovObj &CovObj::operator=(const CovObj &Q)
{
    // A good habit to check for self-assignment
    if (this == &Q)
        return *this;

    cov_ = Q.cov_;
    normal_ = Q.normal_;
    center_ = Q.center_;
    area_ = Q.area_;
    size_ = Q.size_;
    return *this;
}

double CovObj::energy()
{
    if (cov_.determinant() / pow(area_, 5) < 1e-15)
        return cov_.trace() * area_ * 1e-20;
    else
        return cov_.determinant() / pow(area_, 4);
}

void CovObj::computePlaneNormal()
{
    Eigen::SelfAdjointEigenSolver<Matrix3d> es(cov_);
    int smallest = 0;

    for (int i = 1; i < 3; i++)
    {
        if (abs(es.eigenvalues()[i]) < abs(es.eigenvalues()[smallest]))
            smallest = i;
    }
    // Plane normal is the eigenvector corresponding to the smallest eigenvalue
    normal_ = es.eigenvectors().col(smallest);
}
