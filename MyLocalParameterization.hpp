//
// Created by huchao on 19-5-20.
//

#ifndef EIGEN_BA_MYLOCALPARAMETERIZATION_H
#define EIGEN_BA_MYLOCALPARAMETERIZATION_H


class MyLocalParameterization {
public:
    MyLocalParameterization() {};

    virtual ~MyLocalParameterization() {};


    virtual bool Plus(const double *x,
                      const double *delta,
                      double *x_plus_delta) const = 0;

    virtual bool ComputeJacobian(const double *x,
                                 double *jacobian) const = 0;

    virtual int GlobalSize() const = 0;

    virtual int LocalSize() const = 0;

};

#endif //EIGEN_BA_MYLOCALPARAMETERIZATION_H