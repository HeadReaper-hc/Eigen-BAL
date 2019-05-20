//
// Created by huchao on 19-5-20.
//

#ifndef EIGEN_BA_COSTFUNCTION_H
#define EIGEN_BA_COSTFUNCTION_H

#endif //EIGEN_BA_COSTFUNCTION_H

class CostFunction{
public:
    CostFunction(){}
    virtual ~CostFunction(){}

    virtual bool Evaluate(double const *const *parameters) const =0;

    double** jacobians;
    double* residuals;
    int residualDim;
};
