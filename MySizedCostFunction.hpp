//
// Created by huchao on 19-5-20.
//

#ifndef EIGEN_BA_SIZEDCOSTFUNCTION_H
#define EIGEN_BA_SIZEDCOSTFUNCTION_H
#include "CostFunction.hpp"
#include <iostream>
using  namespace std;
typedef int Demension;

template <Demension Residuals_d, Demension parameter_1_d, Demension parameter_2_d = 0, Demension parameter_3_d = 0>
class MySizedCostFunction :public CostFunction{
public:
    MySizedCostFunction(){
        if( Residuals_d == 0 || parameter_1_d == 0 ){
            cout << "The Demension of the Residuals or parameter1 can not be 0..." <<endl;
        }
        else{
           if(parameter_2_d==0){
               jacobians = new double* [1];
               jacobians[0] = new double [Residuals_d*parameter_1_d];}
           else if(parameter_3_d==0){
               jacobians = new double* [2];
               jacobians[0] = new double [Residuals_d*parameter_1_d];
               jacobians[1] = new double [Residuals_d*parameter_2_d];}
           else{
               jacobians = new double* [3];
               jacobians[0] = new double [Residuals_d*parameter_1_d];
               jacobians[1] = new double [Residuals_d*parameter_2_d];
               jacobians[2] = new double [Residuals_d*parameter_3_d];
           }
        }

        residuals = new double[Residuals_d];
        residualDim = Residuals_d;

    }

    virtual ~MySizedCostFunction(){
        if(parameter_2_d==0){
            delete[] jacobians[0];
            delete[] jacobians;
        }else if(parameter_3_d==0){
            delete[] jacobians[0];
            delete[] jacobians[1];
            delete[] jacobians;
        }else{
            delete[] jacobians[0];
            delete[] jacobians[1];
            delete[] jacobians[2];
            delete[] jacobians;
        }
    };

    virtual bool Evaluate(double const *const *parameters){}



};


#endif //EIGEN_BA_SIZEDCOSTFUNCTION_H
