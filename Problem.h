//
// Created by huchao on 19-5-20.
//

#ifndef EIGEN_BA_PROBLEM_H
#define EIGEN_BA_PROBLEM_H
#include "MyLocalParameterization.hpp"
#include "MySizedCostFunction.hpp"
#include <Eigen/Dense>
#include <map>
#include <vector>
using namespace std;

struct ResidualBlock;

struct ParamBlock{
    ParamBlock():is_constant(false),Local_Parameter(nullptr){};
    double* parameter;
    bool is_constant;
    int global_parameter_size;
    MyLocalParameterization* Local_Parameter;
    vector<ResidualBlock*> ResidualBlocks;
};

struct ResidualBlock{
    CostFunction* cost_function;
    double** parameters;
    int param_num;
    vector<int> param_id;
};



class Problem {
public:
    Problem();
    ~Problem();

    void AddParameterBlock(double* param, int size, MyLocalParameterization* Local_Parameter = nullptr);

    void setParameterBlockConstant(double* param);

    void AddResidualBlock(CostFunction* cost_function, double* param1);

    void AddResidualBlock(CostFunction* cost_function, double* param1, double* param2);

    void AddResidualBlock(CostFunction* cost_function, double* param1, double* param2, double* param3);

    int getAllResidualDim();

    bool ProblemSolve();


private:

    bool BuildProblem();

    bool ComputerAndUpdate();

    map<double*,ParamBlock*> Addr_ParamBlock_Map;
    map<CostFunction*,ResidualBlock*> Addr_ResidualBlock_Map;

    int AllResidualDim;
    int ConstantParameterDim;
    int ConstantParameterNum;
    int AllParameterDim;
    int AllParameterNum;
    vector<pair<double*,ParamBlock*>> order_param_block;
    vector<int> order_param_id;

};


#endif //EIGEN_BA_PROBLEM_H
