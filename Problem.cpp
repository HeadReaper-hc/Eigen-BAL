//
// Created by huchao on 19-5-20.
//

#include "Problem.h"
#include <iostream>
using namespace std;
using namespace Eigen;

Problem::Problem():AllResidualDim(0),ConstantParameterDim(0)
                   ,ConstantParameterNum(0),AllParameterDim(0),AllParameterNum(0){}

Problem::~Problem() {}

void Problem::AddParameterBlock(double *param, int size, MyLocalParameterization *Local_Parameter) {

    //find whether the parameter has already existed
    map<double*,ParamBlock*>::iterator iter;
    iter = Addr_ParamBlock_Map.find( param );

    //check the parameter size is or not right.
    if( iter != Addr_ParamBlock_Map.end() ) {
        if ((*iter).second->global_parameter_size != size) {
            cout << "The parameter block has already been added,But the size this time is different from last time"
                 << endl;
            exit(0);
        }

        if (Local_Parameter != nullptr) {
            if ((*iter).second->global_parameter_size != Local_Parameter->GlobalSize()) {
                cout<< "The parameter block has already been added,but the size if different from local_parameter's Global size."
                    << endl;
                exit(0);
            }
        }
        return ;
    }

    //add new parameter block.
    ParamBlock* newparam = new ParamBlock();
    newparam->parameter = param;
    newparam->global_parameter_size = size;
    newparam->Local_Parameter = Local_Parameter;
    Addr_ParamBlock_Map.insert( make_pair( param,newparam ) );
}

void Problem::setParameterBlockConstant(double *param) {

    map<double*,ParamBlock*>::iterator iter;
    iter = Addr_ParamBlock_Map.find( param );

    if( iter == Addr_ParamBlock_Map.end() ){
        cout <<"The parameter doesn't exist,pls add first." <<endl;
        exit(0);
    }

    (*iter).second->is_constant = true;

    return ;
}

void Problem::AddResidualBlock(CostFunction *cost_function, double *param1, double *param2) {

    map<CostFunction*,ResidualBlock*>::iterator iter;
    iter = Addr_ResidualBlock_Map.find( cost_function );

    if( iter != Addr_ResidualBlock_Map.end() ) {
        cout << "The ResidualBlock is already added." << endl;
        return;
    }

    ResidualBlock* newResidualBlock = new ResidualBlock();
    newResidualBlock->cost_function = cost_function;

    map<double*,ParamBlock*>::iterator iter_param1, iter_param2;
    iter_param1 = Addr_ParamBlock_Map.find( param1 );
    iter_param2 = Addr_ParamBlock_Map.find( param2 );
    if( iter_param1 == Addr_ParamBlock_Map.end() ||
         iter_param2 == Addr_ParamBlock_Map.end() ){
        cout << "This ResidualBlock's ParamBlock has not been added,somewhere must wrong."<<endl;
        exit(0);
    }


    newResidualBlock->parameters = new double* [2];
    newResidualBlock->parameters[0] = param1;
    newResidualBlock->parameters[1] = param2;
    newResidualBlock->param_num=2;
    newResidualBlock->param_id.resize(2);
    Addr_ResidualBlock_Map.insert( make_pair( cost_function, newResidualBlock ) );

    (*iter_param1).second->ResidualBlocks.push_back(newResidualBlock);
    (*iter_param2).second->ResidualBlocks.push_back(newResidualBlock);

    return ;


}

bool Problem::BuildProblem() {

    map<CostFunction*, ResidualBlock*>::iterator iter;
    for(iter = Addr_ResidualBlock_Map.begin();iter!=Addr_ResidualBlock_Map.end();++iter){
        AllResidualDim += (*iter).first->residualDim;
    }

    int i=0;
    order_param_block.resize(0);
    order_param_id.resize(0);
    // rearrange the position of parameterblock. the constant paramblock in the front.
    map<double*, ParamBlock*>::iterator order_iter;
    for(order_iter = Addr_ParamBlock_Map.begin(); order_iter!=Addr_ParamBlock_Map.end();++order_iter){
        if( (*order_iter).second->is_constant ) {
            order_param_block.push_back(*order_iter);

            //update the residualblock's param_id
            for(int j=0;j<(*order_iter).second->ResidualBlocks.size();j++){
                auto ptr = (*order_iter).second->ResidualBlocks[j];

                bool find = false;
                for(int k=0;k<ptr->param_num;++k){
                    if( (*order_iter).second->parameter == ptr->parameters[k] ) {
                        ptr->param_id[k] = i;
                        find = true;
                    }
                }
                if(!find){
                    cout<<"ParameterBlock and ResidualBlock not matched,somewhere mush be wrong..."<<endl;
                    return false;
                }
            }

            ConstantParameterNum++;
            AllParameterNum++;
            order_param_id.push_back( AllParameterDim );
            if( (*order_iter).second->Local_Parameter ) {
                ConstantParameterDim += (*order_iter).second->Local_Parameter->LocalSize();
                AllParameterDim += (*order_iter).second->Local_Parameter->LocalSize();

            }
            else{
                ConstantParameterDim += (*order_iter).second->global_parameter_size;
                AllParameterDim += (*order_iter).second->global_parameter_size;
            }
            i++;
        }


    }

    for(order_iter = Addr_ParamBlock_Map.begin();order_iter!=Addr_ParamBlock_Map.end();++order_iter){
        if( (*order_iter).second->is_constant )
            continue;
        order_param_block.push_back((*order_iter));

        //update the residualblock's param_id
        for(int j=0;j<(*order_iter).second->ResidualBlocks.size();j++){
            auto ptr = (*order_iter).second->ResidualBlocks[j];
            bool find = false;
            for(int k=0;k<ptr->param_num;++k){
                if( (*order_iter).second->parameter == ptr->parameters[k] ) {
                    ptr->param_id[k] = i;
                    find = true;
                }

            }
            if(!find){
                cout<<"ParameterBlock and ResidualBlock not matched,somewhere mush be wrong..."<<endl;
                return false;
            }
        }
        i++;

        AllParameterNum++;
        order_param_id.push_back( AllParameterDim );
        if( (*order_iter).second->Local_Parameter ) {
            AllParameterDim += (*order_iter).second->Local_Parameter->LocalSize();

        }
        else{
            AllParameterDim += (*order_iter).second->global_parameter_size;
        }
    }

    if( order_param_block.size() != Addr_ParamBlock_Map.size() ){
        cout<<"The size of order_param_block is not equal with Addr_ParamBlock_Map..."<<endl;
        return false;
    }

    cout<<"All Parameter Number is : "<< AllParameterNum <<endl;
    cout<<"All Parameter Dimension is : "<<AllParameterDim<<endl;
    cout<<"Constant Parameter Number is : "<<ConstantParameterNum<<endl;
    cout<<"Constant Parameter Dimension is : "<<ConstantParameterDim<<endl;

    /*
    for(int i=0;i<order_param_id.size();++i)
    {
        cout<<order_param_id[i]<<endl;
    }
    */
    return true;

}

int Problem::getAllResidualDim() {
    return AllResidualDim;
}

bool Problem::ProblemSolve() {
    if( !BuildProblem() ){
        cout<< "Build Problem failed..."<<endl;
        return false;
    }

    /*
    for(auto iter=Addr_ResidualBlock_Map.begin();iter!=Addr_ResidualBlock_Map.end();++iter){
        int num = (*iter).second->param_num;
        for(int i =0;i<num;i++)
        {
            cout<<(*iter).second->param_id[i]<<"  ";
        }
       // cout<<"------------------------------"<<endl;
    }
     */

    ComputerJacobian();

    return true;
}

bool Problem::ComputerJacobian() {


     //Eigen::MatrixXd jacobian(AllResidualDim,AllParameterDim);  //need too much memory, so I compute the Hession directly.
    Eigen::MatrixXd Hession(AllParameterDim,AllParameterDim);

    Eigen::MatrixXd Residuals(AllResidualDim,1);
    Residuals.setZero();
    int Residuals_id = 0;

    map<CostFunction*,ResidualBlock*>::iterator iter;
    for( iter = Addr_ResidualBlock_Map.begin(); iter != Addr_ResidualBlock_Map.end(); ++iter){
        (*iter).first->Evaluate( (*iter).second->parameters );

        int row_begin = Residuals_id;
        int row_range = (*iter).first->residualDim;

        for(int j=0;j<(*iter).second->param_num;j++){
            int id = (*iter).second->param_id[j];
            int col_begin = order_param_id[ id ];

            //build jacobian and delta_b
            if( order_param_block[id].second->Local_Parameter )
            {
                int globalSize = order_param_block[id].second->global_parameter_size;
                int localSize = order_param_block[id].second->Local_Parameter->LocalSize();
                int col_range = localSize;

                double temp_jacobian[globalSize * localSize];
                order_param_block[id].second->Local_Parameter->ComputeJacobian( order_param_block[id].first,temp_jacobian );

                double* jacobian_j = ( (*iter).first->jacobians )[j];
                Map<MatrixXd> jacobian_j_eigen(jacobian_j,globalSize,row_range);            //this is not colmajor,so I need to transpose it.
                Map<MatrixXd> temp_jacobian_eigen(temp_jacobian,localSize,globalSize);      //this is not colmajor,so I need to transpose it.


            }
            else{
                int globalSize = order_param_block[id].second->global_parameter_size;
                int col_range = globalSize;

                double* jacobian_j = ( (*iter).first->jacobians )[j];
                Map<MatrixXd> jacobian_j_eigen(jacobian_j,globalSize,row_range);             //this is not colmajor,so I need to transpose it.


            }
        }


        double* delta_b = ( (*iter).first->residuals );
        Map<MatrixXd> delta_b_eigen(delta_b,row_range,1);
        Residuals.block(row_begin,0,row_range,1) = delta_b_eigen;
        Residuals_id +=row_range;

    }

    cout <<"The whole Residuals is : " <<Residuals.norm()<<endl;

    return true;
}
