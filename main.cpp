#include <iostream>
#include "readData.h"
#include "Problem.h"
#include "ProjectFactor.h"
#include <Eigen/Dense>
#include "CameraParameterization.h"
#include <ceres/rotation.h>
#include <chrono>
using namespace std;
using namespace Eigen;
using namespace cv;
using namespace ceres;
void camVector2double( optimData& data, double** newData );
void pointsVector2double( optimData& data, double** newData);

int main(int argc, char** argv) {

    if( argc!=2 ) {
        cout << "At least need two parameters , usage as " << argv[0] << " ${file_path}" << endl;
        return 0;
    }


    string path = argv[1];

    optimData sbaData;

    readData rd(path);
    if(rd.getDataFromTxt(sbaData))
        cout<<"Read data process is successful..." <<endl;
    else {
        cout<<"Read data process is failed..."<<endl;
        return 0;
    }




    auto time_begin = chrono::steady_clock::now();


    double** camData = new double* [sbaData.numCam];
    for(int i=0;i<sbaData.numCam;++i){
        camData[i] =new double [10];
    }

    double** pointData = new double* [sbaData.numPoints];
    for(int i=0;i<sbaData.numPoints;++i){
        pointData[i] = new double [3];
    }

   camVector2double(sbaData, camData);
   pointsVector2double(sbaData, pointData);

   Problem problem;

   for(int i=0;i<sbaData.numCam;++i){
      /* ceres::LocalParameterization *local_parameterization = new ProductParameterization(
               new QuaternionParameterization(),
               new IdentityParameterization(6)); */
       MyLocalParameterization* local_parameter = new CameraParameterization();
       problem.AddParameterBlock(camData[i],10,local_parameter);
   }

   for(int i=0;i<sbaData.numPoints;++i){
       problem.AddParameterBlock(pointData[i],3);
   }

   problem.setParameterBlockConstant(camData[0]);
   problem.setParameterBlockConstant(pointData[0]);


    for(int i=0;i<sbaData.numCam;++i){
       map<int,Point2f>::iterator iter;
       for(iter = (sbaData.cam_ptn_pt)[i].begin();iter != (sbaData.cam_ptn_pt)[i].end(); iter++){
           CostFunction* cost_function =
                           new ProjectFactor( (*iter).second  );
           problem.AddResidualBlock(cost_function,camData[i], pointData[(*iter).first]);
       }
   }

    problem.ProblemSolve();
    cout <<"All residual dim : " <<problem.getAllResidualDim() <<endl;

    return 0;
}


void camVector2double( optimData& data, double** newData )
{
    for(int i=0;i<data.numCam;++i){
        double x[3];
        x[0] = ((data.cams)[i])[0];
        x[1] = ((data.cams)[i])[1];
        x[2] = ((data.cams)[i])[2];
        AngleAxisToQuaternion( x,newData[i] );
        newData[i][4] = ((data.cams)[i])[3];
        newData[i][5] = ((data.cams)[i])[4];
        newData[i][6] = ((data.cams)[i])[5];
        newData[i][7] = ((data.cams)[i])[6];
        newData[i][8] = ((data.cams)[i])[7];
        newData[i][9] = ((data.cams)[i])[8];

    }
}

void pointsVector2double( optimData& data, double** newData)
{
    for(int i=0;i<data.numPoints;++i)
    {
        newData[i][0] = ((data.pts)[i])[0];
        newData[i][1] = ((data.pts)[i])[1];
        newData[i][2] = ((data.pts)[i])[2];

    }
}