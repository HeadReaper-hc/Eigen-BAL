//
// Created by huchao on 19-5-17.
//

#ifndef SBA_PROJECTFACTOR_H
#define SBA_PROJECTFACTOR_H
#include "MySizedCostFunction.hpp"
#include <opencv2/opencv.hpp>


class ProjectFactor : public MySizedCostFunction<2,10,3>{
public:
    ProjectFactor(cv::Point2f _pt):pt(_pt){}
    virtual bool Evaluate(double const *const *parameters) const;

private:

    cv::Point2f pt;

};




#endif //SBA_PROJECTFACTOR_H
