#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H
#pragma once

#include <math.h>
#include <iostream>

using namespace std;

namespace SplineSpace
{
    class SplineFailure    // Exceptions
    {
    private:
        const char* Message;
    public:
        SplineFailure(const char* msg);
        const char* GetMessage();
    };

    class SplineInterface  // Interfaces
    {
    public:
        virtual bool SinglePointInterp(const double& x,double& y)=0;
        virtual bool MultiPointInterp(const double* x,const int& num,double* y)=0;
        virtual bool AutoInterp(const int& num,double* x,double* y)=0;
        virtual ~SplineInterface(){}
    };

    enum BoundaryCondition
    {
        GivenFirstOrder=1 , GivenSecondOrder
    };

    class Spline:public SplineInterface	 // 样条插值
    {
    public:
        //输入： x0 y0,num数据的个数
        Spline(const double* x0,const double* y0,const int& num,
            BoundaryCondition bc=GivenSecondOrder,const double& leftBoundary=0,const double& rightBoundary=0);

        bool SinglePointInterp(const double& x,double& y)throw(SplineFailure);
        bool MultiPointInterp(const double* x,const int& num,double* y)throw(SplineFailure);
        bool AutoInterp(const int& num,double* x,double* y)throw(SplineFailure);

        ~Spline();

    private:
        // 求导（一阶和二阶）
        void PartialDerivative1(void);
        void PartialDerivative2(void);

        const double* GivenX;	//已知数据的自变量
        const double* GivenY;	//已知数据的因变量
        const int GivenNum;
        const BoundaryCondition Bc;	//边界类型
        const double LeftB;
        const double RightB;

        double* PartialDerivative;
        double MaxX;
        double MinX;

        const double* SplineX;
        double* SplineY;
        const int* SplineNum;
    };
}

#endif // CUBICSPLINE_H
