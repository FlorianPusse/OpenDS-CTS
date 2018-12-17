#include "math_utils.h"


MyVector::MyVector(double _dw,double _dh) {
    dw=_dw;
    dh=_dh;
}
MyVector::MyVector(double angle,double length,int dummy)
{
	dw=length*ModelParams::lookupTable->cos(angle);
	dh=length*ModelParams::lookupTable->sin(angle);
}
double MyVector::GetAngle()   //[-pi,pi]
{
	return atan2(dh,dw);
}
double MyVector::GetLength()
{
	return sqrt(dh*dh+dw*dw);
}
void MyVector::GetPolar(double &angle,double &length)
{
	angle=GetAngle();
	length=GetLength();
}
void MyVector::AdjustLength(double length)
{
	if(GetLength()<0.1) return;   //vector length close to 0
	double rate=length/GetLength();
	dw*=rate;
	dh*=rate;
}

MyVector  MyVector::operator + (MyVector  vec)
{
	return MyVector(dw+vec.dw,dh+vec.dh);
}

double DotProduct(double x1,double y1, double x2,double y2)
{
	return x1*x2+y1*y2;
}

double CrossProduct(MyVector vec1, MyVector vec2)
{
	return vec1.dw*vec2.dh-vec1.dh*vec2.dw;
}

double Norm(double x,double y)
{
	return sqrt(x*x+y*y);
}

void Uniform(double x,double y,double &ux,double &uy)
{
	double l=Norm(x,y);
	ux=x/l;
	uy=y/l;
}

