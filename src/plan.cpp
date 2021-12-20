#include"plan.h"
#include<cmath>
#include<iostream>
using namespace std;


/////////////////////////////////////梯形曲线///////////////////////////////////////////////
//生成梯形曲线0->1
//输入：时间，每毫秒计数一次
//输出：当前时刻s的值
auto TCurve::getTCurve(int count)->double
{

	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
        this->Tc_ = ((a_ + v_ * v_) / v_ / a_);
        // this->a_ = a_;
        // this->v_ = v_;
	}
	else
	{
        //按速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
        // this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}
