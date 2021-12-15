#ifndef PLAN_H_
#define PLAN_H_




///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;

public:
	auto getTCurve(int count)->double;
	auto getCurveParam()->void;
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; }
	~TCurve() {}
};

#endif



