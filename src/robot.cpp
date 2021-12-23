#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <vector>
#include "robot.h"
#include "plan.h"
#include "T_planner.h"

using namespace aris::dynamic;
using namespace aris::plan;

const double PI = aris::PI;

const double T_A = 50;      // t planner acc
const double T_V = 100;     // t planner vel
const double C_A = 1;       // command acc
const double C_V = 1;       // command vel
const double M_A = 5;       // move acc
const double M_V = 5;       // move vel
const double MOV_LEN = 10;  // WASD move distance in mm
const double Z_ZERO = 200;  // initial Z height
const double Z_DROP = 125 - Z_ZERO;  // drop Z distance
const double Z_THR = 1;     // threshold in mm

const double POINT_1[2] = { -150, -150};   // (x,y)
const double POINT_2[2] = { -150,  150};
const double POINT_3[2] = { -300, -150};
const double POINT_4[2] = { -300,  150};
const double POINT_END[2] = {200, 0};

const int X = 0; // X motor index
const int Y = 2; // Y motor index
const int Z = 1; // Z motor index

static double __ZERO_ANGLE[3]; // zero angles

/*
_________
|   |   |
|  END  |
|   |   |
|_(0,0)_|
| 1 | 2 |
| 3 | 4 |
---------
x ^
  |__> y

Example Command Flow:

    c           // lift Z to initial pos and record this XYZ pos as (0,0)
    1/2/3/4     // move gripper to predefined point
    f           // drop Z
    e           // pick and place
    r           // return to (0,0)

*/

namespace robot
{

static void set_zero_angle(double* pos){
    for(int idx=0; idx<3; idx++)
        __ZERO_ANGLE[idx] = pos[idx];
}

static double* get_zero_angle(){
    return __ZERO_ANGLE;
}

//* RETURNZ ////////////////////////////////////////
ReturnZ::ReturnZ(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"r\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto ReturnZ::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions())
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ReturnZ::executeRT()->int //进入实时线程
{
    static double begin_angle[3];
    double angle, x_pos, y_pos, z_pos;
    double totaltime = 0.0;
    // double xyz_pos[3] = {};
    double xyz_pt1[3] = {};
    double xyz_pt2[3] = {};
    double seg_time;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
        begin_angle[2] = controller()->motionPool()[Z].actualPos();
    }

    x_pos = -(begin_angle[0] - __ZERO_ANGLE[0]);
    y_pos = -(begin_angle[1] - __ZERO_ANGLE[1]);
    z_pos = -(begin_angle[2] - __ZERO_ANGLE[2]);

    TPlanner t_plan(T_A, T_V);
    xyz_pt1 = {x_pos, y_pos, 0};
    xyz_pt2 = {0, 0, z_pos};
    // TCurve s1(C_A, C_V); //s1(a,v)
    // s1.getCurveParam(); // 计算曲线参数

    if (xyz_pos[2]<Z_THR && xyz_pos[2]>-Z_THR){ // Z pos near zero
        t_plan.update(xyz_pt1);
        totaltime = t_plan.getPlanTime();

        angle = begin_angle[0] + t_plan.getXCurve(count());
        controller()->motionPool()[X].setTargetPos(angle);
        angle = begin_angle[1] + t_plan.getYCurve(count());
        controller()->motionPool()[Y].setTargetPos(angle);
    }else{
        //! update here
        seg_time = t_plan.getPlanTime(xyz_pt2);
        totaltime = seg_time + t_plan.getPlanTime(xyz_pt1);

        if(count() <= seg_time){ // 抬升夹爪
            t_plan.update(xyz_pt2);
            angle = begin_angle[2] + t_plan.getZCurve(count());
            controller()->motionPool()[Z].setTargetPos(angle);

        }else if(seg_time < count() && count() <= totaltime){ // 夹爪回(0, 0)
            t_plan.update(xyz_pt1);
            angle = begin_angle[0] + t_plan.getXCurve(count()-seg_time);
            controller()->motionPool()[X].setTargetPos(angle);
            angle = begin_angle[1] + t_plan.getYCurve(count()-seg_time);
            controller()->motionPool()[Y].setTargetPos(angle);
        }

        // totaltime = s1.getTc()*2000;
        // if(count() <= s1.getTc()*1000){ // 抬升夹爪
        //     angle = begin_angle[2] + z_pos * s1.getTCurve(count());
        //     controller()->motionPool()[Z].setTargetPos(angle);

        // }else if(s1.getTc()*1000 < count() && count() <= s1.getTc()*2000){ // 夹爪回(0, 0)
        //     angle = begin_angle[0] + x_pos * s1.getTCurve(count()-s1.getTc()*1000);
        //     controller()->motionPool()[X].setTargetPos(angle);
        //     angle = begin_angle[1] + y_pos * s1.getTCurve(count()-s1.getTc()*1000);
        //     controller()->motionPool()[Y].setTargetPos(angle);
        // }
    }

    return totaltime - count();
}
auto ReturnZ::collectNrt()->void {}

//* POINTEND //////////////////////////////////////
Place::Place(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"e\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto Place::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Place::executeRT()->int //进入实时线程
{
    static double begin_angle[3];
    double angle;
    double x_pos, y_pos;
    double xyz_pt1[3], xyz_pt2[3], xyz_pt3[3];
    double totaltime, seg_time1, seg_time2;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
        begin_angle[2] = controller()->motionPool()[Z].actualPos();
    }

    x_pos = -(begin_angle[0]-__ZERO_ANGLE[0]) + POINT_END[0]/36.0*PI;
    y_pos = -(begin_angle[1]-__ZERO_ANGLE[1]) + POINT_END[1]/36.0*PI;
    
    TPlanner t_plan(T_A, T_V);
    xyz_pt1 = {0, 0, +Z_DROP/36.0*PI};
    xyz_pt2 = {x_pos, y_pos, 0};
    xyz_pt3 = {0, 0, -Z_DROP/36.0*PI};
    seg_time1 = t_plan.getPlanTime(xyz_pt1);
    seg_time2 = seg_time1 + t_plan.getPlanTime(xyz_pt2);
    totaltime = seg_time2 + t_plan.getPlanTime(xyz_pt3);

    // TCurve s1(C_A, C_V); // s(a,v)
    // s1.getCurveParam();
    // mid_time = s1.getTc()*1000 + t_plan.getPlanTime();
    // totaltime = mid_time + s1.getTc()*1000;

    //! update here
    if(count() <= seg_time1){ // 上升夹爪
        t_plan.update(xyz_pt1);
        angle = begin_angle[2] + t_plan.getZCurve(count());
        controller()->motionPool()[Z].setTargetPos(angle);

        if (count()==seg_time1){ // record new begin_angle in Z
            begin_angle[2] = angle;
        }
    }
    else if(seg_time1 < count() && count() <= seg_time2){ // 移动到 End Point
        t_plan.update(xyz_pt2);
        angle = begin_angle[0] + t_plan.getXCurve(count()-seg_time1);
        controller()->motionPool()[X].setTargetPos(angle);
        angle = begin_angle[1] + t_plan.getYCurve(count()-seg_time1);
        controller()->motionPool()[Y].setTargetPos(angle);
    }
    else if(seg_time2 < count() && count() <= totaltime){ // 放下夹爪
        t_plan.update(xyz_pt3);
        angle = begin_angle[2] + t_plan.getZCurve(count()-seg_time2);
        controller()->motionPool()[Z].setTargetPos(angle);
    }

    return totaltime - count();
}
auto Place::collectNrt()->void {}

//* ZEROZ ////////////////////////////////////////
ZeroZ::ZeroZ(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"c\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto ZeroZ::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ZeroZ::executeRT()->int //进入实时线程
{
    static double begin_angle[3];
    double totaltime;
    double xyz_pos[3] = {};
    double angle;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
        begin_angle[2] = controller()->motionPool()[Z].actualPos();
    }

    xyz_pos[2] = -Z_ZERO/36.0*PI;

    TPlanner t_plan(T_A, T_V, xyz_pos);
    totaltime = t_plan.getPlanTime();

    // lift Z from 0 to Z_ZERO
    angle = begin_angle[2] + t_plan.getZCurve(count());
    controller()->motionPool()[Z].setTargetPos(angle);

    // record the final Z pos as the __ZERO_ANGLE
    if(count()==int(totaltime)){
        begin_angle[2] = angle;
        set_zero_angle(begin_angle);
    }

    return totaltime - count();
}
auto ZeroZ::collectNrt()->void {}

//* POINT1 ////////////////////////////////////////
Pt1::Pt1(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"1\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto Pt1::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Pt1::executeRT()->int //进入实时线程
{
    static double begin_angle[2];
    double angle;
    double xyz_pos[3] = {};
    long totaltime;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    xyz_pos[0] = -(begin_angle[0]-__ZERO_ANGLE[0]) + POINT_1[0]/36.0*PI;
    xyz_pos[1] = -(begin_angle[1]-__ZERO_ANGLE[1]) + POINT_1[1]/36.0*PI;
    
    TPlanner t_plan(T_A, T_V, xyz_pos);
    totaltime = t_plan.getPlanTime();

    angle = begin_angle[0] + t_plan.getXCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + t_plan.getYCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return totaltime - count();
}
auto Pt1::collectNrt()->void {}

//* POINT2 ////////////////////////////////////////
Pt2::Pt2(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"2\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto Pt2::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Pt2::executeRT()->int //进入实时线程
{
    static double begin_angle[2];
    double angle;
    double xyz_pos[3] = {};
    long totaltime;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    xyz_pos[0] = -(begin_angle[0]-__ZERO_ANGLE[0]) + POINT_2[0]/36.0*PI;
    xyz_pos[1] = -(begin_angle[1]-__ZERO_ANGLE[1]) + POINT_2[1]/36.0*PI;
    
    TPlanner t_plan(T_A, T_V, xyz_pos);
    totaltime = t_plan.getPlanTime();

    angle = begin_angle[0] + t_plan.getXCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + t_plan.getYCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return totaltime - count();
}
auto Pt2::collectNrt()->void {}

//* POINT3 ////////////////////////////////////////
Pt3::Pt3(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"3\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto Pt3::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Pt3::executeRT()->int //进入实时线程
{
    static double begin_angle[2];
    double angle;
    double xyz_pos[3] = {};
    long totaltime;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    xyz_pos[0] = -(begin_angle[0]-__ZERO_ANGLE[0]) + POINT_3[0]/36.0*PI;
    xyz_pos[1] = -(begin_angle[1]-__ZERO_ANGLE[1]) + POINT_3[1]/36.0*PI;
    
    TPlanner t_plan(T_A, T_V, xyz_pos);
    totaltime = t_plan.getPlanTime();

    angle = begin_angle[0] + t_plan.getXCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + t_plan.getYCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return totaltime - count();
}
auto Pt3::collectNrt()->void {}

//* POINT4 ////////////////////////////////////////
Pt4::Pt4(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"4\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto Pt4::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Pt4::executeRT()->int //进入实时线程
{
    static double begin_angle[2];
    double angle;
    double xyz_pos[3] = {};
    long totaltime;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    xyz_pos[0] = -(begin_angle[0]-__ZERO_ANGLE[0]) + POINT_4[0]/36.0*PI;
    xyz_pos[1] = -(begin_angle[1]-__ZERO_ANGLE[1]) + POINT_4[1]/36.0*PI;
    
    TPlanner t_plan(T_A, T_V, xyz_pos);
    totaltime = t_plan.getPlanTime();

    angle = begin_angle[0] + t_plan.getXCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + t_plan.getYCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return totaltime - count();
}
auto Pt4::collectNrt()->void {}

//* DROPZ ////////////////////////////////////////
DropZ::DropZ(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"f\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto DropZ::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto DropZ::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[Z].actualPos();
    }

    TCurve s1(C_A, C_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle = begin_angle - Z_DROP/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[Z].setTargetPos(angle);

    return s1.getTc() * 1000 - count();
}
auto DropZ::collectNrt()->void {}

//* MOVEW ////////////////////////////////////////
MoveW::MoveW(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"w\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto MoveW::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MoveW::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[X].actualPos();
    }
    
    TCurve s1(M_A, M_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle + MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[X].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveW::collectNrt()->void {}

//* MOVES ////////////////////////////////////////
MoveS::MoveS(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"s\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto MoveS::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MoveS::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[X].actualPos();
    }
    
    TCurve s1(M_A, M_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle - MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[X].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveS::collectNrt()->void {}

//* MOVED ////////////////////////////////////////
MoveD::MoveD(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"d\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto MoveD::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MoveD::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[Y].actualPos();
    }

    TCurve s1(M_A, M_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle + MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[Y].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveD::collectNrt()->void {}

//* MOVEA ////////////////////////////////////////
MoveA::MoveA(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"a\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto MoveA::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MoveA::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[Y].actualPos();
    }

    TCurve s1(M_A, M_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle - MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[Y].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveA::collectNrt()->void {}

//* My Drive /////////////////////////////////////
MyDrive::MyDrive(const std::string &name) //构造函数
{
    //* moveit -c=1 //record cur pos as zero pos
    //* moveit -x=100 -y=50 -z=60 // move independently
    //* moveit -p=1 -l=100 -s=90 // draw rectangle
    //* moveit -r=1 // return to zero pos
    aris::core::fromXmlString(command(),
        "<Command name=\"moveit\">"
        "	<GroupParam>"
        "	<Param name=\"x_pos\" default=\"0\" abbreviation=\"x\"/>"
        "	<Param name=\"y_pos\" default=\"0\" abbreviation=\"y\"/>"
        "	<Param name=\"z_pos\" default=\"0\" abbreviation=\"z\"/>"
        "	<Param name=\"reset\" default=\"0\" abbreviation=\"r\"/>"
        "	<Param name=\"set_zero\" default=\"0\" abbreviation=\"c\"/>"
        "	</GroupParam>"
        "</Command>");
}
auto MyDrive::prepareNrt() -> void{
    x_pos = doubleParam("x_pos");
    y_pos = doubleParam("y_pos");
    z_pos = -doubleParam("z_pos");
    reset = int32Param("reset");
    set_zero = int32Param("set_zero");

    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MyDrive::executeRT() -> int{
    static double cur_angle[3];
    double * temp_angle;
    int idx;
    double angle;
    double totaltime;

    if(count()==1){
        cur_angle[0] = controller()->motionPool()[X].actualPos();
        cur_angle[1] = controller()->motionPool()[Y].actualPos();
        cur_angle[2] = controller()->motionPool()[Z].actualPos();
    }

    if(set_zero){
        set_zero_angle(cur_angle);
        return 0;
    }

    if(reset){
        temp_angle = get_zero_angle();
        x_pos = -(cur_angle[0] - temp_angle[0])*36.0/PI;
        y_pos = -(cur_angle[1] - temp_angle[1])*36.0/PI;
        z_pos = -(cur_angle[2] - temp_angle[2])*36.0/PI;
        reset = 0;
    }

    TCurve s_curve(1,1);
    s_curve.getCurveParam();

    totaltime = 1000*s_curve.getTc();

    angle = cur_angle[0] + x_pos/36.0*PI*s_curve.getTCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);

    angle = cur_angle[1] + y_pos/36.0*PI*s_curve.getTCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    angle = cur_angle[2] + z_pos/36.0*PI*s_curve.getTCurve(count());
    controller()->motionPool()[Z].setTargetPos(angle);

    return totaltime - count();
}
auto MyDrive::collectNrt()->void {}

//* 单关节正弦往复轨迹 ////////////////////////////
struct MoveJSParam
{
    double j1;
    double time;
    uint32_t timenum;
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motionPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto &option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJS::executeRT()->int
{
    auto &param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //
    auto &cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
            this->master()->logFileRawName("TestMotor");//建立记录数据的文件夹
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }

    // 打印 //
    if (count() % 10 == 0)
    {
        std::cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
        std::cout << "vel" << ":" << controller()->motionAtAbs(0).actualVel() << "  ";
        std::cout << std::endl;
    }

    //* log //
    // auto &lout = controller()->lout();
    // lout << controller()->motionAtAbs(0).targetPos() << ",";
    // lout << std::endl;
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;

    return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJS\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}

auto createControllerMotor()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 3; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[2]
        {
            0,0,0,0,0,0
        };
#else
        double pos_offset[2]
        {
         //  1.900100

        };
#endif
        double pos_factor[3] //偏置系数
        {
            2000/PI,2000/PI,2000/PI
        };
        double max_pos[3] //最大位置
        {
            500*PI,500*PI,500*PI
        };
        double min_pos[3] //最小位置
        {
            -500*PI,-500*PI,-500*PI
        };
        double max_vel[3]  //最大速度
        {
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[3]  //最大加速度
        {
            3000,  3000,  3000
        };

        int phy_id[3]={0,1,2};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1600\" is_tx=\"false\">"
            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";


        auto &s = controller->slavePool().add<aris::control::EthercatMotor>();
        aris::core::fromXmlString(s,xml_str);

#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setDcAssignActivate(0x300);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setControlWord(0x00);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setModeOfOperation(0x08);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setTargetPos(0.0);
    };
    return controller;
}
auto createPlanMotor()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();
    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //自己写的命令
//    plan_root->planPool().add<TcurveDrive>();
    plan_root->planPool().add<MyDrive>();
    plan_root->planPool().add<MoveJS>();
    plan_root->planPool().add<MoveW>();
    plan_root->planPool().add<MoveS>();
    plan_root->planPool().add<MoveD>();
    plan_root->planPool().add<MoveA>();
    plan_root->planPool().add<ReturnZ>();
    plan_root->planPool().add<DropZ>();
    plan_root->planPool().add<ZeroZ>();
    plan_root->planPool().add<Pt1>();
    plan_root->planPool().add<Pt2>();
    plan_root->planPool().add<Pt3>();
    plan_root->planPool().add<Pt4>();
    plan_root->planPool().add<Place>();
    return plan_root;
}

}
