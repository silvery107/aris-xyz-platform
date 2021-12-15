#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <vector>
#include "robot.h"
#include "plan.h"

using namespace aris::dynamic;
using namespace aris::plan;

const double PI = aris::PI;
const double C_A = 5;
const double C_V = 2;
const double MOV_LEN = 10.0;
const double POINT_1[2] = {-50, -25}; // (x,y)
const double POINT_2[2] = { 50, -25};
const double POINT_3[2] = {-50, -75};
const double POINT_4[2] = { 50, -75};
const int X = 0;
const int Y = 2;
const int Z = 1;

/*

|_(0,0)_|
| 1 | 2 |
| 3 | 4 |

x ^
  |__> y

*/

namespace robot
{

static void set_zero_angle(double* pos){
    for(int idx=0; idx<3; ++idx)
        zero_pos[idx] = pos[idx];
}
static double* get_zero_angle(){
    return zero_pos;
}

//* POINT1 ////////////////////////////////////////
Pt1::Pt1(const std::string &name) //构造函数
{
    // ! Check if number command work
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
    double x_pos, y_pos;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    x_pos = -(begin_angle[0]*36.0/PI - POINT_1[0]);
    y_pos = -(begin_angle[1]*36.0/PI - POINT_1[1]);
    
    TCurve s1(C_A, C_V); // s(a,v)
    s1.getCurveParam();

    angle = begin_angle[0] + x_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + y_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return s1.getTc() * 1000 - count();
}
auto Pt1::collectNrt()->void {}

//* POINT2 ////////////////////////////////////////
Pt2::Pt2(const std::string &name) //构造函数
{
    // ! Check if number command work
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
    double x_pos, y_pos;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    x_pos = -(begin_angle[0]*36.0/PI - POINT_2[0]);
    y_pos = -(begin_angle[1]*36.0/PI - POINT_2[1]);
    
    TCurve s1(C_A, C_V); // s(a,v)
    s1.getCurveParam();

    angle = begin_angle[0] + x_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + y_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return s1.getTc() * 1000 - count();
}
auto Pt2::collectNrt()->void {}

//* POINT3 ////////////////////////////////////////
Pt3::Pt3(const std::string &name) //构造函数
{
    // ! Check if number command work
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
    double x_pos, y_pos;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    x_pos = -(begin_angle[0]*36.0/PI - POINT_3[0]);
    y_pos = -(begin_angle[1]*36.0/PI - POINT_3[1]);
    
    TCurve s1(C_A, C_V); // s(a,v)
    s1.getCurveParam();

    angle = begin_angle[0] + x_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + y_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return s1.getTc() * 1000 - count();
}
auto Pt3::collectNrt()->void {}

//* POINT4 ////////////////////////////////////////
Pt4::Pt4(const std::string &name) //构造函数
{
    // ! Check if number command work
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
    double x_pos, y_pos;

    if(count()==1){
        begin_angle[0] = controller()->motionPool()[X].actualPos();
        begin_angle[1] = controller()->motionPool()[Y].actualPos();
    }

    x_pos = -(begin_angle[0]*36.0/PI - POINT_4[0]);
    y_pos = -(begin_angle[1]*36.0/PI - POINT_4[1]);
    
    TCurve s1(C_A, C_V); // s(a,v)
    s1.getCurveParam();

    angle = begin_angle[0] + x_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[X].setTargetPos(angle);
    angle = begin_angle[1] + y_pos/36.0*PI * s1.getTCurve(count());
    controller()->motionPool()[Y].setTargetPos(angle);

    return s1.getTc() * 1000 - count();
}
auto Pt4::collectNrt()->void {}

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
    
    TCurve s1(C_A, C_V); //s1(a,v)
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
    
    TCurve s1(C_A, C_V); //s1(a,v)
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

    TCurve s1(C_A, C_V); //s1(a,v)
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

    TCurve s1(C_A, C_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle - MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[Y].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveA::collectNrt()->void {}

//* MOVEZ ////////////////////////////////////////
MoveZ::MoveZ(const std::string &name) //构造函数
{
    //! change this to <space> if work
    aris::core::fromXmlString(command(),
       "<Command name=\"f\">"
        "	<Param name=\"len\" default=\"10\" abbreviation=\"n\"/>"
        "</Command>");
}
auto MoveZ::prepareNrt()->void
{
    len = doubleParam("len");
    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto MoveZ::executeRT()->int //进入实时线程
{
    static double begin_angle;

    if(count()==1){
        begin_angle = controller()->motionPool()[Z].actualPos();
    }

    TCurve s1(C_A, C_V); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle - MOV_LEN/36.0*PI * s1.getTCurve(count()); // 返回count时刻的角度

    controller()->motionPool()[Z].setTargetPos(angle0);

    return s1.getTc() * 1000 - count();
}
auto MoveZ::collectNrt()->void {}

//* My Drive //////////////////////////////////////////////
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
    z_pos = doubleParam("z_pos");
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
        for(idx=0; idx<3; ++idx)
            cur_angle[idx] = controller()->motionPool()[idx].actualPos();
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


//* 梯形曲线位置控制
auto TcurveDrive::prepareNrt()->void
{
    dir_ = doubleParam("direction");

    for(auto &m:motorOptions()) 
        m = aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}

auto TcurveDrive::executeRT()->int //进入实时线程
{
    static double begin_angle[3];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        this->master()->logFileRawName("TestMotor");//建立记录数据的文件夹
    }

    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(5,2); //s1(a,v)
    s1.getCurveParam(); // 计算曲线参数
    double angle0 = begin_angle[0] + PI * dir_ * s1.getTCurve(count()); // 返回count时刻的角度
    controller()->motionPool()[0].setTargetPos(angle0);
    // mout() << angle0 << std::endl;
    lout() << controller()->motionAtAbs(0).actualPos() <<"\t";
    lout() << controller()->motionAtAbs(0).actualVel() <<std::endl;
    return s1.getTc() * 1000 - count(); //运行时间为T型曲线的周期
}

auto TcurveDrive::collectNrt()->void {}
TcurveDrive::TcurveDrive(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_mvj\">"
        "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
TcurveDrive::~TcurveDrive() = default;  //析构函数

//* 单关节正弦往复轨迹
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
    plan_root->planPool().add<TcurveDrive>();
    plan_root->planPool().add<MyDrive>();
    plan_root->planPool().add<MoveJS>();
    plan_root->planPool().add<MoveW>();
    plan_root->planPool().add<MoveS>();
    plan_root->planPool().add<MoveD>();
    plan_root->planPool().add<MoveA>();
    plan_root->planPool().add<MoveZ>();
    plan_root->planPool().add<Pt1>();
    plan_root->planPool().add<Pt2>();
    plan_root->planPool().add<Pt3>();
    plan_root->planPool().add<Pt4>();
    return plan_root;
}

}
