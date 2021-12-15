#ifndef ROBOT_H_
#define ROBOT_H_

#include <aris.hpp>
#include <memory>

namespace robot
{
double ZERO_ANGLE[3]; // zero angles

void set_zero_angle(double* pos);
// static double * get_zero_angle();

class Place : public aris::core::CloneObject<Place, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit Place(const std::string &name = "pick_place");

  private:
    double len;
};

class Pt1 : public aris::core::CloneObject<Pt1, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit Pt1(const std::string &name = "pt_1");

  private:
    double len;
};

class Pt2 : public aris::core::CloneObject<Pt2, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit Pt2(const std::string &name = "pt_2");

  private:
    double len;
};

class Pt3 : public aris::core::CloneObject<Pt3, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit Pt3(const std::string &name = "pt_3");

  private:
    double len;
};

class Pt4 : public aris::core::CloneObject<Pt4, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit Pt4(const std::string &name = "pt_4");

  private:
    double len;
};

class MoveW : public aris::core::CloneObject<MoveW, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MoveW(const std::string &name = "move_w");

  private:
    double len;
};

class MoveS : public aris::core::CloneObject<MoveS, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MoveS(const std::string &name = "move_s");

  private:
    double len;
};

class MoveD : public aris::core::CloneObject<MoveD, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MoveD(const std::string &name = "move_d");
    
  private:
    double len;
};

class MoveA : public aris::core::CloneObject<MoveA, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MoveA(const std::string &name = "move_a");
    
  private:
    double len;
};

class ReturnZ : public aris::core::CloneObject<ReturnZ, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit ReturnZ(const std::string &name = "zero_z");
    
  private:
    double len;
};

class DropZ : public aris::core::CloneObject<DropZ, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit DropZ(const std::string &name = "drop_z");
    
  private:
    double len;
};

class ZeroZ : public aris::core::CloneObject<ZeroZ, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit ZeroZ(const std::string &name = "zero_z");
    
  private:
    double len;
};

class MyDrive : public aris::core::CloneObject<MyDrive, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MyDrive(const std::string &name = "my_drive");

  private:
    double x_pos, y_pos, z_pos; // mm
    int32_t reset; // return to zero positions
    int32_t set_zero; // set current positions as zero positions
    int32_t draw_rect;
};

class TcurveDrive : public aris::core::CloneObject<TcurveDrive, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    virtual ~TcurveDrive();
    explicit TcurveDrive(const std::string &name = "motor_drive");

  private:
    double dir_;
};

class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
{
  public:
    auto virtual prepareNrt() -> void;
    auto virtual executeRT() -> int;
    auto virtual collectNrt() -> void;

    explicit MoveJS(const std::string &name = "MoveJS_plan");
};

auto createControllerMotor() -> std::unique_ptr<aris::control::Controller>;
auto createPlanMotor() -> std::unique_ptr<aris::plan::PlanRoot>;
} // namespace robot

#endif
