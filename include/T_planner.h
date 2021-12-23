#ifndef T_PLANNER_H_
#define T_PLANNER_H_

struct TPlanData {
    double a_m;
    double v_m;
    double S;
    double T;
    double Ta;
    double sign;
    bool tri_or_trap; // true for tri, false for trap
};

class TPlanner {
    // 适用条件：零始末速度

public:
    TPlanner(double a_, double v_);
    TPlanner(double a_, double v_, double* Ss_);
    ~TPlanner() { }
    void update(double* xyz_pos); // update new displacement
    long getPlanTime() { return (long)(T_opt * 1000); }
    long getPlanTime(double* xyz_pos); 
    double getXCurve(long count) { return getTrapCurve(curves[0], count); }
    double getYCurve(long count) { return getTrapCurve(curves[1], count); }
    double getZCurve(long count) { return getTrapCurve(curves[2], count); }

private:
    double A_max;
    double V_max;
    double T_opt; // optimal and synchronized total time
    TPlanData curves[3]; // XYZ curves

    void setVm(double v_);
    void setAm(double a_);
    void setS(double* Ss_);
    void calcOptTimeParamAll();
    void calcGivenTimeParam(TPlanData& c, double time);
    double maximum(double a, double b, double c);
    double getTrapCurve(TPlanData& c, long count);
};

#endif
// T_PLANNER_H_
