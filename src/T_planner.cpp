#include "T_planner.h"
#include <algorithm>
#include <cmath>

TPlanner::TPlanner(double a_, double v_, double* Ss_)
{
    this->A_max = a_;
    this->V_max = v_;

    setAm(A_max);
    setVm(V_max);
    setS(Ss_);
    calcOptTimeParamAll();

    this->T_opt = maximum(curves[0].T, curves[1].T, curves[2].T);

    for (int i = 0; i < 3; i++) {
        double temp = curves[i].T;
        if (temp < T_opt && temp != 0.0) {
            calcGivenTimeParam(curves[i], T_opt);
        }
    }
}

void TPlanner::setVm(double v_)
{
    for (auto& curve : curves) {
        curve.v_m = v_;
    }
}

void TPlanner::setAm(double a_)
{
    for (auto& curve : curves) {
        curve.a_m = a_;
    }
}

void TPlanner::setS(double* Ss_)
{
    for (int i = 0; i < 3; i++) {
        if (Ss_[i] < 0)
            curves[i].sign = -1;
        else
            curves[i].sign = +1;

        curves[i].S = fabs(Ss_[i]);
    }
}

void TPlanner::calcOptTimeParamAll()
{
    for (auto& c : curves) {
        if (c.S == 0.0) {
            c.Ta = 0;
            c.T = 0;
            continue;
        }

        c.tri_or_trap = (c.v_m * c.v_m / c.a_m >= c.S);

        if (c.tri_or_trap) {
            c.v_m = sqrt(c.S * c.a_m);
            c.Ta = c.v_m / c.a_m;
            c.T = 2 * c.Ta;
        } else {
            c.Ta = c.v_m / c.a_m;
            c.T = c.v_m / c.a_m + c.S / c.v_m;
        }
    }
}

void TPlanner::calcGivenTimeParam(TPlanData& c, double time)
{
    c.T = time;
    double v1 = 0.5 * (c.a_m * c.T + sqrt(c.a_m * c.a_m * c.T * c.T - 4.0 * c.a_m * c.S));
    double v2 = 0.5 * (c.a_m * c.T - sqrt(c.a_m * c.a_m * c.T * c.T - 4.0 * c.a_m * c.S));
    if (v1 >= 0 && v2 >= 0)
        c.v_m = v1 < v2 ? v1 : v2;
    else if (v1 < 0 && v2 > 0)
        c.v_m = v2;
    else // v1>0 && v2<0
        c.v_m = v1;

    c.Ta = c.v_m / c.a_m;
    c.tri_or_trap = (c.v_m * c.v_m / c.a_m >= c.S);
}

double TPlanner::maximum(double a, double b, double c)
{
    double max = (a < b) ? b : a;
    return (max < c) ? c : max;
}

double TPlanner::getTrapCurve(TPlanData& c, long count)
{
    if (count > c.T * 1000.0) {
        count = c.T * 1000.0;
    }

    double s;
    double t = (count + 1) / 1000.0; // ms to s
    if (c.tri_or_trap) {
        if (t < c.Ta)
            s = 0.5 * c.a_m * t * t;
        else
            s = 0.5 * c.a_m * c.Ta * c.Ta + 0.5 * (t - c.Ta) * (2 * c.v_m - c.a_m * (t - c.Ta));

    } else {
        if (t < c.Ta)
            s = 0.5 * c.a_m * t * t;
        else if (t >= c.Ta && t < (c.T - c.Ta))
            s = 0.5 * c.a_m * c.Ta * c.Ta + c.v_m * (t - c.Ta);
        else
            s = 0.5 * c.a_m * c.Ta * c.Ta + c.v_m * (c.T - 2 * c.Ta) + 0.5 * (t - (c.T - c.Ta)) * (2 * c.v_m - c.a_m * (t - (c.T - c.Ta)));
    }
    return s * c.sign;
}
