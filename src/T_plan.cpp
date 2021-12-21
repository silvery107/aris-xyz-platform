#include <algorithm>
#include <cmath>

struct TPlanData {
    double a_m;
    double v_m;
    double S;
    double T;
    double Ta;
    bool tri_or_trap; // true for tri, false for trap
};

class TPlan {
private:
    double A_max;
    double V_max;
    long T_opt; // optimal and synchronized total time
    TPlanData curves[3]; // XYZ curves
    void setVm(double v_)
    {
        for (auto& curve : curves) {
            curve.v_m = v_;
        }
    }

    void setAm(double a_)
    {
        for (auto& curve : curves) {
            curve.a_m = a_;
        }
    }

    void setS(double* Ss_)
    {
        for (int i = 0; i < 3; i++) {
            curves[i].S = Ss_[i];
        }
    }

    void calcOptTimeParamAll()
    {
        for (auto& c : curves) {
            if (c.S == 0)
                continue;

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

    void calcGivenTimeParam(TPlanData& c, double time)
    {
        c.T = time;
        double v1 = 0.5 * (c.a_m * c.T + sqrt(c.a_m * c.a_m * c.T * c.T - 4 * c.a_m * c.S));
        double v2 = 0.5 * (c.a_m * c.T - sqrt(c.a_m * c.a_m * c.T * c.T - 4 * c.a_m * c.S));
        if (v1 > 0 && v2 > 0)
            c.v_m = v1 < v2 ? v1 : v2;
        else if (v1 < 0 && v2 > 0)
            c.v_m = v2;
        else // v1>0 && v2<0
            c.v_m = v1;

        c.Ta = c.v_m / c.a_m;
    }

    long maximum(double a, double b, double c)
    {
        double max = (a < b) ? b : a;
        return (long)((max < c) ? c : max);
    }

    double getTrapCurve(TPlanData& c, int count)
    {
        double s;
        int t = (count + 1) / 1000.0; // ms to s
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
        return s;
    }

public:
    long getTime()
    {
        return T_opt * 1000;
    }

    TPlan(double a_, double v_, double* Ss_)
    {
        this->A_max = a_;
        this->V_max = v_;

        setAm(A_max);
        setVm(V_max);
        setS(Ss_);
        calcOptTimeParamAll();

        this->T_opt = maximum(curves[0].T, curves[1].T, curves[2].T);

        if (curves[0].T < T_opt) {
            calcGivenTimeParam(curves[0], T_opt);
        }

        if (curves[1].T < T_opt) {
            calcGivenTimeParam(curves[1], T_opt);
        }

        if (curves[2].T < T_opt) {
            calcGivenTimeParam(curves[2], T_opt);
        }
    }

    double getXCurve(int count)
    {
        return getTrapCurve(curves[0], count);
    }

    double getYCurve(int count)
    {
        return getTrapCurve(curves[1], count);
    }

    double getZCurve(int count)
    {
        return getTrapCurve(curves[2], count);
    }

    ~TPlan() { }
};