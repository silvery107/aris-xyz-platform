#include <cmath>

class TPlan {
private:
    double a_m;
    double v_m;
    double S;
    double T;
    double Ta;
    bool tri_or_trap; // true for tri, false for trap

public:
    long getTc()
    {
        return (long)T;
    }

    double getTCurve(int count)
    {
        double s;
        int t = (count + 1) / 1000.0; // ms to s
        if (tri_or_trap) {
            if (t < Ta)
                s = 0.5 * a_m * t * t;

            else
                s = 0.5 * a_m * Ta * Ta + 0.5 * (t - Ta) * (2 * v_m - a_m * (t - Ta));

        } else {
            if (t < Ta)
                s = 0.5 * a_m * t * t;

            else if (t >= Ta && t < (T - Ta))
                s = 0.5 * a_m * Ta * Ta + v_m * (t - Ta);

            else
                s = 0.5 * a_m * Ta * Ta + v_m * (T - 2 * Ta) + 0.5 * (t - (T - Ta)) * (2 * v_m - a_m * (t - (T - Ta)));
        }
        return s;
    }

    void getCurveParam()
    {
        tri_or_trap = (v_m * v_m / a_m >= S);
        if (tri_or_trap) {
            v_m = sqrt(S * a_m);
            Ta = v_m / a_m;
            T = 2 * Ta;
        } else {
            Ta = v_m / a_m;
            T = v_m / a_m + S / v_m;
        }
    }

    TPlan(double a, double v, double s, double t = -1)
    {
        this->a_m = a;
        this->v_m = v;
        this->S = s;
        this->T = t;

        if (t > 0) {
            double v1 = 0.5 * (a_m * T + sqrt(a_m*a_m * T*T - 4 * a_m * S));
            double v2 = 0.5 * (a_m * T - sqrt(a_m*a_m * T*T - 4 * a_m * S));
            if (v1>0 && v2>0)
                this->v_m = v1<v2 ? v1 : v2;
            else if (v1<0 && v2>0)
                this->v_m = v2;
            else // v1>0 && v2<0
                this->v_m = v1;
            
            this->Ta = v_m / a_m;
        }
    }

    ~TPlan() { }
}