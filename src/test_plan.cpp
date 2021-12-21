#include "T_planner.cpp"
#include "plan.cpp"
#include <iostream>

int main()
{
    double A_M = 10;
    double V_M = 5;
    double xyz_pos[3] = { 1, 0, 0 };

    TPlanner test = TPlanner(A_M, V_M, xyz_pos);
    long totaltime = test.getPlanTime();
    double s = test.getXCurve(3500);

    std::cout << totaltime << "\n"
              << s << std::endl;

    TCurve test_old = TCurve(A_M, V_M);
    test_old.getCurveParam();
    totaltime = test_old.getTc()*1000;
    s = test_old.getTCurve(900);


    std::cout << totaltime << "\n"
              << s*xyz_pos[0] << std::endl;

    return 1;
}
