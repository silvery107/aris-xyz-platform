import math

class TPlanData:
    def __init__(self) -> None:
        self.a_m = 0.0
        self.v_m = 0.0
        self.S = 0.0
        self.T = 0.0
        self.Ta = 0.0
        self.sign = 0.0
        self.tri_or_trap = False


class TPlanner:
    def __init__(self, a_: float, v_: float, Ss_: list) -> None:
        self.A_max = a_
        self.V_max = v_
        self.curves = [TPlanData() for _ in range(3)]
        self.setAm(self.A_max)
        self.setVm(self.V_max)
        self.setS(Ss_)
        self.calcOptTimeParamAll()
        self.T_opt = self.maximum(self.curves[0].T, self.curves[1].T, self.curves[2].T)

        for i in range(3):
            temp = self.curves[i].T
            if temp < self.T_opt and temp != 0.0:
                self.calcGivenTimeParam(self.curves[i], self.T_opt)

    def getPlanTime(self):
        return int(self.T_opt*1000)

    def setVm(self, v_: float):
        for c in self.curves:
            c.v_m = v_

    def setAm(self, a_: float):
        for c in self.curves:
            c.a_m = a_

    def setS(self, Ss_: list):
        for i in range(3):
            if Ss_[i] < 0:
                self.curves[i].sign = -1
            else:
                self.curves[i].sign = +1

            self.curves[i].S = math.fabs(Ss_[i])

    def calcOptTimeParamAll(self):
        for c in self.curves:
            if c.S == 0.0:
                c.Ta = 0
                c.T = 0
                continue

            c.tri_or_trap = (c.v_m * c.v_m / c.a_m >= c.S)

            if c.tri_or_trap:
                c.v_m = math.sqrt(c.S * c.a_m)
                c.Ta = c.v_m / c.a_m
                c.T = 2 * c.Ta
            else:
                c.Ta = c.v_m / c.a_m
                c.T = c.v_m / c.a_m + c.S / c.v_m

    def calcGivenTimeParam(self, c: TPlanData, time: float):
        c.T = time
        v1 = 0.5 * (c.a_m * c.T +
                    math.sqrt(c.a_m * c.a_m * c.T * c.T - 4.0 * c.a_m * c.S))
        v2 = 0.5 * (c.a_m * c.T -
                    math.sqrt(c.a_m * c.a_m * c.T * c.T - 4.0 * c.a_m * c.S))
        if v1 >= 0 and v2 >= 0:
            c.v_m = v1 if v1 < v2 else v2
        elif v1 < 0 and v2 > 0:
            c.v_m = v2
        else:  # v1>0 and v2<0
            c.v_m = v1

        c.Ta = c.v_m / c.a_m
        c.tri_or_trap = (c.v_m * c.v_m / c.a_m >= c.S)

    def maximum(self, a: float, b: float, c: float):
        max = b if a < b else a
        return c if max < c else max
    
    def getTrapCurve(self, c:TPlanData, count:int):
        s = 0.0
        t = (count + 1) / 1000.0
        if t > c.T:
            t = c.T
            
        if c.tri_or_trap:
            if t < c.Ta:
                s = 0.5 * c.a_m * t * t
            else:
                s = 0.5 * c.a_m * c.Ta * c.Ta + 0.5 * (t - c.Ta) * (2 * c.v_m - c.a_m * (t - c.Ta));
        else:
            if t < c.Ta:
                s = 0.5 * c.a_m * t * t
            elif t >= c.Ta and t < (c.T - c.Ta):
                s = 0.5 * c.a_m * c.Ta * c.Ta + c.v_m * (t - c.Ta)
            else:
                s = 0.5 * c.a_m * c.Ta * c.Ta + c.v_m * (c.T - 2 * c.Ta) + 0.5 * (t - (c.T - c.Ta)) * (2 * c.v_m - c.a_m * (t - (c.T - c.Ta)))
        
        return s*c.sign

    def getXCurve(self, count:int):
        return self.getTrapCurve(self.curves[0], count)
        
    def getYCurve(self, count:int):
        return self.getTrapCurve(self.curves[1], count)

    def getZCurve(self, count:int):
        return self.getTrapCurve(self.curves[2], count)

if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as plt
    A_M = 50
    V_M = 100
    xyz_pos = [10, 5, 0]

    test = TPlanner(A_M, V_M, xyz_pos)
    totaletime = test.getPlanTime()
    # s = test.getXCurve(totaletime)

    print("totale time:", totaletime)

    x_curve = [test.getXCurve(count) for count in range(0, totaletime)]
    y_curve = [test.getYCurve(count) for count in range(0, totaletime)]
    plt.subplot(121)
    plt.plot(x_curve)
    plt.plot(y_curve)
    plt.xlabel("Position")
    plt.ylabel("Time ms")
    plt.legend(["X curve", "Y curve"])
    plt.grid()
    # plt.show()

    vel_curves = []
    for c in test.curves[:2]:
        length = int(c.T*1000)
        ta_len = int(c.Ta*1000)
        curve = np.zeros((length,), dtype=np.float32)

        curve[:ta_len] = np.array([c.a_m*count/1000.0 for count in range(0, ta_len)])
        curve[ta_len: length-ta_len] = c.v_m
        curve[length-ta_len:] = np.array([c.v_m-c.a_m*count/1000.0 for count in range(0, ta_len)])
        vel_curves.append(curve)
        print("max Vel:", c.v_m)

    plt.subplot(122)
    plt.plot(vel_curves[0])
    plt.plot(vel_curves[1])
    plt.ylabel("Velocity")
    plt.xlabel("Time ms")
    plt.legend(["X curve", "Y curve"])
    plt.grid()

    plt.show()