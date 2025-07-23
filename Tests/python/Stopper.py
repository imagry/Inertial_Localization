import math
import numpy as np

class FifthOrderProfile:
    def __init__(self, clock, distance):
        self.starting_clock = clock
        self.distance = distance
        self.profile_quintic = np.array([0, 0, 0, 0, 0, 0], dtype=float)

    def pos(self, t):
        a = self.profile_quintic
        return a[0] + a[1] * t + a[2] * (t ** 2) + a[3] * (t ** 3) + a[4] * (t ** 4) + a[5] * (t ** 5)

    def vel(self, t):
        a = self.profile_quintic
        return a[1] + 2 * a[2] * t + 3 * a[3] * (t ** 2) + 4 * a[4] * t**3 + 5 * a[5] * t**4

    def acc(self, t):
        a = self.profile_quintic
        return 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t**2 + 20 * a[5] * t**3

    def jerk(a, t):
        return 6 * a[3] + 24 * a[4] * t + 60 * a[5] * t**2

    '''
    Find fifth order polynomial P(t) which satifies the following:
        P(0) = p0
        P'(0) = dp0
        P''(0) = ddp0
        P(T) = p1
        P'(T) = dp1
        P''(T) = ddp1
    '''
    def solve_quintic(T, p0, dp0, ddp0, p1, dp1, ddp1):
        boundary_c = np.array([[1, 0,   0,     0,      0,       0],
                            [0, 1,   0,     0,      0,       0],
                            [0, 0,   2,     0,      0,       0],
                            [1, T, T*T,  T**3,   T**4,    T**5],
                            [0, 1, 2*T, 3*T*T, 4*T**3,  5*T**4],
                            [0, 0,   2,   6*T, 12*T*T, 20*T**3]], dtype=float)
        
        B = np.array([p0, dp0, ddp0, p1, dp1, ddp1], dtype=float)
        
        a = np.linalg.inv(boundary_c) @ np.transpose(B)
        return a

    '''
    Iteratively find T which satisfies P```(T) = 0
    Return value: {T, P} or T=-1 in case of failure
    '''


def converge_to_zero_jerk(T1, T2, p0, dp0, ddp0, p1, dp1, ddp1):
    t_low = T1
    q_low = FifthOrderProfile.solve_quintic(t_low, p0, dp0, ddp0, p1, dp1, ddp1)
    j_low = FifthOrderProfile.jerk(q_low, t_low)
    # print("time: {}, jerk: {}".format(T1, j_low))
    # print(q_low)
    t_high = T2
    q_high = FifthOrderProfile.solve_quintic(t_high, p0, dp0, ddp0, p1, dp1, ddp1)
    j_high = FifthOrderProfile.jerk(q_high, t_high)
    # print("time: {}, jerk: {}".format(T2, j_high))
    # print(q_high)
    
    for i in range(20):
        t_med = (t_low + t_high) / 2
        q_med = FifthOrderProfile.solve_quintic(t_med, p0, dp0, ddp0, p1, dp1, ddp1)
        j_med = FifthOrderProfile.jerk(q_med, t_med)
        # print("time: {}, jerk: {}".format(t_med, j_med))
        # print(q_med)
        if j_med * j_low > 0:
            j_low = j_med
            t_low = t_med
        else:
            j_high = j_med
            t_high = t_med

        if abs(j_med) < 0.01:
            break

    return t_med


class FifthOrderStopper:
    def __init__(self, clock, distance, state_0, max_accel):
        self.starting_clock = clock
        self.distance = distance

        if abs(state_0[0]) < 0.25 and abs(state_0[1]) < 0.25:
            print("Gliding")
            self.profile_quintic, self.total = FifthOrderStopper.build_glider(distance, max_accel)
            self.reverse_quintic = False
            return

        self.profile_quintic, self.total = self.solve_high_quintic(distance, state_0[0], -state_0[1])
        self.reverse_quintic = True

    # Find polynom P(t) = b1*t^5 + b0*t^4 which satifies the following:
    #    P(T) = A
    #    P'(T) = B
    #    P''(T) = C
    # Find b0, b1, T for given A, B, C
    def solve_high_quintic(self, A, B, C):
        # System of eqns:
        #    b1*T^5 + b0*T^4 = A
        #    5*b1*T^4 + 4*b0*T^3 = B
        #    20*b1*T^3 + 12*b0*T^2 = C
        #
        # Substitute a0 = b0*T^2, a1 = b1*T^3:
        #    a1*T^2 + a0*T^2 = A
        #    5*a1*T + 4*a0*T = B
        #    20*a1 + 12*a0 = C
    
        Dis = 4*B*B - 5*A*C
        # print("Dis", Dis)
        if (Dis <0):
            # Relax deceleration, repeat
            C1 = 4*B*B / (5*A) - 0.01
            # print("C, C1", C, C1)
            return self.solve_high_quintic(A, B, C1)
            # T = 4*B / C
        elif C != 0:
            T1 = (4*B + 2*math.sqrt(Dis)) / C
            T2 = (4*B - 2*math.sqrt(Dis)) / C
            # print(T1, T2)
            if T1 > 0 and T2 > 0:
                T = min(T1, T2)
            elif T1 > 0:
                T = T1
            else:
                T = T2
        else:
            T = 5*A / (2*B)
            # print(T)
    
        a0 = B/T - C/4
        a1 = C/5 - 0.6*B/T
    
        b0 = a0/(T*T)
        b1 = a1/(T**3)
        
        b = np.array([0, 0, 0, 0, b0, b1], dtype=float)
        return b, T

    def build_glider(distance, max_accel):
        T = math.sqrt((10 * math.sqrt(3) * distance) / (3 * max_accel))

        a3 = 10 * distance / (T * T * T)
        a4 = -15 * distance / (T * T * T * T)
        a5 = 6 * distance / (T * T * T * T * T)

        a = np.array([0, 0, 0, a3, a4, a5], dtype=float)
        return a, T

    def pos(a, t):
        return a[0] + a[1] * t + a[2] * (t ** 2) + a[3] * (t ** 3) + a[4] * (t ** 4) + a[5] * (t ** 5)
    
    def vel(a, t):
        return a[1] + 2 * a[2] * t + 3 * a[3] * (t ** 2) + 4 * a[4] * t**3 + 5 * a[5] * t**4
    
    def acc(a, t):
        return 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t**2 + 20 * a[5] * t**3
    
    def jerk(a, t):
        return 6 * a[3] + 24 * a[4] * t + 60 * a[5] * t**2
    
    def sample(self, t_in):
        t = t_in - self.starting_clock
        if t < self.total:
            if self.reverse_quintic:
                return (FifthOrderStopper.vel(self.profile_quintic, self.total-t),
                        -FifthOrderStopper.acc(self.profile_quintic, self.total-t))
            else:
                return (FifthOrderStopper.vel(self.profile_quintic, t),
                        FifthOrderStopper.acc(self.profile_quintic, t))
        else:
            return (0, 0)
