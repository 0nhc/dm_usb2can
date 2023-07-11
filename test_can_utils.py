from can_utils import *
import math

can = CAN(1)
while 1:
    can.SendControlPara(f_p, f_v, f_kp, f_kd, f_t)
