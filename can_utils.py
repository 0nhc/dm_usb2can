import numpy as np
from socket import *
import serial
import time

class CAN():
    def __init__(self, MOTOR_ID = 1):
        # 参数
        self.CMD_MOTOR_MODE = 0x01
        self.CMD_RESET_MODE = 0x02
        self.CMD_ZERO_POSITION = 0x03
        self.CMD_START_MODE = 0x04
        self.P_MIN = -4*np.pi
        self.P_MAX = 4*np.pi
        self.V_MIN = -45.0  
        self.V_MAX = 45.0
        self.KP_MIN = 0.0    
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0     
        self.KD_MAX = 5.0
        self.T_MIN = -13.0
        self.T_MAX = 13.0
        self.MOTOR_ID = MOTOR_ID

        # 电机状态变量
        self.state = [0, 0, 0, 0, 0] #位置 速度 力矩 驱动器温度 电机线圈温度

        # 打开串口
        # self.serial=serial.Serial("/dev/dmbot_usb2can",921600)

        # 初始化电机
        self.ControlCmd(self.CMD_MOTOR_MODE)
        self.SendControlPara(0,0,0,0,0)
        time.sleep(1)

    def limit_min_max(self, x, min, max):
        if x<=min:
            x=min
        elif x>max:
            x=max
        return x

    def float_to_uint(self, x, x_min, x_max, bits):
        span=x_max-x_min
        offset=x_min
        return np.uint16((x-offset)*((1<<bits)-1)/span)

    def uint_to_float(self, x_int, x_min, x_max, bits):
        span=x_max-x_min
        offset=x_min
        return  float(x_int)*span/(float(1<<bits)-1)+offset

    def pi2pi(self, rad):
        while(rad > np.pi):
            rad -= 2*np.pi
        while(rad < -np.pi):
            rad += 2*np.pi
        return rad
    
    def ControlCmd(self, cmd):
        send_data=np.array([0x55,0xAA,0x1e,0x01,0x01,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0,0,0,0,0x00,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x88],np.uint8)
        buf=np.array([self.MOTOR_ID,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00],np.uint8)
        if cmd==self.CMD_MOTOR_MODE:
            buf[8]=0xFC
        elif cmd==self.CMD_RESET_MODE:
            buf[8]=0xFD
        elif cmd==self.CMD_ZERO_POSITION:
            buf[8]=0xFE
        else:
            print("等待电机启动命令")
        send_data[13] = self.MOTOR_ID
        send_data[21:29] = buf[1:9]
        # self.serial.write(bytes(send_data.T))

    def SendControlPara(self, f_kp, f_kd, f_p, f_v, f_t):
        send_data=np.array([0x55,0xAA,0x1e,0x01,0x01,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0,0,0,0,0x00,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x88],np.uint8)
        self.limit_min_max(f_p,  self.P_MIN,  self.P_MAX)
        self.limit_min_max(f_v,  self.V_MIN,  self.V_MAX)
        self.limit_min_max(f_kp, self.KP_MIN, self.KP_MAX)
        self.limit_min_max(f_kd, self.KD_MIN, self.KD_MAX)
        self.limit_min_max(f_t,  self.T_MIN,  self.T_MAX)
        p = self.float_to_uint(f_p,      self.P_MIN,  self.P_MAX,  16)            
        v = self.float_to_uint(f_v,      self.V_MIN,  self.V_MAX,  12)
        kp = self.float_to_uint(f_kp,    self.KP_MIN, self.KP_MAX, 12)
        kd = self.float_to_uint(f_kd,    self.KD_MIN, self.KD_MAX, 12)
        t = self.float_to_uint(f_t,      self.T_MIN,  self.T_MAX,  12)
        buf=np.zeros((9,1),np.uint8)
        buf[0] = self.MOTOR_ID
        buf[1] = p>>8
        buf[2] = p&0xFF
        buf[3] = v>>4
        buf[4] = ((v&0xF)<<4)|(kp>>8)
        buf[5] = kp&0xFF
        buf[6] = kd>>4
        buf[7] = ((kd&0xF)<<4)|(t>>8)
        buf[8] = t&0xff
        send_data[13] = self.MOTOR_ID
        send_data[21:29] = buf[1:9,0]
        print(buf)
        # self.serial.write(bytes(send_data.T))

    def get_feedback(self):
        # data = list(self.serial.read(16*1))
        data =[]
        # 接收成功标志位
        if(data[1] == 0x11):
            data = data[7:15]
            POS = (data[2] | (data[1] << 8))
            VEL = ((data[4] & 0xF0) >> 4) | (data[3] << 4)
            T = (data[5] | ((data[4] & 0x0F) << 8))

            _pos = self.uint_to_float(POS, self.P_MIN, self.P_MAX, 16)
            _vel = self.uint_to_float(VEL, self.V_MIN, self.V_MAX, 12)
            _tor = self.uint_to_float(T, self.T_MIN, self.T_MAX, 12)
            _pos = self.pi2pi(_pos)

            _motor_temp = data[7]
            _driver_temp = data[6]
            
            self.state = [_pos, _vel, _tor, _motor_temp, _driver_temp]

    def get_state(self):
        return self.state
    
    def update_motor(self, f_p, f_v, f_kp, f_kd, f_t):
        self.SendControlPara(f_p, f_v, f_kp, f_kd, f_t)
        self.get_feedback()
        return self.state
    
    def calibrate(self):
        self.ControlCmd(self.CMD_ZERO_POSITION)
