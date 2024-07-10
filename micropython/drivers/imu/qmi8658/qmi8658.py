# QMI8658 driver for MicroPython
# MIT license; Copyright (c) 2022-2024 WEMOS.CC

import struct
from machine import I2C
import machine
import time
import array
import math

WHO_AM_I = const(0x00)  # Device identifier
REVISION_ID = const(0x01)
CTRL1 = const(0x02)  # Serial Interface and Sensor Enable
CTRL2 = const(0x03)  # Accelerometer Settings
CTRL3 = const(0x04)  # Gyroscope Settings
CTRL4 = const(0x05)  # Magnetometer Settings
CTRL5 = const(0x06)  # Sensor Data Processing Settings
CTRL7 = const(0x08)  # Enable Sensors and Configure Data Reads
CTRL8 = const(0x09)  # Reserved – Special Settings

# <Sensor Data Output Registers>

TEMP_L = const(0x33)

AccX_L = const(0x35)
AccX_H = const(0x36)
AccY_L = const(0x37)
AccY_H = const(0x38)
AccZ_L = const(0x39)
AccZ_H = const(0x3A)

GyrX_L = const(0x3B)
GyrX_H = const(0x3C)
GyrY_L = const(0x3D)
GyrY_H = const(0x3E)
GyrZ_L = const(0x3F)
GyrZ_H = const(0x40)


class QMI8658():
    def __init__(self, i2c, addr=0x6b, accel_scale=2, gyro_scale=2048,accel_odr=500,gyro_odr=500):
        self.i2c = i2c
        self.addr = addr

        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        
        self.accel_odr=accel_odr
        self.gyro_odr=gyro_odr

        self.accel_sensitivity=32768/self.accel_scale
        self.gyro_sensitivity=32768/gyro_scale
        
        self.scratch_int = array.array("h", [0, 0, 0])

        if self.i2c.readfrom_mem(self.addr, WHO_AM_I, 1) != b'\x05':
            raise OSError(
                "No QMI8658 device was found at address 0x%x" % (self.addr))

        # Accelerometer Full-scale
        SCALE_ACCEL = {2: 0, 4: 1, 8: 2, 16: 3}

        # Gyroscope Full-scale
        SCALE_GYRO = {16: 0, 32: 1, 64: 2, 128: 3,
                      256: 4, 512: 5, 1024: 6, 2048: 7}

        # Accelerometer Output Data Rate (ODR)
        ODR_ACCEL = {
            # Normal
            8000: 0,
            4000: 1,
            2000: 2,
            1000: 3,
            500: 4,
            250: 5,
            125: 6,
            62.5: 7,
            31.25: 8,
            # Low Power
            128: 12,
            21: 13,  # 58% Duty Cycle
            11: 14,  # 31% Duty Cycle
            3: 15  # 8.5% Duty Cycle
        }

        # Gyroscope Output Data Rate (ODR)
        ODR_GYRO = {
            # Normal
            8000: 0,
            4000: 1,
            2000: 2,
            1000: 3,
            500: 4,
            250: 5,
            125: 6,
            62.5: 7,
            31.25: 8,
        }

        # Sanity checks

        if not self.accel_scale in SCALE_ACCEL:
            raise ValueError("Invalid accelerometer scaling: %d" %
                             self.accel_scale)

        if not self.gyro_scale in SCALE_GYRO:
            raise ValueError("invalid gyro scaling: %d" % self.gyro_scale)

        if not self.accel_odr in ODR_ACCEL:
            raise ValueError("Invalid sampling rate: %d" % self.accel_odr)
        
        if not self.gyro_odr in ODR_GYRO:
            raise ValueError("Invalid sampling rate: %d" % self.gyro_odr)

        # Serial Interface and Sensor Enable, address auto increment
        self.i2c.writeto_mem(self.addr, CTRL1, b'\x40')
        # Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>
        self.i2c.writeto_mem(self.addr, CTRL7, b'\x03')

        # Accelerometer Settings
        parm=(ODR_ACCEL[self.accel_odr])|(SCALE_ACCEL[self.accel_scale]<<4)
        self.i2c.writeto_mem(self.addr, CTRL2, bytes([parm]))
        
        # Gyroscope Settings< ±2048dps 500Hz>
        parm=(ODR_GYRO[self.gyro_odr])|(SCALE_GYRO[self.gyro_scale]<<4)
        self.i2c.writeto_mem(self.addr, CTRL3, bytes([parm]))

        # Sensor Data Processing Settings<Enable Gyroscope Accelerometer Low-Pass Filter>
        self.i2c.writeto_mem(self.addr, CTRL5, b'\x11')

    def test(self):
        return self.i2c.readfrom_mem(self.addr, AccX_L, 12)
    
    def read_gyro(self):
        mv = memoryview(self.scratch_int)
        self.i2c.readfrom_mem_into(self.addr,GyrX_L, mv)
        return (mv[0] / self.gyro_sensitivity, mv[1] / self.gyro_sensitivity, mv[2] / self.gyro_sensitivity)
    
    def read_accel(self):
        mv = memoryview(self.scratch_int)
        self.i2c.readfrom_mem_into(self.addr,AccX_L, mv)
        return (mv[0] / self.accel_sensitivity, mv[1] / self.accel_sensitivity, mv[2] / self.accel_sensitivity)

    def read_temp(self):
        return struct.unpack("<h",self.i2c.readfrom_mem(self.addr, TEMP_L, 2))[0]/256

    def invSqrt(self,number):
        threehalfs = 1.5
        x2 = number * 0.5
        y = number
        
        packed_y = struct.pack('f', y)       
        i = struct.unpack('i', packed_y)[0]  # treat float's bytes as int 
        i = 0x5f3759df - (i >> 1)            # arithmetic with magic number
        packed_i = struct.pack('i', i)
        y = struct.unpack('f', packed_i)[0]  # treat int's bytes as float
        
        y = y * (threehalfs - (x2 * y * y))  # Newton's method
        return y
    
    def get_euler_angles(self,gx,gy,gz,ax,ay,az):

        Ki=const(0.008)
        Kp=const(10.0)
        halfT=const(0.002127)

        q0 = 1.0
        q1 = 0.0
        q2 = 0.0
        q3 = 0.0	#/** quaternion of sensor frame relative to auxiliary frame */

        exInt=0.0
        eyInt=0.0
        ezInt=0.0

        q0q0 = q0*q0
        q0q1 = q0*q1
        q0q2 = q0*q2

        q1q1 = q1*q1
        q1q3 = q1*q3

        q2q2 = q2*q2
        q2q3 = q2*q3

        q3q3 = q3*q3


        if( ax*ay*az==0):
            return 0
        #/* 对加速度数据进行归一化处理 */
        recipNorm = self.invSqrt( ax* ax +ay*ay + az*az)
        ax = ax *recipNorm
        ay = ay *recipNorm
        az = az *recipNorm
        #/* DCM矩阵旋转 */
        vx = 2*(q1q3 - q0q2)
        vy = 2*(q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3 
        #/* 在机体坐标系下做向量叉积得到补偿数据 */
        ex = ay*vz - az*vy 
        ey = az*vx - ax*vz 
        ez = ax*vy - ay*vx 
        #/* 对误差进行PI计算，补偿角速度 */
        exInt = exInt + ex * Ki
        eyInt = eyInt + ey * Ki
        ezInt = ezInt + ez * Ki

        gx = gx + Kp*ex + exInt
        gy = gy + Kp*ey + eyInt
        gz = gz + Kp*ez + ezInt
        #/* 按照四元素微分公式进行四元素更新 */
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT

        recipNorm = self.invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)

        q0 = q0*recipNorm
        q1 = q1*recipNorm
        q2 = q2*recipNorm
        q3 = q3*recipNorm

        roll =  math.atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3
        pitch =  math.asin(2*q1*q3 - 2*q0*q2)*57.3
        yaw  =  -math.atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3



        # print("pitch:{:>8.3f} roll:{:>8.3f} yaw:{:>8.3f}\r\n".format(pitch,roll,yaw))
        return pitch,roll,yaw
