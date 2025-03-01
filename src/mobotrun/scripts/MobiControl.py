#!/usr/bin/python3

import json
import os
import numpy as np
import math
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library
from dxl_address import *

class Mobility:
    def __init__(self, port): 
        # Wheel parameters
        self.wheel_Lx = 0.23
        self.wheel_Ly = 0.20
        self.wheel_R = 0.06
        self.wheel_K = 40.92 # gain value to 1023 (max value of moving speed)
        self.eqm = np.array([[1, 1, -(self.wheel_Lx+self.wheel_Ly)], [1, -1, (self.wheel_Lx+self.wheel_Ly)], [1, -1, -(self.wheel_Lx+self.wheel_Ly)], [1, 1, (self.wheel_Lx+self.wheel_Ly)]])

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWriteACC = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_ACCELERATION, LEN_GOAL_ACCELERATION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED)
        self.group_sync_read_speed = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_MX_PRESENT_SPEED, LEN_PRESENT_SPEED)

        #Dynamixel Port check
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        for id in DXL_IDS:

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully set to wheel mode" % id)

        for id in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Torque enable failed for ID {id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Torque enable error for ID {id}: {packet_handler.getRxPacketError(dxl_error)}")

    def GetVelocity(self):
        dxltorpm = 0.114 #0 ~ 1,023 can be used, and the unit is about 0.114rpm

        vel1 = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_MX_PRESENT_SPEED)
        if vel1[0] > 1023:
            velcal1 = (vel1[0]-1023)*-1*dxltorpm*2*3.14/60 #Convert from rpm to rad/s
        else:
            velcal1 = vel1[0]*dxltorpm*2*3.14/60

        vel2 = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_MX_PRESENT_SPEED)
        if vel2[0] > 1023:
            if(vel2[0] == 0):
                velcal2 = 0
            else:
                velcal2 = (vel2[0]-1023)*dxltorpm*2*3.14/60
        else:   
            velcal2 = vel2[0]*-1*dxltorpm*2*3.14/60

        vel3 = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_MX_PRESENT_SPEED)
        if vel3[0] > 1023:
            velcal3 = (vel3[0]-1023)*-1*dxltorpm*2*3.14/60
        else:
            velcal3 = vel3[0]*dxltorpm*2*3.14/60     

        vel4 = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_MX_PRESENT_SPEED)
        if vel4[0] > 1023:
            if(vel4[0] == 0):
                velcal4 = 0
            else:
                velcal4 = (vel4[0]-1023)*dxltorpm*2*3.14/60
        else:
            velcal4 = vel4[0]*-1*dxltorpm*2*3.14/60

        #print(vel1,vel2,vel3,vel4)
        allvel = np.array([[velcal1], [velcal2], [velcal3], [velcal4]]) #valcal1-4 is angular velocity
        tra_mat = np.array([[1, 1, 1, 1],[1, -1, -1, 1],[-1/(self.wheel_Lx+self.wheel_Ly), 1/(self.wheel_Lx+self.wheel_Ly), -1/(self.wheel_Lx+self.wheel_Ly), 1/(self.wheel_Lx+self.wheel_Ly)]])
        return ((self.wheel_R/4)*tra_mat).dot(allvel) #Return Linear velocity

    def GetTempAndLoad(self):
        temp = np.zeros(4)
        load = np.zeros(4)
        for id in range(1, 5):
            temp[id-1] = self.packetHandler.read1ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_TEMPERATURE)[0]
            load[id-1] = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_LOAD)[0]
        status = [temp[0], load[0], temp[1], load[1], temp[2], load[2], temp[3], load[3]]

        return status
        
    def SetWheelVelocity(self, Vx=0.0, Vy=0.0, Wz=0.0, ACC = 30):
        robot_vel = np.array([[Vx], [Vy], [Wz]])
        wheel_vel = ((1/self.wheel_R)*self.eqm).dot((robot_vel))
        wheel_vel = np.absolute(wheel_vel)
        findK = np.max(wheel_vel)

        if findK != 0:
            self.wheel_K = 1023/findK 
        else:
            self.wheel_K = 0

        wheel_vel = ((1/self.wheel_R)*self.eqm).dot((robot_vel))*self.wheel_K*math.sqrt((Vx*Vx)+(Vy*Vy))

        if Vx == 0 and Vy == 0 and Wz != 0:
            wheel_vel = ((1/self.wheel_R)*self.eqm).dot((robot_vel))*10

        W1 = np.clip(int(math.floor(wheel_vel[0])),-1024, 1024) #CW direction: 1024-2046 to move forward
        if(W1 < 0):
            W1 = abs(W1)
        else:
            W1 = W1+1024

        W2 = np.clip(int(math.floor(wheel_vel[1])),-1024, 1024) #CCW direction 0-1023 to move forward 
        if(W2 < 0):
            W2 = abs(W2) +1024
      
        W3 = np.clip(int(math.floor(wheel_vel[2])),-1024, 1024) #CW direction: 1024-2046 to move forward
        if(W3 < 0):
            W3 = abs(W3)
        else:
            W3 = W3+1024
      
        W4 = np.clip(int(math.floor(wheel_vel[3])),-1024, 1024) #CCW direction 0-1023 to move forward 
        if(W4 < 0):
            W4 = abs(W4) +1024

        if Vx == 0 and Vy == 0 and Wz == 0: #Force Stop
            # self.DXLACC = self.DXLACC_stop
            W1 = 0
            W2 = 0
            W3 = 0
            W4 = 0
        # else:
        #     self.DXLACC = self.DXLACC_drive

        out_vel = [W1, W2, W3, W4]
        self.DXLACC = [ACC]*4

        for id, acceleration in zip(DXL_IDS, self.DXLACC):
            param_acceleration = [acceleration]  # Acceleration is 1 byte
            add_result = self.groupSyncWriteACC.addParam(id, param_acceleration)
            if not add_result:
                print(f"Failed to add acceleration parameter for ID {id}")
                exit()

        # Add velocities to GroupSyncWrite
        for id ,velocity in zip(DXL_IDS, out_vel):
            param_velocity = [
                DXL_LOBYTE(velocity),
                DXL_HIBYTE(velocity),
            ]
            add_result = self.groupSyncWrite.addParam(id, param_velocity)
            if not add_result:
                print(f"Failed to add parameter for ID {id}")
                exit()

        # Transmit acceleration settings
        dxl_comm_result = self.groupSyncWriteACC.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"GroupSyncWrite for acceleration failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        # Clear acceleration parameters
        self.groupSyncWriteACC.clearParam()

        # Send synchronized velocities to all motors
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"GroupSyncWrite failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        # Clear parameters from GroupSyncWrite
        self.groupSyncWrite.clearParam()


    def disable_torque(self, Vx=0, Vy=0, Wz=0):
        if Vx == 0 and Vy == 0 and Wz == 0:
            for id in DXL_IDS:
                self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)  
    
    def Shutdown(self):
        # Stop all motors
        for id in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("dxl_comm moving speed error %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("dxl_error moving speed %s" % self.packetHandler.getRxPacketError(dxl_error))

        for id in DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE) 

        self.portHandler.closePort()