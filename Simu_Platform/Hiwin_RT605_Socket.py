#!/usr/bin/env python3
# license removed for brevity
import rospy
import os
import socket
##多執行序
import threading
import time
import sys
import matplotlib as plot
import Hiwin_RT605_Socket_TCPcmd as TCP
import Hiwin_RT605_Socket_Taskcmd as Taskcmd
import numpy as np
from std_msgs.msg import String
from ROS_Socket.srv import *
from ROS_Socket.msg import *
from std_msgs.msg import Int32MultiArray
import math
import enum
from shake_strategy_content import Arm_connected
#Socket = 0
#data = '0' #設定傳輸資料初始值
Arm_feedback = 1 #假設手臂忙碌
NAME = 'socket_server'
arm_mode_flag = False
##------------class pos-------
class point():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
pos = point(0.0,36.8,11.35,-90.0,0.0,0.0)
##------------class socket_cmd---------
class socket_data():
    def __init__(self, grip, setvel, ra, delay, setboth, action,Speedmode):
        self.grip = grip
        self.setvel = setvel
        self.ra = ra
        self.delay = delay
        self.setboth = setboth
        self.action = action
        self.Speedmode = Speedmode
socket_cmd = socket_data(0,0.0,0,0,0,0,0)
##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False
##-----------client feedback arm state----------
class StateFeedback():
    def __init__(self,ArmState,SentFlag):
        self.ArmState = ArmState
        self.SentFlag = SentFlag

state_feedback = StateFeedback(0,0)

class client():
    def __init__(self):
        self.get_connect()

    def get_connect(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(('192.168.0.1', 8080))

    def send(self, msg):
        self.s.send(msg.encode('utf-8'))    #用utf-8來encode，還有其他encode的方法，str用utf-8就OK!

    def get_recieve(self):
        data = self.s.recv(1024)   #1024指定buffer的大小，限制一次收多少
        data.decode('utf-8')
        return data
    def close(self):
        self.s.close()

Socket = client()


def point_data(x,y,z,pitch,roll,yaw): ##接收策略端傳送位姿資料
    pos.x = '%s'%x
    pos.y = '%s'%y
    pos.z = '%s'%z
    pos.pitch = '%s'%pitch
    pos.roll = '%s'%roll
    pos.yaw = '%s'%yaw
##----------Arm Mode-------------###
def Arm_Mode(action,ra,grip,setvel,setboth): ##接收策略端傳送手臂模式資料
    global arm_mode_flag
    socket_cmd.action = int('%s'%action)
    socket_cmd.grip = int('%s'%grip)
    socket_cmd.ra = int('%s'%ra)
    socket_cmd.setvel = int('%s'%setvel)
    socket_cmd.setboth = int('%s'%setboth)
    arm_mode_flag = True
    Socket_command()
##-------Arm Speed Mode------------###
def Speed_Mode(speedmode): ##接收策略端傳送手臂模式資料
    socket_cmd.Speedmode = int('%s'%speedmode)
##-------Arm Speed Mode------------###
def Suction_Mode(suction): ##接收策略端傳送手臂模式資料
    socket_cmd.Suction = int('%s'%suction)
def socket_talker(): ##創建Server node

    pub = rospy.Publisher('chatter', Int32MultiArray, queue_size=10)
    rospy.init_node(NAME)
    rate = rospy.Rate(200) # 10hz
    print ("Ready to connect")
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        state = Int32MultiArray()
        state.data = [state_feedback.ArmState,state_feedback.SentFlag]
        pub.publish(state)
        rate.sleep()

##----------socket 封包傳輸--------------##
 ##---------------socket 傳輸手臂命令-----------------
def Socket_command():
    global Socket
    for case in switch(socket_cmd.action):
        #-------PtP Mode--------
        if case(Taskcmd.Action_Type.PtoP):
            for case in switch(socket_cmd.setboth):
                if case(Taskcmd.Ctrl_Mode.CTRL_POS):
                    data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_POS,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                    break
                if case(Taskcmd.Ctrl_Mode.CTRL_EULER):
                    data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_EULER,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                    break
                if case(Taskcmd.Ctrl_Mode.CTRL_BOTH):
                    data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_BOTH,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                    break
            break
        #-------Line Mode--------
        if case(Taskcmd.Action_Type.Line):
            for case in switch(socket_cmd.setboth):
                if case(Taskcmd.Ctrl_Mode.CTRL_POS):
                    data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_POS,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                    break
                if case(Taskcmd.Ctrl_Mode.CTRL_EULER):
                    data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_EULER,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel )
                    break
                if case(Taskcmd.Ctrl_Mode.CTRL_BOTH):
                    data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_BOTH,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel )
                    break
            break
        #-------設定手臂速度--------
        if case(Taskcmd.Action_Type.SetVel):
            data = TCP.SetVel(socket_cmd.grip, socket_cmd.setvel)
            break
        #-------設定手臂Delay時間--------
        if case(Taskcmd.Action_Type.Delay):
            data = TCP.SetDelay(socket_cmd.grip,0)
            break
        #-------設定手臂急速&安全模式--------
        if case(Taskcmd.Action_Type.Mode):
            data = TCP.Set_SpeedMode(socket_cmd.grip,socket_cmd.Speedmode)
            break
        #------吸盤------------
        if case(Taskcmd.Action_Type.Suction):
            data = TCP.Set_Suction(socket_cmd.grip,socket_cmd.Suction)
            break
    socket_cmd.action= 10 ##切換初始mode狀態
    Socket.send(data)
##-----------socket client--------
def socket_client():
    global Socket
    try:
        print('Connection has been successful')
        Arm_connected=True
    except socket.error as msg:
        print(msg)
        Arm_connected=False
        sys.exit(1)
    Socket_feedback(Socket)
    rospy.on_shutdown(myhook)
    Socket.close()

def Socket_feedback(s):
    Socket = s
    while 1:
        feedback_str = Socket.get_recieve()
        #手臂端傳送手臂狀態
        if str(feedback_str[2]) == '48':# F 手臂為Ready狀態準備接收下一個運動指令
            state_feedback.ArmState = 0
        if str(feedback_str[2]) == '49':# T 手臂為忙碌狀態無法執行下一個運動指令
            state_feedback.ArmState = 1
        if str(feedback_str[2]) == '54':# 6 策略完成
            state_feedback.ArmState = 6
            print("shutdown")
        #確認傳送旗標
        if str(feedback_str[4]) == '48':#回傳0 false
            state_feedback.SentFlag = 0
        if str(feedback_str[4]) == '49':#回傳1 true
            state_feedback.SentFlag = 1
    ##---------------socket 傳輸手臂命令 end-----------------
        if state_feedback.ArmState == Taskcmd.Arm_feedback_Type.shutdown:
            break
##-----------socket client end--------
##-------------socket 封包傳輸 end--------------##

def myhook():
    print ("shutdown time!")

if __name__ == '__main__':
    socket_cmd.action = 10##切換初始mode狀態
    ## 多執行緒
    t = threading.Thread(target=socket_client)
    t.start() # 開啟多執行緒
    #time.sleep(1)
    try:
        socket_talker()
    except rospy.ROSInterruptException:
        pass
    t.join()
    ## 多執行序 end

