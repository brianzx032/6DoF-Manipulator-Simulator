#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
# smart-shaking strategy trigger for hiwin manipulator RA605 (need connection)
# =======================================================================
# =15/07/2019:add step select, write out position adjust result         =
# =13/07/2019:add simulation                                            =
# =12/07/2019:import most functions instead of define                   =
# =           updated control package; add position adjust              =
# =           incorporate move_test, move_drink, coordinate_collation   =
# =======================================================================
import datetime
import enum
import os
import threading
import tkinter as tk
from time import sleep, time

import numpy as np
import rospy
import speech_recognition as sr
from ROS_Socket.msg import *
from ROS_Socket.srv import *
from std_msgs.msg import Int32MultiArray, String

import Hiwin_RT605_Socket as ArmTask
import pos_change_interface as pos_inter
import shake_strategy_content as shake_cont

##----Arm state-----------
Arm_state_flag = 0
Sent_data_flag = 1
t=time()
##----Arm status enum
class Arm_status(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
def callback(state):
    global Arm_state_flag,Sent_data_flag,t
    # print('get callback!\t{:.3f}\ts'.format(time()-t))
    t=time()
    Arm_state_flag = state.data[0]
    Sent_data_flag = state.data[1]
def arm_state_listener():
    rospy.Subscriber("chatter", Int32MultiArray, callback)


###-------------------------strategy---------------------
##-------move control part------------- 
wait_for_enter_flag = False
adjust_pos_flag = False
to_change_flag = False
Target_Pos=shake_cont.Point_cls(shake_cont.Current_Pos.x,shake_cont.Current_Pos.y,shake_cont.Current_Pos.z,
            shake_cont.Current_Pos.pitch,shake_cont.Current_Pos.roll,shake_cont.Current_Pos.yaw)
def RefreshTargetPos():
    global Sent_data_flag,Arm_state_flag,Target_Pos
    shake_cont.Hiwin_Move_Log.WriteOut(shake_cont.Pos_diff)
    for var in range(6):
        Target_Pos.ChangeSelf(var,shake_cont.Pos_diff[var])
    ArmTask.point_data(Target_Pos.x,Target_Pos.y,Target_Pos.z,Target_Pos.pitch,Target_Pos.roll,Target_Pos.yaw)
    ArmTask.Arm_Mode(2,1,shake_cont.gp_stop,shake_cont.speed_value,2)#action,ra,grip,vel,both
    print('current position: ',[Target_Pos.x,Target_Pos.y,Target_Pos.z,Target_Pos.pitch,Target_Pos.roll,Target_Pos.yaw])
    shake_cont.Pos_diff=[float(0) for i in range(6)]
    while 1:
        if Sent_data_flag and Arm_state_flag == Arm_status.Idle:
            break
def Adjust_Pos(string):
    global to_change_flag,Target_Pos
    Target_Pos=shake_cont.Point_cls(shake_cont.Current_Pos.x,shake_cont.Current_Pos.y,shake_cont.Current_Pos.z,
        shake_cont.Current_Pos.pitch,shake_cont.Current_Pos.roll,shake_cont.Current_Pos.yaw)
    # pos_inter.PosChange()
    pos_change_thread=threading.Thread(target=pos_inter.PosChange)
    pos_change_thread.start()
    while not shake_cont.pos_ok_flag:
        print('Waiting for OK to move to next point...')
    pos_change_thread.join()
    shake_cont.Adjust_Log.WriteOut(string+':'+str(Target_Pos.x)+','+str(Target_Pos.y)+','+str(Target_Pos.z)+','+str(Target_Pos.pitch)+','+str(Target_Pos.roll)+','+str(Target_Pos.yaw))
def Hiwin_Move(Position,speed,grip,string):
    global Sent_data_flag,Arm_state_flag,wait_for_enter_flag,t
    if wait_for_enter_flag:
        # input("下一步按Enter")
        tk.messagebox.showinfo(message='Next Point?')
    ArmTask.Speed_Mode(shake_cont.arm_speed_mode)
    ArmTask.Arm_Mode(4,1,grip,speed,2)#action,ra,grip,vel,both
    ArmTask.point_data(Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw)
    ArmTask.Arm_Mode(shake_cont.move_mode,1,grip,speed,2)#action,ra,grip,vel,both
    print(shake_cont.move_mode)
    shake_cont.Hiwin_Move_Log.WriteOut(string+':')
    shake_cont.Hiwin_Move_Log.WriteOut('to position: '+str([Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw]))
    if grip==shake_cont.gp_stop:
        sleep(0.75)
    else:
        sleep(0.8)
    while 1:
        if Sent_data_flag == 1 and Arm_state_flag == Arm_status.Idle:
            # print('arm idle\t{:.3f}\ts'.format(time()-t))
            t=time()
            break
    if adjust_pos_flag:
        Adjust_Pos(string)
Hiwin_Action=shake_cont.Action_cls(Hiwin_Move)
Hiwin_Shake_Strategy=shake_cont.Shake_Strategy_cls(1,2,1,~wait_for_enter_flag)
def Shake():
    global wait_for_enter_flag,adjust_pos_flag
    if Hiwin_Shake_Strategy.CheckCmd():
        shake_cont.move_str=shake_cont.cmd_str
        shake_cont.Hiwin_Move_Log.StartWrite('Real Action')
        shake_cont.Hiwin_Move_Log.WriteDate()
        Hiwin_Shake_Strategy.SmartShaking(Hiwin_Action)
        tk.messagebox.showinfo(title='Finished!',message='Time used:\n  {:.0f} m {:.2f} s'.format((time()-shake_cont.start_time)//60,(time()-shake_cont.start_time)%60))
        shake_cont.Home(Hiwin_Action)
    wait_for_enter_flag=False
    adjust_pos_flag=False
##-------move control part end------------- 

##-------speech recgnition part------------- 
def SpeechTriggerShaking():
    global wait_for_enter_flag,adjust_pos_flag
    while True:
            # Record Audio
        r = sr.Recognizer()
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.5)
            print("Say something!")
            audio = r.listen(source)
            # Speech recognition using Google Speech Recognition
        try:
            shake_cont.cmd_str = r.recognize_google(audio,language = 'zh-TW')
            shake_cont.move_str=shake_cont.cmd_str
            if Hiwin_Shake_Strategy.CheckCmd():
                break
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
    shake_cont.Hiwin_Move_Log.StartWrite('')
    shake_cont.Hiwin_Move_Log.WriteDate()
    Hiwin_Shake_Strategy.SmartShaking(Hiwin_Action)
    tk.messagebox.showinfo(title='Finished!',message='Time used:\n  {:.0f} m {:.2f} s'.format((time()-shake_cont.start_time)//60,(time()-shake_cont.start_time)%60))
    shake_cont.Home(Hiwin_Action)
    wait_for_enter_flag=False
    adjust_pos_flag=False
                
##-------speech recgnition part end------------- 
def DefaultShaking():
    global wait_for_enter_flag
    shake_cont.cmd_str='冬瓜紅茶半糖全冰'
    wait_for_enter_flag=False
    Hiwin_Shake_Strategy.delay_flag=~wait_for_enter_flag
    Shake()
class Solo_Action_cls(shake_cont.Action_cls):
    def __init__(self,action):
        self.action=action
    def ArmMove(self,Position,speed,grip,string):
        shake_cont.move_mode=2
        shake_cont.move_str=string
        shake_cont.Current_Pos=shake_cont.Point_cls(Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw)
        self.action.MoveFunc(Position,speed,grip,string)
        sleep(0.5)
    def GripCtrl(self,Position,speed,gp_ctrl,move_str,grip_str):
        self.ArmMove(Position,speed,shake_cont.gp_stop,move_str) #move
        shake_cont.Current_grip=shake_cont.grip_state_str.split()[gp_ctrl]
        self.ArmMove(Position,speed,gp_ctrl,grip_str) #grip
        # sleep()
        if shake_cont.grip_delay:
            sleep(0.5)
Hiwin_Solo_Action=Solo_Action_cls(Hiwin_Action)
def CoordinateCollation():
    while 1:
        input("左下定位")
        shake_cont.Left(Hiwin_Action)
        input("右上定位")
        shake_cont.Right(Hiwin_Action)
        if input('OK?')=='1':
            shake_cont.Home(Hiwin_Action)
            break
def PosAdjust():
    global adjust_pos_flag
    adjust_pos_flag=True
    shake_cont.Adjust_Log.WriteDate()
    Shake()
###-------------strategy end-----------------


if __name__ == '__main__':
    argv = rospy.myargv()
    rospy.init_node('strategy', anonymous=True)
    GetInfoFlag = True #Test no data
    arm_state_listener()
    while 1:
        start_cmd_str=int(input('請輸入指令:\n\t開始錄音:\t1\n\t默認飲料:\t2\n\t坐標測試:\t3\n\t坐標校正:\t4\n\t默認模擬:\t5\n\t離開:\t\tEnter\n')) #輸入開始指令
        shake_cont.speed_value=int(input('速度設定:'))
        print('輸入要跳過的步驟:\n1: 加冰塊\n2: 加飲料\n3: 加果糖\n4: 蓋大蓋\n5: 搖飲\n6: 取小蓋\n7: 倒入手搖杯')
        skip_step_flag=[int(i) for i in input().split()]
        for step in skip_step_flag:
            shake_cont.step_flag[step-1]=False
        if start_cmd_str == 1 :
            SpeechTriggerShaking()
        elif start_cmd_str == 2 :
            DefaultShaking()
        elif start_cmd_str == 3 :
            shake_cont.cmd_str='冬瓜紅茶半糖少冰'
            PosAdjust()
        # coordinate collation
        elif start_cmd_str == 4:
            CoordinateCollation()
        else:
            break
    
##--------------shut dowm-------------     
    shake_cont.byebye()
    ArmTask.rospy.spin()
    rospy.spin()
