#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
# off-line simulation for hiwin manipulator RA605
# ====================================================
# =17/07/2019:finish delt_z tester                   =
# =16/07/2019:add delt_z tester(unfinished)          =
# =15/07/2019:add step select                        =
# =13/07/2019:take out move_simulation               =
# =12/07/2019:import most functions instead of define=
# ====================================================
import tkinter as tk
from time import time

import matplotlib.pyplot as plt
import numpy as np
import speech_recognition as sr

import shake_strategy_content as shake_cont
import kinematics as km
Shake_Simu_Strategy=shake_cont.Shake_Strategy_cls(1,2,1,False)
cmd_right=0
def SpeechRecog():
    global cmd_right
    #Record Audio
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source, duration=0.5)
        print("Say something!")
        audio = r.listen(source)
    # Speech recognition using Google Speech Recognition
    try:
        shake_cont.cmd_str = r.recognize_google(audio,language = 'zh-TW')
        shake_cont.move_str=shake_cont.cmd_str
        # print(shake_cont.cmd_str)
        cmd_right=Shake_Simu_Strategy.CheckCmd()
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
def AnimationSimulation():
    if Shake_Simu_Strategy.CheckCmd():
        shake_cont.move_str=shake_cont.cmd_str
        shake_cont.simu_flag=True
        Shake_Simu_Strategy.SmartShaking(shake_cont.Animation_Action)
        tk.messagebox.showinfo(title='Finished!',message='Time used:\n  {:.0f} m {:.2f} s'.format((time()-shake_cont.start_time)//60,(time()-shake_cont.start_time)%60))
        shake_cont.simu_flag=False
        km.fig_import=False
def WritePos():
    shake_cont.cmd_str='冬瓜多多紅綠茶全糖全冰少冰微冰'
    shake_cont.Write_Pos.WriteDate()
    Write_Pos_Strategy=shake_cont.Shake_Strategy_cls(3,4,1,False)
    Write_Pos_Strategy.CheckCmd()
    Write_Pos_Strategy.SmartShaking(shake_cont.Write_Pos_Action)
def HeightRangeAccessibleTest():
    shake_cont.cmd_str='冬瓜多多紅綠茶全糖全冰少冰微冰'
    shake_cont.Height_Range_Accessible.WriteDate()
    shake_cont.Height_Range_Accessible.WriteOut('-----------------------------------------------------------') 
    for height_inmm in range(-50,101):
        shake_cont.delt_z=height_inmm/10
        shake_cont.Height_Range_Accessible.WriteOut('桌面相對原點高度: {:.1f} cm'.format(-27.5-11+shake_cont.delt_height+shake_cont.delt_z))
        Delt_Z_Test=shake_cont.Shake_Strategy_cls(3,4,1,False)
        Delt_Z_Test.SmartShaking(shake_cont.Height_Range_Action)
        if not shake_cont.Height_Range_Accessible.failed_Position:
            shake_cont.Height_Range_Accessible.WriteOut('All accessible!')
        else:
            shake_cont.Height_Range_Accessible.WriteOut('All Falied Position: ')
            for fail_pos in shake_cont.Height_Range_Accessible.failed_Position:
                shake_cont.Height_Range_Accessible.WriteOut(str([fail_pos.x,fail_pos.y,fail_pos.z+shake_cont.delt_z,fail_pos.pitch,fail_pos.roll,fail_pos.yaw]))
        if height_inmm < 101:    
            shake_cont.Height_Range_Accessible.WriteOut('-----------------------------------------------------------') 
        else:
            shake_cont.Height_Range_Accessible.WriteOut('\n')
        shake_cont.Height_Range_Accessible.failed_Position.clear()
if __name__ == '__main__':
    while 1:  
        start_cmd_str=input('請輸入指令:\n\t開始錄音:\t1\n\t默認飲料:\t2\n\t完整坐標:\t3\n\t測試delt_z:\t4\n\t離開:\t\tEnter\n') #輸入開始指令
        if start_cmd_str=='1':
            SpeechRecog()
            if cmd_right:
                Shake_Simu_Strategy.SmartShaking(shake_cont.Animation_Action)
        elif start_cmd_str=='2':
            shake_cont.cmd_str = '冬瓜紅茶半糖少冰'
            AnimationSimulation()
        elif start_cmd_str=='3':
            WritePos()
        elif start_cmd_str=='4':
            HeightRangeAccessibleTest()
        else:
            shake_cont.byebye()
            break
