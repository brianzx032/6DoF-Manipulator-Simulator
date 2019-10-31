#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
# smart-shaking strategy for hiwin manipulator RA605 (without connection)
# ==============================================================================
# =26/07/2019:add GetInFit                                                     =  
# =16/07/2019:add InitData                                                     =    
# =15/07/2019:add grip_catch_open, refine pour Positions                       =    
# =13/07/2019:add Position adjustment, MoveSimulation, make simulation a class = 
# =12/07/2019:declare arm control part and stratedy part as classes            =
# ==============================================================================
# ==============================================================================
# =                                Name Rule                                   =    
# =Functions: AaBb()                                                           =    
# =Variables: aa_bb                                                            =    
# =Class:     Aa_Bb = Aa_Bb_cls()                                              =   
# =List:      aa_bb_list()                                                     = 
# =Set:       aa_bb_set()                                                      = 
# =String:    aa_bb_str                                                        = 
# ==============================================================================
import datetime
import os
import tkinter as tk
from time import sleep, time

import numpy as np

import kinematics as km

import matplotlib.pyplot as plt

##------------Point Class-------
class Point_cls():
    def __init__(self,x,y,z,pitch,roll,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
    def ChangeSelf(self,var,val):
        if var == 0:
            self.x+=val
        if var == 1:
            self.y+=val
        if var == 2:
            self.z+=val
        if var == 3:
            self.pitch+=val
        if var == 4:
            self.roll+=val
        if var == 5:
            self.yaw+=val
##------------Position adjustment--------------
Pos_diff=[float(0) for i in range(6)]
##--------------------data-------------------------
#set the Position
Home_Pos = Point_cls(0,36.8,11.35,-90,0,0)
L_Backward_Collate_Pos = Point_cls(-20,40,-23,-90,0,0)
R_Forward_Collate_Pos = Point_cls(34,60,-23,-90,0,0)
shake_y=63.8
Above_Shake_Pos = Point_cls(0,shake_y,-8,-90,0,0)
#---ice---
Above_Ice_Pos = Point_cls(-15,40.3,-9,-90,0,0)
## ice without base
Full_Ice_Pos = Point_cls(-15,40.3,-21,-90,0,0)
Half_Ice_Pos = Point_cls(-15,37.3,-21,-90,0,0)
Little_Ice_Pos = Point_cls(-15,37.3,-19,-90,0,0)
No_Ice_Pos = Point_cls(-15,37.3,-16,-90,0,0) #useless
Above_Dump_Ice_Pos = Point_cls(0,shake_y-7.5,-8,-90,0,0)
Dump_Ice_Pos = Point_cls(0,shake_y-7.5,-16,-90,0,0)
#---drink---
Green_T_Pos = Point_cls(18,58.8,-22,-105,0,-90)
Blace_T_Pos = Point_cls(34,53.8,-22,-105,0,-90)
Duo_Duo_Pos = Point_cls(34,37.8,-22,-105,0,-90)
Dong_Gua_T_Pos = Point_cls(18,42.8,-22,-105,0,-90)
Above_Green_T_Pos = Point_cls(18,58.8,-8,-100,0,-90)
Above_Blace_T_Pos = Point_cls(34,53.8,-8,-100,0,-90)
Above_Duo_Duo_Pos = Point_cls(34,37.8,-8,-100,0,-90)
Above_Dong_Gua_T_Pos = Point_cls(18,42.8,-9,-100,0,-90)
Pour_Drk_Ready_Pos = Point_cls(7,shake_y,-3.5,-110,0,-90)
Pour_Drk_Pour_R_Pos = Point_cls(8.8,shake_y+0.4,-4,-38,0,-90) 
Pour_Drk_Pour_Pos = Point_cls(7.5,shake_y+0.4,-4,-38,0,-90)
#---sugar---
sugar_y=63.8
Back_Sugar_Pos = Point_cls(-16.5,sugar_y-12.5,-5,-90,0,0)
Spin_Sugar_In_Pos = Point_cls(-16.5,sugar_y,-13,-90,0,-108)
Press_Sugar_Ready_Pos = Point_cls(-16.5,sugar_y,-15.5,-90,0,-90)
Press_Sugar_Pos = Point_cls(-16.5,sugar_y,-18.1,-90,0,-90)
Spin_Out_Ready_Pos = Point_cls(-16.5,sugar_y,-13,-90,0,-90)
Spin_Sugar_Out_Pos = Point_cls(-16.5,sugar_y,-13,-90,0,16)
Above_Sugar_Unspined_Pos = Point_cls(-16.5,sugar_y,-13,-90,0,0)
#---lid---
Above_Lid_Pos = Point_cls(0,47.3,-8,-90,0,0)
Lid_Pos = Point_cls(0,47.3,-17.5,-90,0,0)
Dump_Small_Lid_Pos = Point_cls(0,40.3,-10,-90,0,0)
Collate_Lid_Pos = Point_cls(0,shake_y,-9.8,-90,0,0)
Put_On_Lid_Pos = Point_cls(0,shake_y,-10.9,-90,0,0)
Press_Lid_On_Pos = Point_cls(0,shake_y,-20.1,-90,0,0)
#---small lid---
small_lid_y=shake_y-7
Get_Small_Lid_Pos = Point_cls(0,small_lid_y,-12.6,-90,0,0)
Take_Off_Small_Lid_Pos = Point_cls(0,small_lid_y,-10,-90,0,0)
#---shake---
Grip_Shake_For_Shake_Pos = Point_cls(0,shake_y,-17.3,-90,0,0)
Shake_Up_Pos = Point_cls(0,76.8,27,-90,-300,-90)
Shake_Down_Pos = Point_cls(0,86.8,-18,-90,-310,-90)
#---pour---
Pre_Grip_Pos = Point_cls(-2,shake_y,-5,-90,0,90)
Grip_Shake_For_Pour_Pos = Point_cls(-2,shake_y,-18,-130,0,90)
Grip_Shake_Lift_Pos = Point_cls(-2,shake_y,-13,-130,0,90)
Lift_Up_Full_Shake_Pos = Point_cls(-3,56.8,5,-90,0,90)
Pour_Product_Ready_Pos = Point_cls(-34.54,39.99,-1,-90,0,-231.35)
Pour_Product_Pour_Pos = Point_cls(-34.54,39.99,-1,0,0,-231.35) # 180 - 50 = 130 (deg)
Pour_Product_Down_Pos = Point_cls(-34.54,39.99,-7.2,0,0,-231.35) # 180 - 50 = 130 (deg)


#keywords str 模糊詞間用空格隔開 如：'綠 旅'
full_ice_kw_str = '全冰 正常冰 陳冰 成冰 餐廳'
half_ice_kw_str = '少冰 燒餅 哨兵 扫并 骚饼 扫冰'
little_ice_kw_str = '微冰 胃病 衛兵'
no_ice_kw_str = '去冰 無冰 去並 曲柄'
full_sugar_kw_str = '全糖 全湯 全唐 卷坊 全餐 健康 減糖'
half_sugar_kw_str = '半糖 辦湯'
little_sugar_kw_str = '微糖 爲湯 維棠'
no_sugar_kw_str = '無糖 去糖 無湯'
green_t_kw_str = '綠 浴 御 芋'
black_t_kw_str = '紅 宏 洪 哄'
duo_duo_kw_str = '多多 嘟嘟 都都 豆豆 痘痘 兜兜'
dong_gua_t_kw_str = '冬瓜 動畫 東華 凍花 瓜 花'

#public
Arm_connected=False
start_time=time()
cmd_str='無'
pos_ok_flag=True
simu_flag=False
acc_rate=50
step_flag=[1,1,1,1,1,1,1]
grip_state_str='停止 夾緊 Tight Soft ？ 鬆至極限 鬆開 鬆至極限'
Current_Pos=Point_cls(Home_Pos.x,Home_Pos.y,Home_Pos.z,Home_Pos.pitch,Home_Pos.roll,Home_Pos.yaw)
Current_grip=grip_state_str.split()[0]
move_str='Start'
finished_flag=True
# ice_start_tm=time()
ice_end_tm=time()
sugar_start_tm=time()
sugar_end_tm=time()
arm_speed_mode=0
move_mode=2 # hiwin: 2 for p2p; 3 for line
simu_mode=1 # true for p2p, false for line
ang_fig=0 # on & off
analyse_mode=1 # 1 for on
trajetory_disp=1
speed_value=30
op_speed_limit=[37,16,24,35]
def SpeedModeToggle(mode):
    global arm_speed_mode,speed_value
    arm_speed_mode=mode
    speed_value=100-mode*80
    km.op_factor=0.1**(1-mode)

z_change=float(2)
arm_height=float(84.425)
table_height=float(101.85)
delt_height=table_height-arm_height
delt_z=(table_height-arm_height)-delt_height+z_change
def RefreshDelt_Z():
    global table_height,arm_height,delt_height,delt_z
    Hiwin_Move_Log.WriteOut(str(table_height)+str(arm_height)+str(delt_height)+str(delt_z))
    delt_z=(table_height-arm_height)-delt_height+z_change

#class of ingredients: drinks & ice & sugar
class Ingredient_cls():
    def __init__(self,kind_str,name_str,kw_str,Pos,Above_Pos,press_time):
        self.kind_str = kind_str
        self.name_str = name_str
        self.kw_str = kw_str
        self.Pos = Pos
        self.Above_Pos = Above_Pos
        self.press_time = press_time #for sugar
    def PrintPos(self):
        print('------------------------------------------------------------')
        print(self.kind_str+'位置: x:',[self.Pos.x,self.Pos.y,self.Pos.z,self.Pos.pitch,self.Pos.roll,self.Pos.yaw])
    def PrintPressTime(self):
        print('按壓果糖瓶{}下'.format(self.press_time))

ice_list=list()
drk_list=list()
sugar_list=list()

def InitData(delt_z):
    global ice_list, drk_list, sugar_list,\
        L_Backward_Collate_Pos,R_Forward_Collate_Pos,Above_Shake_Pos,\
        Above_Ice_Pos,Full_Ice_Pos,Half_Ice_Pos,Little_Ice_Pos,No_Ice_Pos,Dump_Ice_Pos,\
            Above_Dump_Ice_Pos,\
        Green_T_Pos,Blace_T_Pos,Duo_Duo_Pos,Dong_Gua_T_Pos,\
            Above_Green_T_Pos,Above_Blace_T_Pos,Above_Duo_Duo_Pos,Above_Dong_Gua_T_Pos,\
            Pour_Drk_Ready_Pos,Pour_Drk_Pour_R_Pos,Pour_Drk_Pour_Pos,\
        Back_Sugar_Pos,Above_Sugar_Unspined_Pos,Spin_Sugar_In_Pos,\
            Press_Sugar_Ready_Pos,Press_Sugar_Pos,Spin_Out_Ready_Pos,Spin_Sugar_Out_Pos,\
        Above_Lid_Pos,Lid_Pos,Dump_Small_Lid_Pos,Put_On_Lid_Pos,Press_Lid_On_Pos,Grip_Shake_For_Shake_Pos,\
            Collate_Lid_Pos,Get_Small_Lid_Pos,Take_Off_Small_Lid_Pos,\
        Pre_Grip_Pos,Grip_Shake_For_Pour_Pos,Grip_Shake_Lift_Pos,Lift_Up_Full_Shake_Pos,\
            Pour_Product_Ready_Pos,Pour_Product_Pour_Pos,Pour_Product_Down_Pos

    L_Backward_Collate_Pos.z+=delt_z;  R_Forward_Collate_Pos.z+=delt_z
    Above_Shake_Pos.z+=delt_z
    #---ice---
    Above_Ice_Pos.z+=delt_z;  Full_Ice_Pos.z+=delt_z;  Half_Ice_Pos.z+=delt_z
    Little_Ice_Pos.z+=delt_z;  No_Ice_Pos.z+=delt_z;  Dump_Ice_Pos.z+=delt_z;  Above_Dump_Ice_Pos.z+=delt_z
    #---drink---
    Green_T_Pos.z+=delt_z;  Blace_T_Pos.z+=delt_z;  Duo_Duo_Pos.z+=delt_z
    Dong_Gua_T_Pos.z+=delt_z;  Above_Green_T_Pos.z+=delt_z;  Above_Blace_T_Pos.z+=delt_z
    Above_Duo_Duo_Pos.z+=delt_z;  Above_Dong_Gua_T_Pos.z+=delt_z;  Pour_Drk_Ready_Pos.z+=delt_z
    Pour_Drk_Pour_R_Pos.z+=delt_z;  Pour_Drk_Pour_Pos.z+=delt_z
    #---sugar---
    Back_Sugar_Pos.z+=delt_z;  Above_Sugar_Unspined_Pos.z+=delt_z;  Spin_Sugar_In_Pos.z+=delt_z
    Press_Sugar_Ready_Pos.z+=delt_z;  Press_Sugar_Pos.z+=delt_z;  Spin_Out_Ready_Pos.z+=delt_z; Spin_Sugar_Out_Pos.z+=delt_z
    #---lid---
    Above_Lid_Pos.z+=delt_z;  Lid_Pos.z+=delt_z;  Dump_Small_Lid_Pos.z+=delt_z;  Put_On_Lid_Pos.z+=delt_z
    Press_Lid_On_Pos.z+=delt_z;  Grip_Shake_For_Shake_Pos.z+=delt_z
    Get_Small_Lid_Pos.z+=delt_z;  Take_Off_Small_Lid_Pos.z+=delt_z    
    Collate_Lid_Pos.z+=delt_z
    #---pour---
    Pre_Grip_Pos.z+=delt_z;  Grip_Shake_For_Pour_Pos.z+=delt_z;  Grip_Shake_Lift_Pos.z+=delt_z;  Lift_Up_Full_Shake_Pos.z+=delt_z 
    Pour_Product_Ready_Pos.z+=delt_z;  Pour_Product_Pour_Pos.z+=delt_z;  Pour_Product_Down_Pos.z+=delt_z
    
   
    Full_Ice = Ingredient_cls('冰塊','全冰',full_ice_kw_str,Full_Ice_Pos,Above_Ice_Pos,0)
    Half_Ice = Ingredient_cls('冰塊','少冰',half_ice_kw_str,Half_Ice_Pos,Above_Ice_Pos,0)
    Little_Ice = Ingredient_cls('冰塊','微冰',little_ice_kw_str,Little_Ice_Pos,Above_Ice_Pos,0)
    No_Ice = Ingredient_cls('冰塊','去冰',no_ice_kw_str,No_Ice_Pos,Above_Ice_Pos,0)
    ice_list = [Full_Ice,Half_Ice,Little_Ice,No_Ice]

    Green_T = Ingredient_cls('飲料','綠茶',green_t_kw_str,Green_T_Pos,Above_Green_T_Pos,0)
    Black_T = Ingredient_cls('飲料','紅茶',black_t_kw_str,Blace_T_Pos,Above_Blace_T_Pos,0)
    Duo_Duo = Ingredient_cls('飲料','比菲多多',duo_duo_kw_str,Duo_Duo_Pos,Above_Duo_Duo_Pos,0)
    Dong_Gua_T = Ingredient_cls('飲料','冬瓜茶',dong_gua_t_kw_str,Dong_Gua_T_Pos,Above_Dong_Gua_T_Pos,0)
    drk_list = [Green_T,Black_T,Duo_Duo,Dong_Gua_T]

    Full_Sugar = Ingredient_cls('甜度','全糖',full_sugar_kw_str,Press_Sugar_Pos,Press_Sugar_Ready_Pos,3)# Pos here only for declare
    Half_Sugar = Ingredient_cls('甜度','半糖',half_sugar_kw_str,Press_Sugar_Pos,Press_Sugar_Ready_Pos,2)
    Little_Sugar = Ingredient_cls('甜度','微糖',little_sugar_kw_str,Press_Sugar_Pos,Press_Sugar_Ready_Pos,1)
    No_Sugar = Ingredient_cls('甜度','無糖',no_sugar_kw_str,Press_Sugar_Pos,Press_Sugar_Ready_Pos,0)
    sugar_list = [Full_Sugar,Half_Sugar,Little_Sugar,No_Sugar]

##--------------------data end-------------------------

#enum grip action
gp_stop=0
gp_catch=1
gp_tight_catch=2
gp_soft_catch=3
gp_open=5
gp_loosen=6
grip_delay=1
##------------------move control-----------------------
class Action_cls():
    def __init__(self,MoveFunc):
        self.MoveFunc=MoveFunc
    def ArmMove(self,Position,speed,grip,string):
        global Current_Pos,move_str,move_mode
        move_mode=2
        move_str=string
        Current_Pos=Point_cls(Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw)
        self.MoveFunc(Position,speed,grip,string)
    def LimitArmMove(self,Position,speed,speed_lim,grip,string):
        if speed > speed_lim:
            self.ArmMove(Position,speed_lim,gp_stop,move_str) 
        else:
            self.ArmMove(Position,speed,gp_stop,move_str)
    def LinearMove(self,Position,speed,grip,string):
        global Current_Pos,move_str,move_mode
        move_mode=3
        move_str=string
        Current_Pos=Point_cls(Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw)
        self.MoveFunc(Position,speed,grip,string)
    def LimitLinearMove(self,Position,speed,speed_lim,grip,string):
        if speed > speed_lim:
            self.LinearMove(Position,speed_lim,gp_stop,move_str) 
        else:
            self.LinearMove(Position,speed,gp_stop,move_str)
    def GripCtrl(self,Position,speed,gp_ctrl,move_str,grip_str):
        global gp_catch,gp_loosen,gp_open,gp_stop,gp_tight_catch,gp_soft_catch,Current_grip,grip_state_str,grip_delay
        self.ArmMove(Position,speed,gp_stop,move_str) #move
        Current_grip=grip_state_str.split()[gp_ctrl]
        self.ArmMove(Position,speed,gp_ctrl,grip_str) #grip
        # if grip_delay:
        #     if grip_str==gp_soft_catch or grip_str==gp_tight_catch:
        #         sleep(1)
            # elif grip_str==gp_open:
            #     sleep(0.5)


def Left(Action):
    SpeedModeToggle(0)
    Action.ArmMove(L_Backward_Collate_Pos,50,0,'左下')
def Right(Action):
    SpeedModeToggle(1)
    Action.ArmMove(R_Forward_Collate_Pos,20,0,'右上')
    SpeedModeToggle(0)
def Home(Action):
    SpeedModeToggle(1)
    Action.ArmMove(Home_Pos,10,0,'回家！')
    SpeedModeToggle(0)
def ArmTest(Action):
    SpeedModeToggle(1)
    grip_test_list=[gp_stop,gp_catch,gp_open]
    pos_val_list=[5,-10,5]
    for pos_var in range(6):
        for i in range(3):
            Action.ArmMove(Home_Pos,30,grip_test_list[pos_var%2],'Arm Testing')
            Home_Pos.ChangeSelf(pos_var,pos_val_list[i])
    Action.ArmMove(Home_Pos,30,0,'Arm Testing')

class Write_File_cls():
    def __init__(self,file_name_str):
        self.file_name_str=file_name_str
    def StartWrite(self,string):#覆蓋原文本
        print(string)
        f=open(os.path.dirname(__file__)+'/'+self.file_name_str,'w')
        f.write(string+'\n')
        f.close()
    def WriteOut(self,string):#原文本續寫
        print(string)
        f=open(os.path.dirname(__file__)+'/'+self.file_name_str,'a')
        f.write(string+'\n')
        f.close()
    def WriteDate(self):
        self.WriteOut('===========================================================')
        self.WriteOut('\t\t{}'.format(datetime.datetime.now()))
        self.WriteOut('===========================================================')
Hiwin_Move_Log=Write_File_cls('move_log.txt')
Hiwin_Simu_Log=Write_File_cls('simu_log.txt')
Adjust_Log=Write_File_cls('pos_updated.txt')

##-------write pos part------------- 
class Write_Pos_cls():
    def __init__(self,file_name_str):
        self.Write_File=Write_File_cls(file_name_str)
    def WriteDate(self):
        self.Write_File.StartWrite('')
        self.Write_File.WriteDate()
    def WritePos(self,Position,speed,grip,string):
        self.Write_File.WriteOut(string+':'+str(Position.x)+','+str(Position.y)+','+str(Position.z)+','+str(Position.pitch)+','+str(Position.roll)+','+str(Position.yaw)+'\tgrip:'+str(grip))
        self.Write_File.WriteOut('-----------------------------------------------------------')
Write_Pos=Write_Pos_cls('full_procedure_n_pos.txt')
Write_Pos_Action=Action_cls(Write_Pos.WritePos)
##-------write pos part end------------- 

##-------move simulation part------------- 
def PointToMatrix(Pos):
    return np.array([Pos.x*10,Pos.y*10,Pos.z*10,Pos.pitch,Pos.roll,Pos.yaw])#from cm to mm

class Simulation_cls():
    def __init__(self,Home_Pos,Simu):
        self.p2p_mode=True
        self.linear_mode=False
        self.current_pos_array = PointToMatrix(Home_Pos)#初始化當前坐標
        self.grip_angle=0
        self.Simu=Simu
    def MoveSimulation(self,Position,speed,grip,string):
        global simu_mode
        if grip == gp_catch or grip == gp_tight_catch:
            self.grip_angle = self.Simu.Arm_Model.grip_on
        elif grip == gp_stop:
            self.grip_angle = self.grip_angle
        elif grip == gp_soft_catch:
            self.grip_angle = self.Simu.Arm_Model.grip_half
        elif grip == gp_open:
            self.grip_angle = self.Simu.Arm_Model.grip_off
        Hiwin_Simu_Log.WriteOut(string+':')
        Hiwin_Simu_Log.WriteOut('to position: '+str([Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw]))
        if analyse_mode:
            self.Simu.AdvSimu(self.current_pos_array,PointToMatrix(Position),self.grip_angle,speed,simu_mode)
        else:
            self.Simu.MySimuAni(self.current_pos_array,PointToMatrix(Position),self.grip_angle,speed,simu_mode)
        plt.show()
        Hiwin_Simu_Log.WriteOut('------------------------------------------------------------')
        self.current_pos_array=PointToMatrix(Position)
Animation_Simulation=Simulation_cls(Home_Pos,km.HiwinRA605_Simu)
Animation_Action=Action_cls(Animation_Simulation.MoveSimulation)
##-------move simulation part end------------- 

##-------delt_z test part----------------  
class Pos_Accessible_cls():
    def __init__(self,file_name_str):
        self.failed_Position=set()
        self.Write_File=Write_File_cls(file_name_str)
    def WriteDate(self):
        self.Write_File.StartWrite('')
        self.Write_File.WriteDate()
        self.Write_File.WriteOut('當前各軸極限爲:\n'+str(km.hiwin_axis_limit))
    def WriteOut(self,string):
        self.Write_File.WriteOut(string)
    def PosAccessibleTest(self,Position,speed,grip,string):
        km.HiwinRA605_Model.arm_center.InverseKinematics(PointToMatrix(Position))
        if km.theta_error_set:
            self.WriteOut('Failed: '+string+'\tPosition:'+str([Position.x,Position.y,Position.z,Position.pitch,Position.roll,Position.yaw]))
            self.failed_Position.add(Position)
Height_Range_Accessible=Pos_Accessible_cls('delt_z_test.txt')
Height_Range_Action=Action_cls(Height_Range_Accessible.PosAccessibleTest)
##-------delt_z test part end------------     

class Shake_Strategy_cls():
    def __init__(self,ice_quantity,drk_quantity,sugar_quantity,delay_flag):
        # sets of commands
        self.drk_cmd_set = set()
        self.sugar_cmd_set = set()
        self.ice_cmd_set = set()
        self.delay_flag = delay_flag
        self.drk_up_or_lat=0
        self.shake_up_or_lat=0
        self.ice_quantity=ice_quantity
        self.drk_quantity=drk_quantity
        self.sugar_quantity=sugar_quantity
    ##-------get command part-------
    #put commands matched into set
    def GetCmdToSet(self,cls_list,cmd_str,cmd_set):
        cmd_set.clear()
        for ingred in cls_list:
            for kw in ingred.kw_str.split():
                if cmd_str.find(kw) != -1:
                    cmd_set.add(ingred.name_str)
        Hiwin_Move_Log.WriteOut(cls_list[0].kind_str+':'+str(cmd_set))
    def CheckCmd(self):
        global drk_list,ice_list,sugar_list,cmd_str,move_str,start_time,finished_flag
        InitData(0) #!!!!!!!!!!!!!!!!! DONT CHANGE THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Hiwin_Move_Log.WriteOut(cmd_str)
        self.GetCmdToSet(drk_list,cmd_str,self.drk_cmd_set)
        self.GetCmdToSet(ice_list,cmd_str,self.ice_cmd_set)
        self.GetCmdToSet(sugar_list,cmd_str,self.sugar_cmd_set)
        ##----check ingredients' quantity----
        if len(self.drk_cmd_set) != self.drk_quantity or len(self.ice_cmd_set) !=self.ice_quantity or len(self.sugar_cmd_set) !=self.sugar_quantity:
            cmd_str="Wrong Command!"
            move_str=cmd_str
            # tk.messagebox.showinfo(title=cmd_str,message="Please say again.")
            Hiwin_Move_Log.WriteOut(cmd_str)
            Hiwin_Move_Log.WriteOut("Please say again.")
            return False
        else:
            finished_flag=False
            start_time=time()
            return True
    ##-------get command part end-------

    ##-------full procedure------------
    def SmartShaking(self,Action_cls):
        global step_flag,speed_value,delt_z,finished_flag,ice_end_tm,sugar_end_tm,sugar_start_tm
        def GetInFit():
            delt_l=0.3
            delt_l_list=[delt_l,-2*delt_l,delt_l]
            var_name_str='x y z'
            for var in range(2):
                for i in range(3):
                    Current_Pos.ChangeSelf(var,delt_l_list[i])
                    Action_cls.ArmMove(Current_Pos,speed_value,gp_stop,'微調'+var_name_str.split()[var])
        InitData(delt_z)
        Hiwin_Move_Log.WriteOut('桌面相對原點高度: {:.1f} cm'.format(-27.5-11+delt_height+delt_z))
        if simu_flag:
            Shake_Up_Pos.pitch=-30
            Shake_Up_Pos.roll=-90
            Shake_Up_Pos.yaw=0
            Shake_Down_Pos.pitch=-40
            Shake_Down_Pos.roll=-90
            Shake_Down_Pos.yaw=0
        if step_flag[0]:
            # ice_start_tm=time()
            ##----add ice----
            for ice in range(4):
                ice_end_tm=time()
                if ice_list[ice].name_str in self.ice_cmd_set:
                    if ice_list[ice].name_str != '去冰':
                        ice_list[ice].PrintPos()  #print ice Position
                        Action_cls.ArmMove(Above_Ice_Pos,speed_value,gp_stop,'移動至冰塊上方') #above ice
                        Action_cls.GripCtrl(ice_list[ice].Pos,speed_value,gp_tight_catch,'向下移動','夾住冰塊('+ice_list[ice].name_str+')') #move to and grip ice
                        Action_cls.ArmMove(Above_Ice_Pos,speed_value,gp_stop,'夾起至上方') #lift up
                        Action_cls.ArmMove(Above_Dump_Ice_Pos,speed_value,gp_stop,'移動至雪克杯上方') #lift up
                        Action_cls.GripCtrl(Dump_Ice_Pos,speed_value,gp_open,'向下移動','丟冰塊('+ice_list[ice].name_str+')')
                        Action_cls.ArmMove(Above_Dump_Ice_Pos,speed_value,gp_stop,'移動至雪克杯上方') #lift up
            ##----add ice end----
            ice_end_tm=time()

        if step_flag[1]:
            ##----add drink----
            for drink in range(4):
                if drk_list[drink].name_str in self.drk_cmd_set:
                    drk_list[drink].PrintPos() #print drink Positon
                    #get drink
                    Action_cls.ArmMove(drk_list[drink].Above_Pos,speed_value,gp_stop,'移動至'+drk_list[drink].name_str+'上方') #move to above drink
                    Action_cls.GripCtrl(drk_list[drink].Pos,speed_value,gp_soft_catch,'向下移動','夾住'+drk_list[drink].name_str) #move to and grip
                    # tk.messagebox.showinfo(message='ready?')
                    Action_cls.ArmMove(drk_list[drink].Above_Pos,speed_value,gp_stop,'夾起至上方') #move to above drink
                    #pour
                    if arm_speed_mode==1:
                        Action_cls.LimitArmMove(Pour_Drk_Ready_Pos,speed_value,op_speed_limit[drink],gp_stop,'移動至雪克杯上方準備倒'+drk_list[drink].name_str) #move to ready pour Position
                        sleep(3)
                        Action_cls.LimitArmMove(Pour_Drk_Pour_R_Pos,speed_value,30,gp_stop,'倒'+drk_list[drink].name_str) #pour
                        Action_cls.LimitArmMove(Pour_Drk_Pour_Pos,speed_value,25,gp_stop,'倒'+drk_list[drink].name_str) #pour
                    else:
                        Action_cls.ArmMove(Pour_Drk_Ready_Pos,speed_value,gp_stop,'移動至雪克杯上方準備倒'+drk_list[drink].name_str) #move to ready pour Position
                        Action_cls.ArmMove(Pour_Drk_Pour_R_Pos,speed_value,gp_stop,'倒'+drk_list[drink].name_str) #pour
                        Action_cls.ArmMove(Pour_Drk_Pour_Pos,speed_value,gp_stop,'倒'+drk_list[drink].name_str) #pour
                    if self.delay_flag:
                        sleep(3.75)
                    Action_cls.ArmMove(Pour_Drk_Ready_Pos,speed_value,gp_stop,'倒'+drk_list[drink].name_str+'結束') #end pour
                    #put back drink
                    if arm_speed_mode==1:
                        Action_cls.LimitArmMove(drk_list[drink].Above_Pos,speed_value,op_speed_limit[drink]+10,gp_stop,'移動至'+drk_list[drink].name_str+'空位上方') #move to above drink
                    else:
                        Action_cls.ArmMove(drk_list[drink].Above_Pos,speed_value,gp_stop,'移動至'+drk_list[drink].name_str+'空位上方') #move to above drink
                    Action_cls.GripCtrl(drk_list[drink].Pos,speed_value,gp_open,'放下'+drk_list[drink].name_str,'鬆開'+drk_list[drink].name_str) #move to and release
                    Action_cls.ArmMove(drk_list[drink].Above_Pos,speed_value,gp_stop,'移動至'+drk_list[drink].name_str+'上方') #move to above drink
            ##----add drink end----

        if step_flag[2]:
            sugar_start_tm=time()
            ##----add sugar----
            for sugar in range(4):
                sugar_end_tm=time()
                if sugar_list[sugar].name_str in self.sugar_cmd_set:
                    sugar_list[sugar].PrintPressTime() #print sugar press time
                    if sugar_list[sugar].press_time:
                        Action_cls.ArmMove(Back_Sugar_Pos,speed_value,gp_stop,'移動至果糖瓶後方') 
                        Action_cls.ArmMove(Above_Sugar_Unspined_Pos,speed_value,gp_stop,'移動至果糖瓶上方')
                        Action_cls.ArmMove(Spin_Sugar_In_Pos,speed_value,gp_stop,'順時針旋轉吸管') 
                        Action_cls.GripCtrl(Press_Sugar_Ready_Pos,speed_value,gp_tight_catch,'回轉以按壓果糖瓶','夾') 
                        for cnt in range(sugar_list[sugar].press_time):
                            Action_cls.ArmMove(Press_Sugar_Pos,speed_value,gp_stop,'按下') #press
                            Action_cls.ArmMove(Press_Sugar_Ready_Pos,speed_value,gp_stop,'鬆開') #unpress
                        Action_cls.GripCtrl(Spin_Out_Ready_Pos,speed_value,gp_open,'向上移動','鬆開') 
                        Action_cls.ArmMove(Spin_Sugar_Out_Pos,speed_value,gp_stop,'逆時針旋轉吸管') #spin the straw away shake
                        Action_cls.ArmMove(Above_Sugar_Unspined_Pos,speed_value,gp_stop,'回轉') #move to above sugar
                        Action_cls.ArmMove(Back_Sugar_Pos,speed_value,gp_stop,'移動至果糖瓶後方') 
            ##----add sugar end----
            sugar_end_tm=time()

        if step_flag[3]:
            ##----put on lid----
            #get lid
            Action_cls.ArmMove(Above_Lid_Pos,speed_value,gp_stop,'移動至雪克杯蓋上方') #move to above lid
            Action_cls.GripCtrl(Lid_Pos,speed_value,gp_tight_catch,'移動至雪克杯蓋','夾住雪克杯蓋') #move to and grip
            Action_cls.ArmMove(Above_Lid_Pos,speed_value,gp_stop,'拿起雪克杯蓋') #lift up lid
            #put on lid
            Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'移動至雪克杯上方') #lift up lid
            Action_cls.ArmMove(Collate_Lid_Pos,speed_value,gp_stop,'蓋杯蓋') #lift up lid
            GetInFit()
            Action_cls.GripCtrl(Put_On_Lid_Pos,speed_value,gp_open,'移動至雪克杯上方','鬆開雪克杯蓋') #put on and release
            ##----put on lid end----

        if step_flag[4]:
            ##----shake----
            if arm_speed_mode==1:
                sleep(0.5)
            Action_cls.ArmMove(Put_On_Lid_Pos,speed_value,gp_stop,'移動至雪克杯上方') #lift up lid
            Action_cls.LimitArmMove(Press_Lid_On_Pos,speed_value,20,gp_stop,'壓杯蓋') #move to above shake
            Action_cls.GripCtrl(Grip_Shake_For_Shake_Pos,speed_value,gp_tight_catch,'移動至雪克杯','夾住雪克杯') #move to and grip
            Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'移動至雪克杯空位上方') #move to above shake
            if arm_speed_mode==1:
                for cnt in range(3):
                    Action_cls.LimitArmMove(Shake_Up_Pos,speed_value,30,gp_stop,'上搖') #shake up
                    Action_cls.LimitArmMove(Shake_Down_Pos,speed_value,30,gp_stop,'下搖') #shake down
                Action_cls.LimitArmMove(Shake_Up_Pos,speed_value,30,gp_stop,'搖飲結束') #shake up
                Action_cls.LimitArmMove(Above_Shake_Pos,speed_value,40,gp_stop,'移動至雪克杯空位上方') #move to above shake
            else:
                for cnt in range(3):
                    Action_cls.ArmMove(Shake_Up_Pos,speed_value,gp_stop,'上搖') #shake up
                    Action_cls.ArmMove(Shake_Down_Pos,speed_value,gp_stop,'下搖') #shake down
                Action_cls.ArmMove(Shake_Up_Pos,speed_value,gp_stop,'搖飲結束') #shake up
                Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'移動至雪克杯空位上方') #move to above shake
            Action_cls.GripCtrl(Grip_Shake_For_Shake_Pos,speed_value,gp_open,'放下雪克杯','鬆開雪克杯') #move to and release
            ##----shake end----

        if step_flag[5]:
            ##----take off small lid----
            Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'移動至雪克杯上方') #move to above shake
            Action_cls.GripCtrl(Get_Small_Lid_Pos,speed_value,gp_soft_catch,'移動至雪克杯小蓋','夾住雪克杯小蓋') #move to and grip
            Action_cls.ArmMove(Take_Off_Small_Lid_Pos,speed_value,gp_stop,'取下雪克杯小蓋') #take off small lid
            Action_cls.GripCtrl(Dump_Small_Lid_Pos,speed_value,gp_open,'放下雪克杯小蓋','鬆開雪克杯小蓋') #put on and release
            ##----take off small lid end----
        
        if step_flag[6]:
            ##----pour product----
            Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'移動至雪克杯上方') #move to above shake
            Action_cls.ArmMove(Pre_Grip_Pos,speed_value,gp_stop,'移動至雪克杯上方') #move to above shake
            Action_cls.GripCtrl(Grip_Shake_For_Pour_Pos,speed_value,gp_tight_catch,'移動至雪克杯','夾住雪克杯') #move to and grip
            Action_cls.ArmMove(Grip_Shake_Lift_Pos,speed_value,gp_stop,'向上移動')
            if arm_speed_mode==1:
                Action_cls.LimitArmMove(Lift_Up_Full_Shake_Pos,speed_value,80,gp_stop,'移動至雪克杯空位上方') #move to above shake
                Action_cls.LimitArmMove(Pour_Product_Ready_Pos,speed_value,19,gp_stop,'準備倒飲料至手搖杯') #move to ready pour Position
                Action_cls.LimitArmMove(Pour_Product_Pour_Pos,speed_value,60,gp_stop,'倒飲料至手搖杯') #pour
            else:
                Action_cls.ArmMove(Lift_Up_Full_Shake_Pos,speed_value,gp_stop,'移動至雪克杯空位上方') #move to above shake
                Action_cls.ArmMove(Pour_Product_Ready_Pos,speed_value,gp_stop,'準備倒飲料至手搖杯') #move to ready pour Position
                Action_cls.ArmMove(Pour_Product_Pour_Pos,speed_value,gp_stop,'倒飲料至手搖杯') #pour
            Action_cls.ArmMove(Pour_Product_Down_Pos,speed_value,gp_stop,'向下移動') #pour
            if self.delay_flag:
                sleep(10)
            Action_cls.ArmMove(Pour_Product_Pour_Pos,speed_value,gp_stop,'向上移動') #pour
            Action_cls.ArmMove(Pour_Product_Ready_Pos,speed_value,gp_stop,'倒飲料至手搖杯結束') #end pour
            Action_cls.ArmMove(Lift_Up_Full_Shake_Pos,speed_value,gp_stop,'移動至雪克杯空位上方') #move to above shake
            Action_cls.ArmMove(Grip_Shake_Lift_Pos,speed_value,gp_stop,'向下移動')
            Action_cls.GripCtrl(Grip_Shake_For_Pour_Pos,speed_value,gp_open,'放下雪克杯','鬆開雪克杯') #put on and release
            Action_cls.ArmMove(Pre_Grip_Pos,speed_value,gp_stop,'移動至雪克杯上方') #move to above shake
            Action_cls.ArmMove(Above_Shake_Pos,speed_value,gp_stop,'轉動') #move to above shake
        Hiwin_Move_Log.WriteOut('FINISHED!!!!!')
        finished_flag=True
        InitData(-delt_z)#reform pos
            ##----pour product end----
    ##-------full procedure end------------

def byebye():
    print ("shutdown time!")
