#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
import tkinter as tk
import shake_strategy_trigger as shake_trig
import shake_strategy_content as shake_cont
# interface for collation
# =======================================================================
# =23/07/2019:add above pos_collation                                   =
# =======================================================================
collate_speed=15
def LeftCollate():
    shake_cont.InitData(shake_cont.delt_z)
    # shake_cont.Left(shake_cont.Animation_Action)
    shake_cont.Left(shake_trig.Hiwin_Solo_Action)
    shake_cont.InitData(-shake_cont.delt_z)
def RightCollate():
    shake_cont.InitData(shake_cont.delt_z)
    # shake_cont.Right(shake_cont.Animation_Action)
    shake_cont.Right(shake_trig.Hiwin_Solo_Action)
    shake_cont.InitData(-shake_cont.delt_z)
def ArmTestAct():
    shake_cont.InitData(shake_cont.delt_z)
    # shake_cont.ArmTest(shake_cont.Animation_Action)
    shake_cont.ArmTest(shake_trig.Hiwin_Action)
    shake_cont.InitData(-shake_cont.delt_z)
def CupCollate():
    global collate_speed
    shake_cont.InitData(shake_cont.delt_z)
    # Action=shake_cont.Animation_Action
    Action=shake_trig.Hiwin_Action
    shake_cont.SpeedModeToggle(1)
    Action.ArmMove(shake_cont.Above_Shake_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯上方')
    Action.ArmMove(shake_cont.Pre_Grip_Pos,collate_speed,shake_cont.gp_stop,'轉動')
    Action.ArmMove(shake_cont.Grip_Shake_For_Pour_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯')
    tk.messagebox.showinfo(message='Shake OK?')
    Action.GripCtrl(shake_cont.Grip_Shake_For_Pour_Pos,collate_speed,shake_cont.gp_tight_catch,'移動至雪克杯','夾住雪克杯')
    tk.messagebox.showinfo(message='Shake OK?')
    Action.ArmMove(shake_cont.Lift_Up_Full_Shake_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯空位上方')
    Action.ArmMove(shake_cont.Pour_Product_Ready_Pos,collate_speed,shake_cont.gp_stop,'準備倒飲料至手搖杯')
    Action.ArmMove(shake_cont.Pour_Product_Pour_Pos,collate_speed,shake_cont.gp_stop,'倒飲料至手搖杯')
    Action.ArmMove(shake_cont.Pour_Product_Down_Pos,collate_speed,shake_cont.gp_stop,'向下移動')
    tk.messagebox.showinfo(message='Cup OK?')
    Action.ArmMove(shake_cont.Pour_Product_Pour_Pos,collate_speed,shake_cont.gp_stop,'向上移動')
    Action.ArmMove(shake_cont.Pour_Product_Ready_Pos,collate_speed,shake_cont.gp_stop,'倒飲料至手搖杯結束')
    Action.ArmMove(shake_cont.Lift_Up_Full_Shake_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯空位上方')
    Action.GripCtrl(shake_cont.Grip_Shake_For_Pour_Pos,collate_speed,shake_cont.gp_open,'放下雪克杯','鬆開雪克杯')
    tk.messagebox.showinfo(message='Cup OK?')
    Action.ArmMove(shake_cont.Pre_Grip_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯上方')
    Action.ArmMove(shake_cont.Above_Shake_Pos,collate_speed,shake_cont.gp_stop,'轉動')
    tk.messagebox.showinfo(message='Collation Finished!')
    Action.ArmMove(shake_cont.Home_Pos,collate_speed,shake_cont.gp_stop,'移動至原位')
    shake_cont.InitData(-shake_cont.delt_z)
def LidCollate():
    shake_cont.InitData(shake_cont.delt_z)
    # Action=shake_cont.Animation_Action
    Action=shake_trig.Hiwin_Solo_Action
    shake_cont.SpeedModeToggle(1)
    Action.LimitArmMove(shake_cont.Above_Lid_Pos,collate_speed,5,shake_cont.gp_stop,'移動至雪克杯蓋上方')
    Action.GripCtrl(shake_cont.Lid_Pos,collate_speed,shake_cont.gp_tight_catch,'移動至雪克杯蓋','夾住雪克杯蓋')
    tk.messagebox.showinfo(message='Lid OK?')
    Action.ArmMove(shake_cont.Above_Lid_Pos,collate_speed,shake_cont.gp_stop,'拿起雪克杯蓋')
    Action.ArmMove(shake_cont.Above_Shake_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯上方')
    Action.LimitArmMove(shake_cont.Collate_Lid_Pos,collate_speed,5,shake_cont.gp_stop,'蓋杯蓋')
    tk.messagebox.showinfo(message='Lid OK?')
    Action.ArmMove(shake_cont.Above_Shake_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯上方')
    Action.ArmMove(shake_cont.Above_Lid_Pos,collate_speed,shake_cont.gp_stop,'移動至雪克杯蓋上方')
    Action.GripCtrl(shake_cont.Lid_Pos,collate_speed//2,shake_cont.gp_open,'放下雪克杯蓋','鬆開雪克杯蓋')
    tk.messagebox.showinfo(message='Collation Finished!')
    Action.ArmMove(shake_cont.Home_Pos,collate_speed,shake_cont.gp_stop,'移動至原位')
    shake_cont.InitData(-shake_cont.delt_z)
def PosCollate():
    shake_cont.InitData(shake_cont.delt_z)
    pos_list = [shake_cont.Home_Pos,shake_cont.Full_Ice_Pos,shake_cont.Above_Ice_Pos,
                shake_cont.Above_Duo_Duo_Pos,shake_cont.Duo_Duo_Pos,shake_cont.Above_Duo_Duo_Pos,
                shake_cont.Above_Dong_Gua_T_Pos,shake_cont.Dong_Gua_T_Pos,shake_cont.Above_Dong_Gua_T_Pos,
                shake_cont.Above_Blace_T_Pos,shake_cont.Blace_T_Pos,shake_cont.Above_Blace_T_Pos,
                shake_cont.Above_Green_T_Pos,shake_cont.Green_T_Pos,shake_cont.Above_Green_T_Pos,
                shake_cont.Above_Lid_Pos,shake_cont.Back_Sugar_Pos,shake_cont.Above_Sugar_Unspined_Pos,
                shake_cont.Above_Sugar_Unspined_Pos,shake_cont.Back_Sugar_Pos]
    pause_list=[1,4,7,10,13,18]
    shake_cont.SpeedModeToggle(1)
    for i in range(len(pos_list)):
        shake_trig.Hiwin_Action.ArmMove(pos_list[i],collate_speed,0,'test point({}/{})'.format(i+1,len(pos_list)))
        # shake_cont.Animation_Action.ArmMove(pos_list[i],20,0,'test point({}/{})'.format(i+1,len(pos_list)))
        if i in pause_list:
            tk.messagebox.showinfo(message='Next Point?')
    tk.messagebox.showinfo(message='Collation Finished!')
    # shake_cont.Home(shake_cont.Animation_Action)
    shake_cont.Home(shake_trig.Hiwin_Action)
    shake_cont.InitData(-shake_cont.delt_z)
def Collation():
    collate = tk.Toplevel()
    collate.title('Coordinate Collation')
    collate.geometry('620x355')
    collate.wm_attributes("-topmost",1)
    def CollateOK():
        collate.destroy()
        # shake_cont.Home(shake_cont.Animation_Action)
        shake_cont.Home(shake_trig.Hiwin_Solo_Action)

    arm_test = tk.Button(collate,text='Arm Test',font=('Arial', 15),width=45,height=2,command=ArmTestAct) 
    arm_test.place(x=50,y=35)
    
    left_collate = tk.Button(collate,text='Left',font=('Arial', 15),width=19,height=2,command=LeftCollate) 
    left_collate.place(x=50,y=110)

    right_collate = tk.Button(collate,text='Right',font=('Arial', 15),width=19,height=2,command=RightCollate) 
    right_collate.place(x=335,y=110)

    pos_collate = tk.Button(collate,text='Pos',font=('Arial', 15),width=12,height=2,command=PosCollate) 
    pos_collate.place(x=50,y=185)

    cup_collate = tk.Button(collate,text='Cup',font=('Arial', 15),width=12,height=2,command=CupCollate) 
    cup_collate.place(x=230,y=185)
    
    lid_collate = tk.Button(collate,text='Lid',font=('Arial', 15),width=12,height=2,command=LidCollate) 
    lid_collate.place(x=410,y=185)

    collate_ok = tk.Button(collate,text='OK',font=('Arial', 15),width=45,height=2,command=CollateOK) 
    collate_ok.place(x=50,y=260)

    def BackHome():
        collate.destroy()
        # shake_cont.Home(shake_cont.Animation_Action)
        shake_cont.Home(shake_trig.Hiwin_Solo_Action)

    collate.protocol('WM_DELETE_WINDOW', BackHome)

    collate.mainloop()