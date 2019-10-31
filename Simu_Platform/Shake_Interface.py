#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
import os
import threading
import tkinter as tk
from time import time
from tkinter import ttk

import numpy as np
import rospy
from PIL import Image, ImageTk

import shake_strategy_content as shake_cont
import shake_strategy_simulation as shake_simu

# smart-shaking interface(main)
# =======================================================================
# =24/07/2019:fix display error                                         =
# =======================================================================
#-----------------------------------------------------------------
root = tk.Tk()
root.title('Hiwin Smart Shaking')
root.geometry('630x690')   
root.wm_attributes("-topmost",1)

img_open = Image.open(os.path.abspath(os.path.dirname(__file__))+'/iclab_hiwin.png')
img_bottom = ImageTk.PhotoImage(img_open)
label_img = tk.Label(root, image = img_bottom)
label_img.place(x=0,y=580,width=630,height=110)
#---------------------------------------------------------------
name_pady=5
val_pady=4
state_frm=tk.Frame(root)
state_frm.place(x=14,y=10)
#==============================Action Flags===========================================
start_time=0
#------------------------------select steps------------------------------------
step_str='加冰塊 加飲料 加果糖 蓋大蓋 搖飲料 取小蓋 手搖杯'
step_var=[]
step_checkbtn=[]
step_frm=tk.Frame(root)
step_frm.place(x=440,y=140)

selected_steps=tk.Label(step_frm,text='Steps',font=('Arial', 14,'bold'))
selected_steps.pack(side='top')

step_frm_l=tk.Frame(step_frm)
step_frm_l.pack(side='left')
step_frm_r=tk.Frame(step_frm)
step_frm_r.pack(side='right')

def StepFlagGet():
    shake_cont.step_flag=[step_var[i].get() for i in range(7)]

for i in range(7):
    step_var.append(tk.IntVar(value=1))
    if i in range(4):
        step_checkbtn.append(tk.Checkbutton(step_frm_l,text=step_str.split()[i],font=('Arial', 13),variable=step_var[i],onvalue=1,offvalue=0,command=StepFlagGet))
    else:
        step_checkbtn.append(tk.Checkbutton(step_frm_r,text=step_str.split()[i],font=('Arial', 13),variable=step_var[i],onvalue=1,offvalue=0,command=StepFlagGet))
    step_checkbtn[i].pack(pady=2)

def AllSteps():
    for i in range(7):
        step_var[i].set(value=all_step_var.get())
    StepFlagGet()
all_step_var=tk.IntVar(value=1)
all_step_ckbtn=tk.Checkbutton(step_frm_r,text='全步驟',font=('Arial', 13),variable=all_step_var,onvalue=1,offvalue=0,command=AllSteps)
all_step_ckbtn.pack(pady=3)
#==============================Current Param===========================================
param_frm=tk.Frame(root)
param_frm.place(x=5,y=140)
current_title=tk.Label(param_frm,text='Settings',font=('Arial', 14,'bold'))
current_title.pack(side='top')
param_name_frm=tk.Frame(param_frm)
param_name_frm.pack(side='left')
param_val_frm=tk.Frame(param_frm)
param_val_frm.pack(side='right')
param_val_frm_l=tk.Frame(param_val_frm)
param_val_frm_l.pack(side='left')
param_val_frm_r=tk.Frame(param_val_frm)
param_val_frm_r.pack(side='right')
#------------------------------current state-----------------------------------

Label_obj = tk.Label(state_frm,text='Aciton: Ready',fg='black',bg='white', font=('Arial', 15), width=53, height=2)
Label_obj.pack(padx=5,pady=4)

coodine_frm=tk.Frame(state_frm)
coodine_frm.pack()
pos_frm=tk.Frame(state_frm)
pos_frm.pack()

pos_str='x y z pitch roll yaw'
pos_val=[shake_cont.Current_Pos.x,shake_cont.Current_Pos.y,shake_cont.Current_Pos.z,
        shake_cont.Current_Pos.pitch,shake_cont.Current_Pos.roll,shake_cont.Current_Pos.yaw]
pos_label=[]
for i in range(6):
    if i in range(3):
        pos_label.append(tk.Label(coodine_frm,text=pos_str.split()[i]+':'+str(pos_val[i]),fg='black',bg='white', font=('Arial', 15), width=16, height=1))
    else:
        pos_label.append(tk.Label(pos_frm,text=pos_str.split()[i]+':'+str(pos_val[i]),fg='black',bg='white', font=('Arial', 15), width=16, height=1))
    pos_label[i].pack(side='left',padx=10,pady=5)

# def GripDelay():
#     global grip_delay_var
#     shake_cont.grip_delay=int(grip_delay_var.get())
#     # print(shake_cont.grip_delay)
# grip_delay_var=tk.IntVar(value=1)
# grip_delay_ckbtn=tk.Checkbutton(root,text='Grip Delay',font=('Arial', 13),variable=grip_delay_var,onvalue=1,offvalue=0,command=GripDelay)
# grip_delay_ckbtn.place(x=40,y=570)
#------------------------------show current param-----------------------------------
param_label=[]
show_param=[]
change_param_entry=[]
param_val=[shake_cont.speed_value,shake_cont.acc_rate,shake_cont.Animation_Simulation.Simu.fps,shake_cont.arm_height,shake_cont.table_height]
param_label_str='Speed(%) AccRate(%) FPS Arm_H(cm) Tab_H(cm)'
for i in range(5):
    param_label.append(tk.Label(param_name_frm,text=param_label_str.split()[i],font=('Arial', 12)))
    param_label[i].pack(padx=5,pady=name_pady)
    show_param.append(tk.Label(param_val_frm_l,text=str(param_val[i]),width=17,fg='black',bg='white',font=('Arial', 12)))
    show_param[i].pack(padx=5,pady=val_pady)
    change_param_entry.append(tk.Entry(param_val_frm_r,width=16,font=('Arial', 10)))
    change_param_entry[i].pack(padx=5,pady=val_pady)


#------------------------------change current param-----------------------------------
param_change_label=tk.Label(param_name_frm,text='',font=('Arial', 13))
param_change_label.pack(padx=5,pady=name_pady)
def ChangeParam():
    for i in range(5):
        if change_param_entry[i].get():
            if i == 0:
                param_val[i]=int(change_param_entry[i].get())  
            else:
                param_val[i]=float(change_param_entry[i].get())
    [shake_cont.speed_value,shake_cont.acc_rate,shake_cont.Animation_Simulation.Simu.fps,shake_cont.arm_height,shake_cont.table_height]=param_val
    shake_cont.RefreshDelt_Z()
    print('delt_z:',shake_cont.delt_z)
param_change_confirm = tk.Button(param_val_frm_r,text='Submit Param',font=('Arial', 11),width=11,height=1,command=ChangeParam) 
param_change_confirm.pack(padx=5,pady=val_pady)

def DefaultParam():
    shake_cont.speed_value= 30
    shake_cont.delt_z= float(0)
    shake_cont.Animation_Simulation.Simu.fps=24
defaut_param_confirm = tk.Button(param_val_frm_l,text='Default Param',font=('Arial', 11),width=17,height=1,command=DefaultParam) 
defaut_param_confirm.pack(padx=5,pady=val_pady)

class My_RadioBtn_cls():
    def __init__(self,lb_text,rb0_text,rb1_text,extra_cmd):
        self.var=tk.IntVar(value=True)
        self.label=tk.Label(param_name_frm,text=lb_text,font=('Arial', 12))
        self.label.pack(padx=5,pady=name_pady)
        self.rb0=tk.Radiobutton(param_val_frm_l,text=rb0_text,variable=self.var,value=0,command=self.MyCmd,font=('Arial', 12))
        self.rb0.pack(padx=5,pady=val_pady)
        self.rb1=tk.Radiobutton(param_val_frm_r,text=rb1_text,variable=self.var,value=1,command=self.MyCmd,font=('Arial', 12))
        self.rb1.pack(padx=5,pady=val_pady)
        self.extra_cmd=extra_cmd
    def MyCmd(self):
        self.extra_cmd(self.var.get())

def SpeedMode(var_get):
    global param_val
    shake_cont.SpeedModeToggle(int(var_get))
    param_val[0]=shake_cont.speed_value
speed_mode_rb=My_RadioBtn_cls('Speed Mode','Safety','Operating',SpeedMode)

def SimuMode(var_get):
    shake_cont.simu_mode=var_get
    if not var_get:
        shake_cont.analyse_mode=False
        shake_cont.ang_fig=False
        shake_cont.trajetory_disp=False
simu_mode_rb=My_RadioBtn_cls('Simu Mode','Linear','PTP',SimuMode)

def AnalyseMode(var_get):
    shake_cont.analyse_mode=var_get
    if not var_get:
        shake_cont.ang_fig=False
        shake_cont.trajetory_disp=False
        # shake_cont.simu_mode=True
analyse_mode_rb=My_RadioBtn_cls('Analytic Mode','Off (Grip)','On (Natural)',AnalyseMode)

def AngFig(var_get):
    shake_cont.ang_fig=var_get
ang_fig_rb=My_RadioBtn_cls('Angle Graph','Off','On (AnaMd)',AngFig)

def TrajDisp(var_get):
    shake_cont.trajetory_disp=var_get
traj_disp_rb=My_RadioBtn_cls('Show Trajetory','Off','On (AnaMd)',TrajDisp)

current_cmd=tk.Label(param_name_frm,text='Command',font=('Arial', 13))
current_cmd.pack(padx=5,pady=name_pady)
show_cmd=tk.Label(param_val_frm_l,text=shake_cont.cmd_str,width=17,fg='black',bg='white',font=('Arial', 12))
show_cmd.pack(padx=5,pady=val_pady)

def CmdInput():
    shake_cont.cmd_str = cmd_entry.get()
    if auto_simu_var.get():
        StartSimu()
cmd_entry = tk.Entry(param_val_frm_r,width=16,font=('Arial', 10))
cmd_entry.pack(padx=5,pady=val_pady)
cmd_confirm = tk.Button(param_val_frm_r,text='Submit Cmd',font=('Arial', 11),width=11,height=1,command=CmdInput) 
cmd_confirm.pack(padx=5,pady=val_pady)

#=============================================================================================

# #===================================General Func=============================================
auto_simu_var=tk.IntVar(value=0)
auto_simu_ckbtn=tk.Checkbutton(param_name_frm,text='Auto-Simu',font=('Arial', 13),variable=auto_simu_var,onvalue=1,offvalue=0)
auto_simu_ckbtn.pack(padx=5,pady=name_pady)
def AudioRecord():
    audio_thread=threading.Thread(target=shake_simu.SpeechRecog)
    audio_thread.start()
    if auto_simu_var.get():
        audio_thread.join()
        StartSimu()
audio_record = tk.Button(param_val_frm_l,text='Record',font=('Arial', 11),width=6,height=1,command=AudioRecord) 
audio_record.pack(side='left',padx=4,pady=val_pady)
def DefaultCmd():
    shake_cont.cmd_str = '冬瓜紅茶半糖少冰'
    if auto_simu_var.get():
        StartSimu()
default_cmd = tk.Button(param_val_frm_l,text='Default',font=('Arial', 11),width=6,height=1,command=DefaultCmd) 
default_cmd.pack(side='left',padx=4,pady=val_pady)

ctrl_frm=tk.Frame(root)
ctrl_frm.place(x=445,y=434)
ctrl_lb=tk.Label(ctrl_frm,text='Execute',font=('Arial', 14,'bold'))
ctrl_lb.pack(padx=5,pady=4)
# #======================================================================================

# #===================================from shake_trigger==============================================
arm_listening_flag=False
def ArmConnect():
    global arm_listening_flag
    if not arm_listening_flag:
        argv = rospy.myargv()
        rospy.init_node('strategy', anonymous=True)
        arm_listening_flag=True
def DefaultShake():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        default_shaking_thread=threading.Thread(target=shake_trig.DefaultShaking)
        default_shaking_thread.start()
        del shake_trig
def PosAdjustTest():
    global speed_mode_rb
    speed_mode_rb.var.set(value=0)
    speed_mode_rb.MyCmd()
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        adjust_thread=threading.Thread(target=shake_trig.PosAdjust)
        adjust_thread.start()
        del shake_trig
def SingleMove():
    if not cnct_var.get():
        tk.messagebox.showinfo('Notice','Hiwin is NOT connected!\nDo you want to Simulate?')
        import single_move_interface as sin_mov_inter
        sin_mov_thread=threading.Thread(target=sin_mov_inter.SingleMovement)
        sin_mov_thread.start()
        del sin_mov_inter
    else:
        # import coordinate_collation_interface as collate_inter
        # collate_thread=threading.Thread(target=collate_inter.Collation)
        # collate_thread.start()
        # del collate_inter
        pass
singe_move = tk.Button(ctrl_frm,text='Single Movement',font=('Arial', 11,'bold'),width=17,height=1,command=SingleMove) 
singe_move.pack(pady=2)

def CoordinateCollate():
    global speed_mode_rb
    speed_mode_rb.var.set(value=0)
    speed_mode_rb.MyCmd()
    StepFlagGet()
    if not cnct_var.get():
        tk.messagebox.showinfo('Notice','Hiwin is NOT connected!\nDo you want to Simulate?')
        import simu_collation_interface as simu_col_inter
        collate_thread=threading.Thread(target=simu_col_inter.Collation)
        collate_thread.start()
        del simu_col_inter
    else:
        import coordinate_collation_interface as collate_inter
        collate_thread=threading.Thread(target=collate_inter.Collation)
        collate_thread.start()
        del collate_inter
coordinate_collate = tk.Button(ctrl_frm,text='Coordinate Collation',font=('Arial', 11,'bold'),width=17,height=1,command=CoordinateCollate) 
coordinate_collate.pack(pady=2)

def RecordnStart():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        record_n_start_thread=threading.Thread(target=shake_trig.SpeechTriggerShaking)
        record_n_start_thread.start()
        del shake_trig
record_n_start=tk.Button(ctrl_frm,text='Record & Start!',font=('Arial', 11,'bold'),width=17,height=1,command=RecordnStart)
record_n_start.pack(pady=2)

#grip part
gp_frm=tk.Frame(root)
gp_frm.place(x=440,y=292)
grip_state=tk.Label(gp_frm,text='Grip Control',font=('Arial', 14,'bold'))
grip_state.pack(side='top')
Label_grip=tk.Label(gp_frm,text='Grip: '+shake_cont.Current_grip,fg='black',bg='white', font=('Arial', 15), width=14, height=1)
Label_grip.pack(pady=4)
gp_frm_btm=tk.Frame(gp_frm)
gp_frm_btm.pack(side='bottom')
gp_frm_btml=tk.Frame(gp_frm_btm)
gp_frm_btml.pack(side='left')
gp_frm_btmr=tk.Frame(gp_frm_btm)
gp_frm_btmr.pack(side='right')
def GripStop():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        stop_thread=threading.Thread(target=shake_trig.Hiwin_Action.GripCtrl,args =(shake_cont.Current_Pos,shake_cont.speed_value,shake_cont.gp_stop,'控制夾爪','停止'))
        stop_thread.start()
        del shake_trig
def GripTightCatch():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        catch_thread=threading.Thread(target=shake_trig.Hiwin_Action.GripCtrl,args =(shake_cont.Current_Pos,shake_cont.speed_value,shake_cont.gp_tight_catch,'控制夾爪','緊夾'))
        catch_thread.start()
        del shake_trig
def GripSoftCatch():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        open_thread=threading.Thread(target=shake_trig.Hiwin_Action.GripCtrl,args =(shake_cont.Current_Pos,shake_cont.speed_value,shake_cont.gp_soft_catch,'控制夾爪','鬆夾'))
        open_thread.start()
        del shake_trig
def GripOpen():
    if not cnct_var.get():
        tk.messagebox.showwarning('ERROR','Hiwin is NOT connected!')
    else:
        import shake_strategy_trigger as shake_trig
        ArmConnect()
        shake_trig.arm_state_listener()
        op_thread=threading.Thread(target=shake_trig.Hiwin_Action.GripCtrl,args =(shake_cont.Current_Pos,shake_cont.speed_value,shake_cont.gp_open,'控制夾爪','鬆開'))
        op_thread.start()
        del shake_trig
grip_stop = tk.Button(gp_frm_btml,text='Stop',font=('Arial', 11),width=6,height=1,command=GripStop) 
grip_stop.pack(padx=5,pady=4)
grip_catch = tk.Button(gp_frm_btml,text='Tight',font=('Arial', 11),width=6,height=1,command=GripTightCatch) 
grip_catch.pack(padx=5,pady=4)
grip_open = tk.Button(gp_frm_btmr,text='Soft',font=('Arial', 11),width=6,height=1,command=GripSoftCatch) 
grip_open.pack(padx=5,pady=4)
grip_loosen = tk.Button(gp_frm_btmr,text='Open',font=('Arial', 11),width=6,height=1,command=GripOpen) 
grip_loosen.pack(padx=5,pady=4)
# #======================================================================================

# #===================================from shake_simu==============================================
def StartSimu():
    shake_cont.Hiwin_Simu_Log.StartWrite('Simu Action')
    shake_cont.Hiwin_Simu_Log.WriteDate()
    simu_thread=threading.Thread(target=shake_simu.AnimationSimulation)
    simu_thread.start()
def PrintFullPos():
    print_thread=threading.Thread(target=shake_simu.WritePos)
    print_thread.start()
def Kine():
    import kinematics_interface as kine_inter
    print_thread=threading.Thread(target=kine_inter.Kinematics)
    print_thread.start()
    del kine_inter
def HeightRangeTest():
    height_test_thread=threading.Thread(target=shake_simu.HeightRangeAccessibleTest)
    height_test_thread.start()
# #======================================================================================

#==============================Interface Control==================================================
#------------------------------menu---------------------------------------
tot_tm_str_var=tk.StringVar(value='總耗時: 0m 0s')
sugar_tm_str_var=tk.StringVar(value='果糖耗時: {:.2f}s'.format(shake_cont.sugar_end_tm-shake_cont.sugar_start_tm))
ice_tm_str_var=tk.StringVar(value='冰塊耗時: {:.2f}s'.format(shake_cont.ice_end_tm-shake_cont.start_time))

menubar=tk.Menu(root)
cnct_var=tk.BooleanVar(value=shake_cont.Arm_connected)
cnct_str_var=tk.StringVar(value='Disconnected')
toolmenu = tk.Menu(menubar, tearoff=0)

toolmenu.add_command(label='Default Shake',command=DefaultShake)
toolmenu.add_command(label='Pos-Adjustable',command=PosAdjustTest)
toolmenu.add_separator()
toolmenu.add_command(label='Start Simu',command=StartSimu)
toolmenu.add_command(label='Height Test',command=HeightRangeTest)
toolmenu.add_command(label='Write Pos',command=PrintFullPos)
menubar.add_cascade(label="Tools", menu=toolmenu)
menubar.add_command(label='Kinematics',command=Kine)
menubar.add_checkbutton(label=cnct_str_var.get(),variable=cnct_var)
menubar.add_command(label=sugar_tm_str_var.get())
menubar.add_command(label=ice_tm_str_var.get())
menubar.add_command(label=tot_tm_str_var.get())
root.config(menu=menubar)
#-----------------------------update-----------------------------------
def update():
    global Label_grip,pos_label,pos_val,start_time,menubar
    cnct_var.set(value=shake_cont.Arm_connected)
    if cnct_var.get():
        cnct_str_var.set(value='Connected')
    else:
        cnct_str_var.set(value='Disconnected')
        
    shake_cont.step_flag=[step_var[i].get() for i in range(7)]
    show_cmd.config(text=shake_cont.cmd_str)

    start_time=shake_cont.start_time
    if not shake_cont.finished_flag:
        Label_obj.config(text='Action: '+shake_cont.move_str)
        tot_tm_str_var.set(value='總耗時: {:.0f}m {:.2f}s'.format((time()-start_time)//60,(time()-start_time)%60))
    if shake_cont.simu_flag:
        for i in range(6):
            pos_val[i]=np.around(shake_cont.Animation_Simulation.Simu.start_pos[i]*0.1*10**(i//3),decimals=2)
    else:
        pos_val=[shake_cont.Current_Pos.x,shake_cont.Current_Pos.y,shake_cont.Current_Pos.z,
                shake_cont.Current_Pos.pitch,shake_cont.Current_Pos.roll,shake_cont.Current_Pos.yaw]
    
    for i in range(6):
        pos_label[i].config(text=pos_str.split()[i]+': {:.2f}'.format(pos_val[i]))
    
    for i in range(5):
        show_param[i].config(text=str(param_val[i]))
    show_cmd.config(text=shake_cont.cmd_str)

    Label_grip.config(text='Grip: '+shake_cont.Current_grip)
    
    sugar_tm_str_var.set(value='果糖耗時: {:.2f}s'.format(shake_cont.sugar_end_tm-shake_cont.sugar_start_tm))
    ice_tm_str_var.set(value='冰塊耗時: {:.2f}s'.format(shake_cont.ice_end_tm-shake_cont.start_time))
    menubar.entryconfig(3,label=cnct_str_var.get())
    menubar.entryconfig(4,label=sugar_tm_str_var.get())
    menubar.entryconfig(5,label=ice_tm_str_var.get())
    menubar.entryconfig(6,label=tot_tm_str_var.get())
    

    if not shake_cont.simu_mode:
        shake_cont.analyse_mode=False
        shake_cont.ang_fig=False
        shake_cont.trajetory_disp=False
    if not shake_cont.analyse_mode:
        shake_cont.ang_fig=False
        shake_cont.trajetory_disp=False
        # shake_cont.simu_mode=True
    speed_mode_rb.var.set(value=shake_cont.arm_speed_mode)
    simu_mode_rb.var.set(value=shake_cont.simu_mode)
    ang_fig_rb.var.set(value=shake_cont.ang_fig)
    analyse_mode_rb.var.set(value=shake_cont.analyse_mode)
    traj_disp_rb.var.set(value=shake_cont.trajetory_disp)

    root.after(300, update)

update_thread=threading.Thread(target=update)
update_thread.start()


#-----------------------------close------------------------------------
def close():
    root.destroy()
    os._exit(0)
    if arm_listening_flag:
        rospy.spin()
        import Hiwin_RT605_Socket as ArmTask
        ArmTask.rospy.spin()

root.protocol('WM_DELETE_WINDOW', close)
#-------------------------------main----------------------------------
if __name__ == "__main__":
    root.mainloop()
