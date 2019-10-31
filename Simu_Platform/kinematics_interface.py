#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
import tkinter as tk
import numpy as np
import kinematics as km
def Kinematics():
    kine_win = tk.Toplevel()
    kine_win.title('Kinematics')
    kine_win.geometry('500x425')
    kine_win.wm_attributes("-topmost",1)

    pos_str='x y z pitch roll yaw'
    pos_input=[]
    pos_entry=[]
    pos_label=[]
    pos_val=np.array([0, 368, 113.5, -90, 0, 0])
    ang_input=[]
    ang_entry=[]
    ang_label=[]
    ang_val=np.array([0, 0, 0, 0, -90, 0])
    def Calculate(pos_val,ang_val):
        # global pos_val,ang_val
        cal_rslt=np.zeros((1,12))
        if cal_var.get()=='P2A':
            for i in range(6):
                if pos_entry[i].get():
                    pos_val[i]=float(pos_entry[i].get())*10*0.1**(i//3)
            cal_rslt=km.PosToAng(pos_val)
        else :
            for i in range(6):
                if ang_entry[i].get():
                    ang_val[i]=float(ang_entry[i].get())
            cal_rslt=km.AngToPos(ang_val)
        pos_val=cal_rslt[0:6]
        ang_val=cal_rslt[6:12]
        for i in range(6):
            pos_label[i].config(text=pos_str.split()[i]+': '+str(np.around(pos_val[i]*0.1*10**(i//3),decimals=2)))
            ang_label[i].config(text='Axis'+str(i+1)+': '+str(np.around(ang_val[i],decimals=2)))
    for i in range(6):
        pos_input.append(tk.Label(kine_win,text=pos_str.split()[i]+':',width=4,font=('Arial', 15)))
        pos_input[i].place(x=30+150*(i%3),y=185+40*(i//3))
        pos_entry.append(tk.Entry(kine_win,width=9))
        pos_entry[i].place(x=80+150*(i%3),y=190+40*(i//3))
        ang_input.append(tk.Label(kine_win,text='Axis'+str(i+1)+':',width=6,font=('Arial', 15)))
        ang_input[i].place(x=15+150*(i%3),y=265+40*(i//3))
        ang_entry.append(tk.Entry(kine_win,width=9))
        ang_entry[i].place(x=80+150*(i%3),y=270+40*(i//3))

        pos_label.append(tk.Label(kine_win,text=pos_str.split()[i]+': '+str(np.around(pos_val[i]*0.1*10**(i//3),decimals=2)),fg='black',bg='white', font=('Arial', 15), width=13, height=1))
        pos_label[i].place(x=30+150*(i%3),y=20+40*(i//3))
        ang_label.append(tk.Label(kine_win,text='Axis'+str(i+1)+': '+str(np.around(ang_val[i],decimals=2)),fg='black',bg='white', font=('Arial', 15), width=13, height=1))
        ang_label[i].place(x=30+150*(i%3),y=100+40*(i//3))
    cal_var=tk.StringVar(value='P2A')
    p2a=tk.Radiobutton(kine_win,text='Pos to Ang',variable=cal_var, value='P2A',font=('Arial', 13))
    p2a.place(x=40,y=350)
    a2p=tk.Radiobutton(kine_win,text='Ang to Pos',variable=cal_var, value='A2P',font=('Arial', 13))
    a2p.place(x=40,y=380)

    cal_confirm = tk.Button(kine_win,text='Calculate',font=('Arial', 15),width=14,height=2,command=lambda: Calculate(pos_val,ang_val)) 
    cal_confirm.place(x=165,y=350)

    kine_win.mainloop()

Kinematics()
