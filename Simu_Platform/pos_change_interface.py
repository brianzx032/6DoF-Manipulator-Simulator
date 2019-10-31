#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
import tkinter as tk
import threading
import shake_strategy_content as shake_cont
import shake_strategy_trigger as shake_trig
# interface for pos_change
# =======================================================================
# =24/07/2019:fix bugs                                                  =
# =======================================================================
def PosChange():
    shake_cont.pos_ok_flag=False
    pos_adjust = tk.Toplevel()
    pos_change_str='x y z pitch roll yaw'
    pos_change_input=[]
    pos_change_entry=[]
    pos_adjust.title('Position Adjustment')
    pos_adjust.geometry('620x200')
    pos_adjust.wm_attributes("-topmost",1)
    def PosOK():
        shake_cont.pos_ok_flag=True
        pos_adjust.destroy()
    def PosConfirm():
        for i in range(6):
            if pos_change_entry[i].get():
                shake_cont.Pos_diff[i]=float(pos_change_entry[i].get())
        shake_trig.RefreshTargetPos()
    for i in range(6):
        pos_change_input.append(tk.Label(pos_adjust,text=pos_change_str.split()[i]+':',width=4,font=('Arial', 15)))
        pos_change_input[i].place(x=40+200*(i%3),y=20+40*(i//3))
        pos_change_entry.append(tk.Entry(pos_adjust,width=10))
        pos_change_entry[i].place(x=90+200*(i%3),y=25+40*(i//3))

    pos_change_confirm = tk.Button(pos_adjust,text='Submit',font=('Arial', 15),width=20,height=2,command=PosConfirm) 
    pos_change_confirm.place(x=330,y=110)

    pos_change_ok = tk.Button(pos_adjust,text='OK',font=('Arial', 15),width=20,height=2,command=PosOK) 
    pos_change_ok.place(x=40,y=110)

    def BackHome():
        shake_cont.pos_ok_flag=True
        pos_adjust.destroy()
        shake_cont.Home(shake_trig.Hiwin_Action)
    pos_adjust.protocol('WM_DELETE_WINDOW', BackHome)
    pos_adjust.mainloop()