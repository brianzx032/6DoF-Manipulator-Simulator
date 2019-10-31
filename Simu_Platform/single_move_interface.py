#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
import re
import threading
import tkinter as tk

import numpy as np
import speech_recognition as sr

import kinematics as km
import shake_strategy_content as shake_cont


current_pos=np.array([0, 368, 113.5, -90, 0, 0])
next_pos=np.array([0, 368, 113.5, -90, 0, 0])
def SingleMovement():
    sin_mov_int = tk.Toplevel()
    sin_mov_int.title('Single Movement')
    sin_mov_int.geometry('500x425')
    sin_mov_int.wm_attributes("-topmost",1)

    #variables
    pos_entry=[]
    inpt_frm=tk.Frame(sin_mov_int)
    inpt_frm.pack()
    add_frm=tk.Frame(sin_mov_int)
    add_frm.pack()
    rb_frm_l=tk.Frame(add_frm)
    rb_frm_l.pack(side='left')
    rb_frm_r=tk.Frame(add_frm)
    rb_frm_r.pack(side='left')
    pos_list=[np.array([0, 368, 113.5, -90, 0, 0])]
    pos_select_idx=tuple((-1,))
    print(pos_select_idx)
    #functions
    def Add():
        global pos_select_idx
        if type_var.get():
            if pos_var.get():
                pos_list.append(np.array([float(pos_entry[i].get() if pos_entry[i].get() else 0) for i in range(6)]))
            else:
                pos_list[0]=np.array([float(pos_entry[i].get()) if pos_entry[i].get() else 0 for i in range(6)])
        else :
            pos=km.HiwinRA605_Model.arm_center.ForwardKinematics(np.array([float(pos_entry[i].get() if pos_entry[i].get() else 0) for i in range(6)]))
            if pos_var.get():
                pos_list.append(pos)
            else:
                pos_list[0]=pos
        pos_listbox.delete(0)
        pos_listbox.insert(0,'From: '+str(np.around(pos_list[0],decimals=2)))
        if pos_var.get():
            pos_listbox.insert(tk.END,'To: '+str(np.around(pos_list[-1],decimals=2)))
        pos_select_idx=tuple((-1,))
    def Mov():
        global pos_select_idx,current_pos
        for pos in range(len(pos_list)-1):
            if shake_cont.analyse_mode:
                km.HiwinRA605_Simu.AdvSimu(pos_list[pos],pos_list[pos+1],0,shake_cont.speed_value,shake_cont.simu_mode)
            else:
                km.HiwinRA605_Simu.MySimuAni(pos_list[pos],pos_list[pos+1],0,shake_cont.speed_value,shake_cont.simu_mode)
            current_pos=pos_list[pos+1]
        pos_select_idx=tuple((-1,))
    def CurSelect(event):
        global pos_select_idx
        pos_select_idx=pos_listbox.curselection()
    def Clear():
        global pos_select_idx
        pos_listbox.delete(0,tk.END)
        pos_list[0]=np.array([0, 368, 113.5, -90, 0, 0])
        for i in range(len(pos_list)-1):    
            del pos_list[-1]
        pos_listbox.insert(0,'From: '+str(np.around(pos_list[0],decimals=2)))
        pos_select_idx=tuple((-1,))
    def DelPos():
        global pos_select_idx
        if pos_select_idx[0]:
            if pos_select_idx[0]==-1:
                pos_listbox.delete(tk.END)
            else:
                pos_listbox.delete(pos_select_idx)
            # print(pos_list[pos_select_idx[0]])
            del pos_list[pos_select_idx[0]]
        if pos_select_idx[0]==0 or len(pos_list)==1:    
            pos_listbox.delete(0)
            pos_list[0]=np.array([0, 368, 113.5, -90, 0, 0])
            pos_listbox.insert(0,'From: '+str(np.around(pos_list[0],decimals=2)))
        pos_select_idx=tuple((-1,))
    def VoiceCtrl():
        global current_pos,next_pos
        tk.messagebox.showinfo(message='必須以“左/右50，前/後50，上/下50”的順序說出。')
        while(1):
        # Record Audio
            r = sr.Recognizer()
            with sr.Microphone() as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                print("Say something!")
                audio = r.listen(source)
        # Speech recognition using Google Speech Recognition
            try:
                voice_cmd = r.recognize_google(audio,language = 'zh-TW')
                print(voice_cmd)
                if voice_cmd.find(u'結束') != -1:
                    break
                #voice_cmd = "x前進5公分 y後退4公分 z上升3公分 z下降2公分"
                digit = re.finditer(r"\d+",voice_cmd)
                dig=[]
                for match in digit: 
                    dig.append(float(match.group()))
                    print (match.group())
                print(dig)
                str_cmd=re.sub(r'\d',"", voice_cmd)
                print(str_cmd)
                dir_str='左 右 前 後 上 下'
                # dir_var=[x,x,y,y,z,z]
                dir_sgn=[-1,1,1,-1,1,-1]
                pos_c=[0.0,0.0,0.0,0.0,0.0,0.0]
                dir_cnt=0
                for dir in range(len(str_cmd)):
                    if str_cmd[dir] in dir_str.split():
                        pos_c[dir]=dig[dir_cnt]*dir_sgn[dir]
                        dir_cnt+=1
                        if dir_cnt==len(dig):
                            break
                # print([pos_c[0]+pos_c[1],pos_c[2]+pos_c[3],pos_c[4]+pos_c[5]])
                # return [pos_c[0]+pos_c[1],pos_c[2]+pos_c[3],pos_c[4]+pos_c[5]]
                for i in range(3):
                    next_pos[i]=current_pos[i]+pos_c[2*i]+pos_c[2*i+1]
                
                if shake_cont.Arm_connected:
                    crnt_pos=shake_cont.Point_cls(current_pos[0],current_pos[1],current_pos[2],current_pos[3],current_pos[4],current_pos[5])
                    nxt_pos=shake_cont.Point_cls(current_pos[0],current_pos[1],current_pos[2],current_pos[3],current_pos[4],current_pos[5])
                    import shake_strategy_trigger as shake_trig
                    shake_trig.Hiwin_Action.ArmMove(crnt_pos,shake_cont.speed_value,0,'')
                    shake_trig.Hiwin_Action.ArmMove(nxt_pos,shake_cont.speed_value,0,'')
                    del shake_trig
                elif shake_cont.analyse_mode:
                    km.HiwinRA605_Simu.AdvSimu(current_pos,next_pos,0,shake_cont.speed_value,shake_cont.simu_mode)
                else:
                    km.HiwinRA605_Simu.MySimuAni(current_pos,next_pos,0,shake_cont.speed_value,shake_cont.simu_mode)
                current_pos=next_pos
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

    def Audio():
        global current_pos,next_pos
        audio_thread=threading.Thread(target=VoiceCtrl)
        audio_thread.start()

    #GUI
    for i in range(6):
        pos_entry.append(tk.Entry(inpt_frm,width=6))
        pos_entry[i].pack(side='left',padx=10,pady=5)
    
    type_var=tk.IntVar(value=1)
    ang_rb=tk.Radiobutton(rb_frm_l,text='Angle',variable=type_var, value=0,font=('Arial', 13))
    ang_rb.pack(padx=5,pady=2)
    pos_rb=tk.Radiobutton(rb_frm_r,text='Position',variable=type_var, value=1,font=('Arial', 13))
    pos_rb.pack(padx=5,pady=2)

    pos_var=tk.IntVar(value=1)
    start_rb=tk.Radiobutton(rb_frm_l,text='Start',variable=pos_var, value=0,font=('Arial', 13))
    start_rb.pack(padx=5,pady=2)
    dest_rb=tk.Radiobutton(rb_frm_r,text='Destination',variable=pos_var, value=1,font=('Arial', 13))
    dest_rb.pack(padx=5,pady=2)

    add_btn = tk.Button(add_frm,text='Add',font=('Arial', 15),width=6,height=1,command=Add) 
    add_btn.pack(side='left',padx=10,pady=5)

    pos_listbox=tk.Listbox(sin_mov_int,width=46,height=14,font=('Arial', 13))
    pos_listbox.bind('<<ListboxSelect>>',CurSelect)
    pos_listbox.pack()
    pos_listbox.insert(0,'From: '+str(np.around(pos_list[0],decimals=2)))

    cmd_frm=tk.Frame(sin_mov_int)
    cmd_frm.pack()

    clr_btn = tk.Button(cmd_frm,text='Clear',font=('Arial', 15),width=5,height=1,command=Clear) 
    clr_btn.pack(side='left',padx=14,pady=5)

    del_btn = tk.Button(cmd_frm,text='Delete',font=('Arial', 15),width=5,height=1,command=DelPos) 
    del_btn.pack(side='left',padx=14,pady=5)

    spc_btn = tk.Button(cmd_frm,text='Audio',font=('Arial', 15),width=5,height=1,command=Audio) 
    spc_btn.pack(side='left',padx=14,pady=5)

    mov_btn = tk.Button(cmd_frm,text='Move',font=('Arial', 15),width=5,height=1,command=Mov) 
    mov_btn.pack(side='left',padx=14,pady=5)

    sin_mov_int.mainloop()

# SingleMovement()
