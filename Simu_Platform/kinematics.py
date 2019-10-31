#!/usr/bin/env python3
# license removed for brevity
#encoding:utf-8
# forward/inverse kinematics & p2p simulation for hiwin manipulator RA605
# ============================================================================
# =14/07/2019:add inverse position tester, pos searcher, data sorter         =
# =14/07/2019:add forward position tester(by force)                          =
# =13/07/2019:pack kinematic function into class, whole manipulator drawer in=
# =           class, simulation in class, add grip & outline & speed control =
# =           add three views of plot                                        =
# =12/07/2019:fix inv-kine angle err & add check whether angles are in range =
# ============================================================================
import math
import re
import time

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation  # 動畫
from mpl_toolkits.mplot3d import Axes3D, proj3d
from numpy import arctan2 as atan2
from numpy import cos, pi, sin
import shake_strategy_content as shake_cont
# 另一種3d視圖
# def orthogonal_proj(zfront, zback):
#     a = (zfront+zback)/(zfront-zback)
#     b = -2*(zfront*zback)/(zfront-zback)
#     return np.array([[1,0,0,0],
#                     [0,1,0,0],
#                     [0,0,a,b],
#                     [0,0,0,zback]])
# proj3d.persp_transformation = orthogonal_proj

fig_inited=False
ani_plt=0
ang_plt=0
#  =========================================================================
#                                 Functions 
#  =========================================================================
def cosd(deg):
    return cos(np.deg2rad(deg))
def sind(deg):
    return sin(np.deg2rad(deg))
def cosd_equal_to(deg,val):
    return cosd(deg) >= val-1e-4 and cosd(deg) <= val+1e-4
def elem(matrix,col,row):
    return matrix[col-1,row-1]
theta_error_set=set()
def ReformTheta(theta,neg_lim_deg,pos_lim_deg):
    if theta > pos_lim_deg/180*pi:
        theta -= 2*pi
    if theta < neg_lim_deg/180*pi:
        theta += 2*pi
    return theta
def ThetaInRange(theta,neg_lim_deg,pos_lim_deg,theta_name):
    theta_reformed=ReformTheta(theta,neg_lim_deg,pos_lim_deg)
    if theta_reformed > pos_lim_deg/180*pi or theta < neg_lim_deg/180*pi :
        theta_error_set.add(theta_name)
    return theta_reformed
def AngInput():
    print('請輸入各軸角度：(deg):')
    ang= np.array([float(i) for i in input().split()])
    return ang
def PosInput(str):
    print('請輸入'+str+'末端點位置：(mm,deg):')
    pos= np.array([float(i) for i in input().split()])
    return pos
#  =========================================================================
#                                 Kinematics 
#  =========================================================================
class Kinematics_Model_cls():
    def __init__(self,DoF,param_L,param_a,param_alpha,param_d,param_theta,orig_pos,axis_limit):
        self.DoF=DoF
        self.param_L=param_L
        self.param_a=param_a
        self.param_alpha=param_alpha
        self.param_d=param_d
        self.param_theta=param_theta
        self.orig_sm=np.eye(4)
        self.orig_sm[0:3,3]=np.array([orig_pos])
        self.DH_orig=np.array([self.param_a,
                               self.param_alpha,
                               self.param_d,
                               self.param_theta])
        self.axis_limit=axis_limit
    def L(self,i):
        return self.param_L[i-1]
    def a(self,i):
        return self.param_a[i-1]  
    def GenerateAm(self,DoF,DH_new):#this DoF is independent!!!!
        Am=np.zeros((0,4))
        for i in range(1,DoF+1):
            ai=elem(DH_new,1,i); alphai=elem(DH_new,2,i)
            di=elem(DH_new,3,i); thetai=elem(DH_new,4,i)
            Am=np.vstack((Am,np.array([[cos(thetai), -sin(thetai)*cos(alphai), sin(thetai)*sin(alphai), ai*cos(thetai)],
                                        [sin(thetai), cos(thetai)*cos(alphai), -cos(thetai)*sin(alphai), ai*sin(thetai)],
                                        [0, sin(alphai), cos(alphai), di],
                                        [0, 0, 0, 1]])))
        return Am      
    def GetAmi(self,i,Am):
        return Am[4*i-4:4*i,:]
    def TmTo(self,m,n,Am):
        Tm2=np.eye(4)
        for i in range(m+1,n+1):
            Tm2=np.dot(Tm2,self.GetAmi(i,Am))
        return Tm2
    def GetRmFromPRY(self,PRY):
        pry=(PRY+[0, -180, 0]) * pi / 180

        p=pry[0];r=pry[1];y=pry[2]
        r_yaw=np.array([[cos(y),-sin(y),0],
                        [sin(y), cos(y),0],
                        [0     , 0     ,1]])
        r_pitch=np.array([[1  ,0      ,0],
                        [0  ,cos(p) ,-sin(p)],
                        [0  ,sin(p) ,cos(p)]])
        r_roll=np.array([[cos(r),0  ,-sin(r)],
                        [0     ,1  ,0],
                        [sin(r),0  ,cos(r)]])
        rm=np.dot(np.dot(r_yaw, r_pitch), r_roll)
        return np.dot(rm,np.array([[0,1,0],[0,0,1],[1,0,0]]))
    def RmTo(self,m,n,Am):
        return self.TmTo(m,n,Am)[0:3,0:3]
    def GetRm0to3(self, theta1, theta2, theta3):
        DH_0to3=self.DH_orig+np.vstack( ( np.zeros((3,6)) , 
                         np.hstack( ( np.array([theta1, theta2, theta3]) , np.zeros((1,3))[0,:] ) ) ) )
        Am=self.GenerateAm(3,DH_0to3)
        return self.RmTo(0,3,Am)
    def GetRm3to6(self, theta1, theta2, theta3, Rm0to6):
        Rm0to3=self.GetRm0to3(theta1, theta2, theta3)
        return np.dot(Rm0to3.T,Rm0to6)
    def ForwardKinematics(self,JointAngle):
        DH_new=self.DH_orig+np.vstack((np.zeros((3,self.DoF)),JointAngle * pi / 180))
        ##生成齊次轉換矩陣
        Am=self.GenerateAm(self.DoF,DH_new)
        Tm=np.dot(self.orig_sm,self.TmTo(0,self.DoF,Am))
        ##正運動學  
        position=Tm[0:3,3].T-[0, 0, self.L(1)]
        Rm0to6=Tm[0:3,0:3]
        pitch=atan2(elem(Rm0to6,3,3),np.sqrt(1-elem(Rm0to6,3,3)**2))
        if cosd_equal_to(pitch/pi*180,0):
            roll=sin(pitch)*atan2(elem(Rm0to6,2,2),elem(Rm0to6,1,2))
            yaw=0
        else:
            yaw=atan2(-elem(Rm0to6,1,3),elem(Rm0to6,2,3))
            roll=atan2(-elem(Rm0to6,3,2),elem(Rm0to6,3,1))
        PRY=np.array([pitch,roll,yaw])/pi *180-[0, 180, 0]

        pos=np.hstack((position, PRY))
        return pos  
    def InverseKinematics(self,inputPosition): 
        ##逆運動學
        theta_error_set.clear()
        #末端點坐標
        od= ( inputPosition[0:3]+[0, 0, self.L(1)] ).T
        # print('末端點坐標:',np.around(od,decimals=1))
        #456原點坐標
        Rm0to6=self.GetRmFromPRY(inputPosition[3:6])
        oc=od-self.L(4)*np.dot(Rm0to6,np.array([0,0,1]).T)
        # print('關節點坐標:',np.around(oc,decimals=1))
        xc=oc[0];yc=oc[1];zc=oc[2]
        #theta1
        theta1=np.array([[atan2(-xc,yc)],
                        [atan2(xc,-yc)]]) 
        for i in range(1):
            theta1[i]=ThetaInRange(theta1[i],self.axis_limit[0,0],self.axis_limit[0,1],'theta1_{}'.format(i+1))

        #theta3
        angle_a3_l3=atan2(-self.a(3),self.L(3))
        diag_a3_l3_pow2=self.L(3)**2+self.a(3)**2
        diag_a3_l3=np.sqrt(diag_a3_l3_pow2)
        L_axis2_c_in_xy=np.array([[np.sqrt(xc**2+yc**2)-self.a(1)],[np.sqrt(xc**2+yc**2)+self.a(1)]])
        cos_angle_axis234=(L_axis2_c_in_xy**2+(zc-self.L(1))**2-(self.L(2)**2+diag_a3_l3_pow2))/(-2*self.L(2)*diag_a3_l3)
        angle_axis234=atan2(np.sqrt(1-cos_angle_axis234**2),cos_angle_axis234)
        theta3=np.vstack((angle_axis234-pi/2-angle_a3_l3,
                        -angle_axis234+pi*3/2-angle_a3_l3))
        for i in range(1):
            theta3[i]=ThetaInRange(theta3[i],self.axis_limit[2,0],self.axis_limit[2,1],'theta3_{}'.format(i+1))
        #theta2
        theta2=np.vstack((atan2( diag_a3_l3*sin(pi-angle_axis234),self.L(2)+diag_a3_l3*cos(pi-angle_axis234) )+\
                            atan2(zc-self.L(1),L_axis2_c_in_xy)-pi/2,
                        -atan2( diag_a3_l3*sin(pi-angle_axis234),self.L(2)+diag_a3_l3*cos(pi-angle_axis234) )+\
                            atan2(zc-self.L(1),L_axis2_c_in_xy)-pi/2))
        for i in range(1):
            theta2[i]=ThetaInRange(theta2[i],self.axis_limit[1,0],self.axis_limit[1,1],'theta2_{}'.format(i+1))

        angle_sol_rad=np.zeros((16,6))
        angle_sol_rad[0,0:3]=np.hstack((theta1[0],theta2[0],theta3[0]))
        angle_sol_rad[1,0:3]=np.hstack((theta1[0],theta2[2],theta3[2]))
        angle_sol_rad[2,0:3]=np.hstack((theta1[1],-theta2[1],theta3[3]))
        angle_sol_rad[3,0:3]=np.hstack((theta1[1],-theta2[3],theta3[1]))
        angle_sol_rad[4,0:3]=np.hstack((theta1[0],theta2[0],theta3[0]))
        angle_sol_rad[5,0:3]=np.hstack((theta1[0],theta2[2],theta3[2]))
        angle_sol_rad[6,0:3]=np.hstack((theta1[1],-theta2[1],theta3[3]))
        angle_sol_rad[7,0:3]=np.hstack((theta1[1],-theta2[3],theta3[1]))

        angle_sol_deg=angle_sol_rad/pi *180
        #theta456
        for sols in range(1):
            Rm0to3=self.GetRm0to3(angle_sol_rad[sols,0],angle_sol_rad[sols,1],angle_sol_rad[sols,2])
            Rm3to6=np.dot(Rm0to3.T,Rm0to6)
            theta5=atan2(np.sqrt(1-elem(Rm3to6,3,3)**2),elem(Rm3to6,3,3))
            if cosd_equal_to(theta5/pi*180,-1) or cosd_equal_to(theta5/pi*180,1):
                theta4 = 0
                theta6 = sin(theta5)*atan2(elem(Rm3to6,2,1),elem(Rm3to6,1,1))
            else:
                theta4=atan2(-elem(Rm3to6,2,3),-elem(Rm3to6,1,3))+pi
                theta5=-theta5
                theta6=atan2(-elem(Rm3to6,3,2),elem(Rm3to6,3,1))+pi
            theta4_1=ThetaInRange(theta4,self.axis_limit[3,0],self.axis_limit[3,1],'theta4_1')
            theta5_1=ThetaInRange(theta5,self.axis_limit[4,0],self.axis_limit[4,1],'theta5_1')
            theta6_1=ThetaInRange(theta6,self.axis_limit[5,0],self.axis_limit[5,1],'theta6_1')
            # theta4_2=ThetaInRange(theta4-pi,self.axis_limit[3,0],self.axis_limit[3,1],'theta4_2')
            # theta5_2=ThetaInRange(-theta5,self.axis_limit[4,0],self.axis_limit[4,1],'theta5_2')
            # theta6_2=ThetaInRange(theta6-pi,self.axis_limit[5,0],self.axis_limit[5,1],'theta6_2')
            if theta_error_set:
                print(theta_error_set)
            angle_sol_rad[sols,3:6]=np.hstack((theta4_1,theta5_1,theta6_1))
            # angle_sol_rad[sols+4,3:6]=np.hstack((theta4_2,theta5_2,theta6_2))
            angle_sol_deg=angle_sol_rad/pi *180
            # print('angle_sol_deg',sols,':',np.around(angle_sol_deg[sols],decimals=1))
        angle=angle_sol_deg[0]
        return angle
    def PreDraw(self,angle):
        ##DH參數和齊次轉換矩陣
        DH_new=self.DH_orig+np.vstack((np.zeros((3,self.DoF)),angle * pi / 180))
        # print('DH_new:',np.around(DH_new,decimals=1))
        Am=self.GenerateAm(self.DoF,DH_new)

        node=self.orig_sm[0:3,3]
        for i in range(1,self.DoF+1):
            Tm0toi=self.TmTo(0,i,Am)
            Sm=np.dot(self.orig_sm,Tm0toi)
            # print(np.around(Sm,decimals=1))
            # print('Node_{}_pos:'.format(i),np.around(Tm0toi[0:3,3],decimals=1))
            node=np.vstack((node,Sm[0:3,3]))
        return node[:,0:3]
    def KinematicsDraw(self,angle,clr_typ,line_width,perspective):
        node=self.PreDraw(angle)
        x=np.array([elem(node,axis,1) for axis in range(1,self.DoF+2)])
        y=np.array([elem(node,axis,2) for axis in range(1,self.DoF+2)])
        z=np.array([elem(node,axis,3) for axis in range(1,self.DoF+2)])
        if perspective == '3D':
            plt.plot(x,y,z,clr_typ,lw=line_width)
        elif perspective == 'FRONT':
            plt.plot(x,z,clr_typ,lw=line_width)
        elif perspective == 'TOP':
            plt.plot(x,y,clr_typ,lw=line_width)
        elif perspective == 'LATERAL':
            plt.plot(y,z,clr_typ,lw=line_width)
        plt.axis('square')
        plt.axis('equal')
        return node[:,0:3]
    
#  =========================================================================
#                         RobotManipulator Drawer
#  =========================================================================
class Arm_Model():
    def __init__(self,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,width,grip_l,grip_r,grip_ctrl,axis_limit):
        self.arm_center=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[0,0,0],axis_limit)
        self.grip_l=grip_l
        self.grip_r=grip_r
        self.edge_0=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[width/2,width/2,width/2],axis_limit)
        self.edge_1=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[width/2,width/2,-width/2],axis_limit)
        self.edge_2=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[width/2,-width/2,width/2],axis_limit)
        self.edge_3=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[width/2,-width/2,-width/2],axis_limit)
        self.edge_4=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[-width/2,width/2,width/2],axis_limit)
        self.edge_5=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[-width/2,width/2,-width/2],axis_limit)
        self.edge_6=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[-width/2,-width/2,width/2],axis_limit)
        self.edge_7=Kinematics_Model_cls(6,arm_center_L,arm_center_a,arm_center_alpha,arm_center_d,arm_center_theta,[-width/2,-width/2,-width/2],axis_limit)
        self.arm_edge=[self.edge_0,self.edge_1,self.edge_2,self.edge_3,self.edge_4,self.edge_5,self.edge_6,self.edge_7]
        self.grip_off=grip_ctrl[0]
        self.grip_half=grip_ctrl[1]
        self.grip_on=grip_ctrl[2]
        self.current_plt=0
        self.neg_lim_deg=axis_limit[:,0]
        self.pos_lim_deg=axis_limit[:,1]

    def ThreeViewsDraw(self,angle,grip_angle,title,sbplt,perspective):
        if perspective == '3D':
            self.current_plt=plt.subplot(sbplt,projection='3d', aspect='equal')  #創建一個三维的繪圖
            self.current_plt.clear()
            plt.title(title)
            self.current_plt.set_zlabel('Z')  # 座標軸
            self.current_plt.set_ylabel('Y')
            self.current_plt.set_xlabel('X')
        elif perspective == 'FRONT':
            self.current_plt=plt.subplot(sbplt, aspect='equal')  
            self.current_plt.clear()
            plt.title('Front')
            self.current_plt.set_ylabel('Z')  
            self.current_plt.set_xlabel('X')
        elif perspective == 'TOP':
            self.current_plt=plt.subplot(sbplt, aspect='equal')  
            self.current_plt.clear()
            plt.title('Top')
            self.current_plt.set_ylabel('Y')
            self.current_plt.set_xlabel('X')
        elif perspective == 'LATERAL':
            self.current_plt=plt.subplot(sbplt, aspect='equal') 
            self.current_plt.clear()
            plt.title('Lateral')
            self.current_plt.set_ylabel('Z')  
            self.current_plt.set_xlabel('Y')
        self.current_plt.grid(True)
        ##----------outline drawer----------
        # outline=np.zeros((56,3))
        # for i in range(8):
        #     outline[7*i:7*(i+1),0:3]=self.arm_edge[i].KinematicsDraw( angle,'b--',1.5,perspective)
        # #draw axis plane
        # for i in range(7):
        #     x=np.array([outline[axis*7+i,0] for axis in range(8)])
        #     y=np.array([outline[axis*7+i,1] for axis in range(8)])
        #     z=np.array([outline[axis*7+i,2] for axis in range(8)])
        #     if perspective == '3D':
        #         plt.plot(x,y,z,'b-',lw=1.5)
        #     elif perspective == 'FRONT':
        #         plt.plot(x,z,'b-',lw=1.5)
        #     elif perspective == 'TOP':
        #         plt.plot(x,y,'b-',lw=1.5)
        #     elif perspective == 'LATERAL':
        #         plt.plot(y,z,'b-',lw=1.5)
        ##----------outline drawer end----------
        self.grip_l.KinematicsDraw( np.hstack((angle,np.array([grip_angle,0]))),'r-',3,perspective)
        self.grip_r.KinematicsDraw( np.hstack((angle,np.array([grip_angle,0]))),'r-',3,perspective)
        self.arm_center.KinematicsDraw(angle,'bo',2,perspective)
    def DrawRobotManipulator(self,angle,grip_angle,title,multi):
        if multi == 1:
            self.ThreeViewsDraw(angle,grip_angle,title,221,'TOP')
            self.ThreeViewsDraw(angle,grip_angle,title,222,'3D')
            self.ThreeViewsDraw(angle,grip_angle,title,223,'FRONT')
            self.ThreeViewsDraw(angle,grip_angle,title,224,'LATERAL')
        elif multi == 21:
            self.ThreeViewsDraw(angle,grip_angle,title,241,'TOP')
            self.ThreeViewsDraw(angle,grip_angle,title,242,'3D')
            self.ThreeViewsDraw(angle,grip_angle,title,245,'FRONT')
            self.ThreeViewsDraw(angle,grip_angle,title,246,'LATERAL')
        elif multi == 22:
            self.ThreeViewsDraw(angle,grip_angle,title,243,'TOP')
            self.ThreeViewsDraw(angle,grip_angle,title,244,'3D')
            self.ThreeViewsDraw(angle,grip_angle,title,247,'FRONT')
            self.ThreeViewsDraw(angle,grip_angle,title,248,'LATERAL')

#  =========================================================================
#                         RobotManipulator Simulation
#  =========================================================================
current_pos_array=np.array([0, 368, 113.5, -90, 0, 0 ])
op_factor=0.1
class Simu_Ani_cls():
    def __init__(self,Arm_Model):
        self.Arm_Model = Arm_Model
        self.simu_mode = True # true for p2p, false for linear
        self.start_pos = np.array([0, 368, 113.5, -90, 0, 0],dtype=float)
        self.start_angle = np.array([0, 0, 0, 0, -90, 0],dtype=float)
        self.d_pos = np.array([0, 0, 0, 0, 0, 0],dtype=float)
        self.d_angle = np.array([0, 0, 0, 0, 0, 0],dtype=float)
        self.grip_angle = 0 
        self.d_grip = 0 
        self.neg_lim_deg=Arm_Model.neg_lim_deg
        self.pos_lim_deg=Arm_Model.pos_lim_deg
        self.fps=30
        self.nodestack=np.zeros((7,0))
        self.ang_deg_stack=np.zeros(6)
        self.ang_vel_stack=np.zeros(6)
        self.ang_acc_stack=np.zeros(6)
        self.ang_acc_lim = np.zeros(6) #degree per second square
        self.ang_vel_lim = np.zeros(6) #degree per second
        self.current_ang_vel=np.zeros(6)
        self.current_ang_acc=np.zeros(6)
        self.all_lin_vel=np.zeros(1)
        self.my_factor=1

    # animate
    def animate(self,i):
        global current_pos_array
        if self.simu_mode:   #true for p2p
            self.start_angle += self.d_angle
            self.grip_angle += self.d_grip
            self.start_pos=self.Arm_Model.arm_center.ForwardKinematics(self.start_angle)
        else:           #false for linear
            current_pos_array=self.start_pos
            self.start_pos += self.d_pos
            self.grip_angle += self.d_grip
            self.start_angle=self.Arm_Model.arm_center.InverseKinematics(self.start_pos)
        self.Arm_Model.DrawRobotManipulator(self.start_angle,self.grip_angle,'Kinematics Simulation',1)
    # init_plot
    def init(self):
        self.start_angle=self.Arm_Model.arm_center.InverseKinematics(self.start_pos)
        self.Arm_Model.DrawRobotManipulator(self.start_angle,self.grip_angle,'Kinematics Simulation',1)
    def MySimuAni(self,begin_pos,end_pos,grip_angle,speed,simu_p2p):
        global fig_inited,ani_plt
        self.start_pos = begin_pos
        print('起點:',self.start_pos,'\n終點:',end_pos)
        self.simu_mode = simu_p2p
        if self.simu_mode:   #true for p2p
            start_angle=self.Arm_Model.arm_center.InverseKinematics(self.start_pos)
            end_angle=self.Arm_Model.arm_center.InverseKinematics(end_pos)
            # print(start_angle[5],end_angle[5])
            if 180<abs(end_angle[5]-start_angle[5])<360:
                if end_angle[5]>start_angle[5]:
                    end_angle[5]-=360
                elif end_angle[5]<start_angle[5]:
                    start_angle[5]-=360
            # print(start_angle[5],end_angle[5])
            print('起點:',np.around(self.start_angle,decimals=2),'\n終點:',np.around(end_angle,decimals=2))
            num = int(np.max(np.maximum(start_angle,end_angle)-np.minimum(start_angle,end_angle))/(float(speed)*(1/12)))
            if num == 0:
                if self.grip_angle != grip_angle:
                    num = int(abs(grip_angle-self.grip_angle)//10)
                    if num ==0:
                        num=1
                else:
                    num = 1
            print('num:',num)
            self.d_angle = (end_angle - start_angle)/num
        else:           #false for linear
            num = int(np.linalg.norm(self.start_pos-end_pos)/(float(speed)**(2/3)))
            if num == 0:
                if self.grip_angle != grip_angle:
                    num = int(abs(grip_angle-self.grip_angle)//10)
                    if num ==0:
                        num=1
                else:
                    num = 1
            print('num:',num)
            self.d_pos = (end_pos - self.start_pos)/num
        self.d_grip=(grip_angle-self.grip_angle)/num
        if not fig_inited:
            ani_plt = plt.figure(figsize=(10,10))
            fig_inited=True
        my_ani = animation.FuncAnimation(ani_plt,
                                func = self.animate,
                                frames=num,
                                init_func=self.init,
                                interval=100/self.fps,
                                repeat=False)
        # plt.show()
        plt.ion()
        plt.pause(0.05)
        # my_ani.save('hiwin_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    def AdvSimu(self,begin_pos,end_pos,grip_angle,speed,simu_p2p):
        global fig_inited,ani_plt,ang_plt
        self.start_pos = begin_pos
        print('起點:',self.start_pos,'\n終點:',end_pos)
        self.simu_mode = simu_p2p
        if self.simu_mode:   #true for p2p
            
            ##-------direction decide-----------
            self.start_angle=self.Arm_Model.arm_center.InverseKinematics(self.start_pos)
            end_angle=self.Arm_Model.arm_center.InverseKinematics(end_pos)
            # print(start_angle[5],end_angle[5])
            if 180<abs(end_angle[5]-self.start_angle[5])<360:
                if end_angle[5]>self.start_angle[5]:
                    end_angle[5]-=360
                elif end_angle[5]<self.start_angle[5]:
                    self.start_angle[5]-=360
            # print(start_angle[5],end_angle[5])
            print('起點:',np.around(self.start_angle,decimals=2),'\n終點:',np.around(end_angle,decimals=2))

            ##------------frame calculate---------------
            self.nodestack=np.zeros((7,0))
            self.all_lin_vel=np.zeros(1)
            self.ang_acc_lim = np.array([360, 288, 420, 444, 450, 720],dtype=float)*shake_cont.acc_rate/100*op_factor*self.my_factor
            self.ang_vel_lim = np.array([360, 288, 420, 444, 450, 720],dtype=float)*shake_cont.speed_value/100*op_factor
            self.ang_deg_stack=self.start_angle
            self.ang_vel_stack=np.zeros(6)
            acc_per_frm=self.ang_acc_lim/(self.fps**2)
            t_acc=(self.ang_vel_lim/self.fps/acc_per_frm)
            ang_asd=0.5*acc_per_frm*np.power(t_acc,2)
            t_eql=((end_angle-self.start_angle)-ang_asd*2)/(self.ang_vel_lim/self.fps)
            frm_num=0
            for axis in range(6):
                if t_eql[axis]<0: 
                    t_acc[axis]=np.minimum(t_acc[axis],np.sqrt(np.abs(end_angle[axis]-self.start_angle[axis])/acc_per_frm[axis]))
            for axis in range(6):
                if t_eql[axis]<0:
                    frm_num=max(frm_num,t_acc[axis]*2) 
                else:
                    frm_num=max(frm_num,t_acc[axis]*2+t_eql[axis])
            frm_num=int((frm_num//2+1)*2)
            # print('a:',acc_per_frm)
            # print('t_a',np.around(t_acc,decimals=3))
            # print('ang_a:',ang_asd)
            # print('ang_d',ang_des)
            # print('t_e',t_eql)
            # print('frm',frm_num)
            # print()
            for axis in range(6):
                if t_eql[axis]<0: 
                    t_eql[axis]=0
                    # t_acc[axis]=np.minimum(t_acc[axis],np.sqrt(np.abs(end_angle[axis]-self.start_angle[axis])/acc_per_frm[axis]))
                    # print('ta:',t_acc[axis])
                    # print('--------')

                else:
                    t_eql[axis]=np.floor(np.sqrt(frm_num**2-4*np.abs(end_angle[axis]-self.start_angle[axis])/acc_per_frm[axis])/2)*2
                    t_acc[axis]=(frm_num-t_eql[axis])//2
                    acc_per_frm[axis]=np.abs(end_angle[axis]-self.start_angle[axis])/t_acc[axis]/(t_acc[axis]+t_eql[axis])
                    # print('actual',(t_eql[axis]+frm_num)*(frm_num-t_eql[axis])/2*acc_per_frm[axis]/2)
                     
                    # print('t_e=',t_eql[axis])
            for axis in range(6):
                t_acc[axis]=np.ceil(np.max(t_acc))
                t_eql[axis]=frm_num-t_acc[axis]*2
                acc_per_frm[axis]=np.abs(end_angle[axis]-self.start_angle[axis])/t_acc[axis]/(t_acc[axis]+t_eql[axis])
            # print('a:',acc_per_frm)
            # print('t_e',t_eql)
            # print('t_a',np.around(t_acc,decimals=3))
            # print('a:',acc_per_frm)
            # print('t_a:',t_acc)
            # frm_num=int(np.max(t_acc*2+3+t_eql))
            print('frm',frm_num)
            acc_time_lb=[0]
            vel_time_lb=[0]
            ang_time_lb=[0]
            for frm in range(frm_num):
                vel_time_lb.append(frm+0.5)
                ang_time_lb.append(frm+1)
            vel_time_lb.append(frm_num)
            self.ang_acc_stack=np.zeros(6)
            if not np.linalg.norm(end_angle-self.start_angle):
                frm_num=int(1)
                acc_time_lb=[0,1]
                vel_time_lb=[0,1]
                ang_time_lb=[0,1]
                self.start_pos=self.Arm_Model.arm_center.ForwardKinematics(self.start_angle)
                self.nodestack=np.hstack((self.nodestack,self.Arm_Model.arm_center.PreDraw(self.start_angle))) 
                self.ang_deg_stack=np.vstack((self.ang_deg_stack,self.start_angle)) 
                self.ang_vel_stack=np.vstack((self.ang_vel_stack,np.zeros(6)))  
                self.ang_acc_stack=np.vstack((self.ang_acc_stack,np.zeros(6)))  
                self.all_lin_vel=np.hstack((self.all_lin_vel,0))
            else:
                for frm in range(frm_num+1):
                    # print('frm:',frm)
                    if frm==np.max(t_acc+t_eql) and np.max(t_eql)!=0:
                        self.ang_acc_stack=np.vstack((self.ang_acc_stack,self.current_ang_acc))
                        acc_time_lb.append(frm)
                    if frm==frm_num or frm==np.max(t_acc):
                        self.ang_acc_stack=np.vstack((self.ang_acc_stack,self.current_ang_acc))
                        acc_time_lb.append(frm-0.1)
                    for axis in range(6):
                        sgn=np.sign(end_angle[axis]-self.start_angle[axis])
                        acc_sgn=0
                        # print(sgn)
                        if frm<t_acc[axis]:
                            acc_sgn=1
                        elif frm==t_acc[axis]:
                            if t_eql[axis]<=0: # no equal segment
                                acc_sgn=-1
                        elif t_acc[axis]+t_eql[axis]<=frm<frm_num:
                            acc_sgn=-1

                        self.start_angle[axis]+=self.current_ang_vel[axis]+0.5*acc_per_frm[axis]*sgn*acc_sgn
                        self.current_ang_vel[axis]+=acc_per_frm[axis]*sgn*acc_sgn
                        # print(self.current_ang_acc)
                        
                        self.current_ang_acc[axis]=acc_per_frm[axis]*(self.fps**2)*sgn*acc_sgn/self.my_factor
                    self.nodestack=np.hstack((self.nodestack,self.Arm_Model.arm_center.PreDraw(self.start_angle)))
                    # print(self.ang_deg_stack,'\n',self.start_angle)
                    if frm==0 or frm==np.max(t_acc+t_eql):
                        self.ang_acc_stack=np.vstack((self.ang_acc_stack,self.current_ang_acc))
                        acc_time_lb.append(frm+0.1)
                    if frm==np.max(t_acc) and np.max(t_eql)!=0:
                        self.ang_acc_stack=np.vstack((self.ang_acc_stack,self.current_ang_acc))
                        acc_time_lb.append(frm)
                    if frm!=frm_num:
                        acc_time_lb.append(frm+0.5)
                        self.ang_deg_stack=np.vstack((self.ang_deg_stack,self.start_angle)) 
                        self.ang_vel_stack=np.vstack((self.ang_vel_stack,self.current_ang_vel))  
                    else:
                        acc_time_lb.append(frm_num)
                    self.ang_acc_stack=np.vstack((self.ang_acc_stack,self.current_ang_acc))
                    # print('past: ',np.around(self.start_pos[0:3]+np.array([0,0,self.Arm_Model.arm_center.L(1)])))
                    # print('now: ',np.around(self.nodestack[self.Arm_Model.arm_center.DoF,3*frm:3+3*frm]))
                    delta_pos=self.nodestack[self.Arm_Model.arm_center.DoF,3*frm:3+3*frm]-(self.start_pos[0:3]+np.array([0,0,self.Arm_Model.arm_center.L(1)]))
                    # print(np.around(self.nodestack))
                    # print(np.around(self.current_ang_vel,decimals=2))
                    # print(np.around(delta_pos,decimals=2))
                    # print(self.all_lin_vel)
                    # print(np.linalg.norm(delta_pos))
                    self.all_lin_vel=np.hstack((self.all_lin_vel,np.linalg.norm(delta_pos)))
                    self.start_pos=self.Arm_Model.arm_center.ForwardKinematics(self.start_angle)
                    # print(self.all_lin_vel)
                    # print(self.current_ang_vel)
                    # print('------------')
                # print(self.start_angle)
                
            # print(np.around(self.all_lin_vel,decimals=2))
            # print('frm',frm_num)
            # print(np.around(self.nodestack))
            # print('ang_deg:',np.around(self.ang_deg_stack,decimals=1))

                

        ## ----------------draw------------------
        if not fig_inited:
            if shake_cont.ang_fig:
                ani_plt = plt.figure(figsize=(12,10))
            else:
                ani_plt = plt.figure(figsize=(12,6))
            fig_inited=True
        itvl=0.001
        # itvl=0.1/self.fps
        x_end=np.zeros((0,1))
        y_end=np.zeros((0,1))
        z_end=np.zeros((0,1))

        ##--------------------linear velocity-----------------------
        sbplt_vel=223 if shake_cont.ang_fig else 122
        lin_vel_plt=plt.subplot(sbplt_vel, aspect='equal') 
        lin_vel_plt.clear()
        plt.title('Object Center')
        lin_vel_plt.set_ylabel('Velocity (mm/s)')
        lin_vel_plt.set_xlabel('Time (s)')
        lin_vel_plt.grid(True)
        plt.plot(np.array(vel_time_lb)/self.fps*self.my_factor,np.around(self.all_lin_vel,decimals=2)*self.fps,'b*--')
        plt.axis('tight')


        ##--------------------angle graph-----------------------
        if shake_cont.ang_fig:
        # if 1:
            ang_ylb='Position(deg) Velocity(deg/s) Acceleration(deg/s^2)'
            axis_color='b g r c m y'
            ang_factor=[1,self.fps,1]
            ang_stack_lst=[self.ang_deg_stack,self.ang_vel_stack,self.ang_acc_stack]
            for num in range(3):
                ang_plt=plt.subplot(3,2,2+num*2,aspect='equal')
                ang_plt.clear()
                if not num:
                    plt.title('Joints')
                ang_plt.set_ylabel(ang_ylb.split()[num])
                ang_plt.set_xlabel('Time (s)')
                ang_plt.grid(True)
            for num in range(3): 
                for axis in range(6):
                    ang_plt=plt.subplot(3,2,2+num*2,aspect='equal')
                    if num==2:
                        plt.plot(np.array(acc_time_lb)/self.fps*self.my_factor,ang_stack_lst[num][:,axis]*ang_factor[num],axis_color.split()[axis]+'*--',label='Joint'+str(axis+1))
                    else:
                        plt.plot(np.array(ang_time_lb)/self.fps*self.my_factor,ang_stack_lst[num][:,axis]*ang_factor[num],axis_color.split()[axis]+'*--',label='Joint'+str(axis+1))
                    plt.axis('tight')
                plt.legend(loc='upper right')

        ##--------------------3D-----------------------
        for frm in range(frm_num):
            #創建一個三维的繪圖
            sbplt_3d=221 if shake_cont.ang_fig else 121
            threeD_plt=plt.subplot(sbplt_3d,projection='3d', aspect='equal')  
            threeD_plt.clear()
            #arm
            x=np.array([elem(self.nodestack,axis,1+frm*3) for axis in range(1,self.Arm_Model.arm_center.DoF+2)])
            y=np.array([elem(self.nodestack,axis,2+frm*3) for axis in range(1,self.Arm_Model.arm_center.DoF+2)])
            z=np.array([elem(self.nodestack,axis,3+frm*3) for axis in range(1,self.Arm_Model.arm_center.DoF+2)])
            plt.title('Analytic Simulation')
            threeD_plt.set_zlabel('Z')  # 座標軸
            threeD_plt.set_ylabel('Y')
            threeD_plt.set_xlabel('X')
            threeD_plt.grid(True)
            plt.plot(x,y,z,'ro-',label='Manipulator')
            #traj
            if shake_cont.trajetory_disp:
                x_end=np.vstack((x_end,x[self.Arm_Model.arm_center.DoF]))
                y_end=np.vstack((y_end,y[self.Arm_Model.arm_center.DoF]))
                z_end=np.vstack((z_end,z[self.Arm_Model.arm_center.DoF]))
                plt.plot(x_end[:,0],y_end[:,0],z_end[:,0],'b*--',label='Object Center Trajetory')
            
            plt.legend(loc='lower left')
            plt.axis('square')
            plt.axis('equal')

            plt.ion()
            plt.pause(itvl)

hiwin_center_L=[375, 340, 338, 266.5]
hiwin_center_a=[30, 340, -40, 0, 0, 0]
hiwin_center_alpha=[pi/2, 0, -pi/2, pi/2, -pi/2, 0]
hiwin_center_d=[375, 0, 0, 338, 0, 266.5]
hiwin_center_theta=[pi/2, pi/2, pi, 0, 0, 0]
hiwin_axis_limit=np.array([[-90,90],[-125,85],[-55,90],[-90,90],[-115,0],[-360,360]]) # for speed
# hiwin_axis_limit=np.array([[-165,165],[-125,85],[-55,185],[-190,190],[-115,115],[-360,360]]) # full
hiwin_grip_l=Kinematics_Model_cls(8,[375, 340, 338, 266.5],
                    [30, 340, -40, 0, 0, 0, 90, 0],
                    [pi/2, 0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0],
                    [375, 0, 0, 338, 0, 166.5, 0, 180],
                    [pi/2, pi/2, pi, 0, 0, pi/2, 0, 0],
                    [0,0,0],hiwin_axis_limit)
hiwin_grip_r=Kinematics_Model_cls(8,[375, 340, 338, 266.5],
                    [30, 340, -40, 0, 0, 0, 90, 0],
                    [pi/2, 0, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0],
                    [375, 0, 0, 338, 0, 166.5, 0, 180],
                    [pi/2, pi/2, pi, 0, 0, -pi/2, 0, 0],
                    [0,0,0],hiwin_axis_limit)
hiwin_grip_ctrl=[0, 10, 20]
HiwinRA605_Model=Arm_Model(hiwin_center_L,hiwin_center_a,hiwin_center_alpha,hiwin_center_d,
                            hiwin_center_theta,60,hiwin_grip_l,hiwin_grip_r,hiwin_grip_ctrl,hiwin_axis_limit)
HiwinRA605_Simu=Simu_Ani_cls(HiwinRA605_Model)
#  =========================================================================
#                               Main Function
#  =========================================================================
def AngToPos(JointAngle):
    # 初始化&輸入
    plt.figure(figsize=(20,10))
    print('JointAngle(input):',JointAngle)
    # 正運動學
    print('\n-----forward kinematics-----')
    Position=HiwinRA605_Model.arm_center.ForwardKinematics( JointAngle)
    HiwinRA605_Model.DrawRobotManipulator(JointAngle,HiwinRA605_Model.grip_off,'Forward Kinematics',21)
    print('Positon(rounded):',np.around(Position,decimals=1))
    # 逆運動學
    print('\n-----inverse kinematics-----')
    inputPosition=Position
    Angle=HiwinRA605_Model.arm_center.InverseKinematics( inputPosition)
    HiwinRA605_Model.DrawRobotManipulator(Angle,HiwinRA605_Model.grip_on,'Inverse Kinematics',22)
    print('JointAngle(rounded):',np.around(Angle,decimals=1))
    plt.ion()
    plt.pause(0.1)
    return np.hstack((Position,Angle))
def PosToAng(inputPosition):
    # 初始化&輸入
    plt.figure(figsize=(20,10))
    print('Position(input):',inputPosition)
    # 逆運動學
    print('\n-----inverse kinematics-----')
    Angle=HiwinRA605_Model.arm_center.InverseKinematics( inputPosition)
    HiwinRA605_Model.DrawRobotManipulator(Angle,HiwinRA605_Model.grip_on,'Inverse Kinematics',21)
    print('JointAngle(rounded):',np.around(Angle,decimals=1))
    # 正運動學
    print('\n-----forward kinematics-----')
    JointAngle = Angle
    Position=HiwinRA605_Model.arm_center.ForwardKinematics(JointAngle)
    HiwinRA605_Model.DrawRobotManipulator(JointAngle,HiwinRA605_Model.grip_off,'Forward Kinematics',22)
    print('Position(rounded):',np.around(Position,decimals=1))
    plt.ion()
    plt.pause(0.1)
    return np.hstack((Position,Angle))
def MyKine():
    while 1:
        start_cmd_str = input('請輸入指令:\n\t角度->坐標:\t1\n\t坐標->角度:\t2\n\t點到點動畫:\t3\n\t測試極限:\t4\n\t整理資料:\t5\n\t搜尋坐標庫:\t6\n\t離開:\t\tEnter\n')
        if start_cmd_str == '1':
            JointAngle = np.array([0, -30, 30, 40, -45, 60])
            print('\n-----Input-----')
            JointAngle=AngInput()
            AngToPos(JointAngle)
        elif start_cmd_str == '2':
            inputPosition=np.array([-30, 678, -47, -90, 0, -90])
            print('\n-----Input-----')
            inputPosition=PosInput('')
            PosToAng(inputPosition)
        # 繪製動畫 輸入兩點
        elif start_cmd_str =='3':
            begin_pos=np.array([0, 368, 113.5, -90, 0, 0])
            end_pos=np.array([0, 628, 0, -90, 0, 0])
            simu_mode=1
            speed=50
            grip=20
            HiwinRA605_Simu.AdvSimu(begin_pos,end_pos,grip,speed,simu_mode)
            # begin_pos=PosInput('起點')
            # end_pos=PosInput('終點')
            # simu_mode=int(input('模式(0:linear; 1:p2p):'))
            # speed=int(input('速度:'))
            # grip=int(input('夾爪角度:'))
            # HiwinRA605_Simu.MySimuAni(begin_pos,end_pos,grip,speed,simu_mode)
        # 極限測試
        elif start_cmd_str =='4':
            test_type=input('0:\t正運動學\n1:\t逆運動學\n')
            filename=input('檔名:')
            cal_begin_tm=time.time()
            pos_result=np.zeros((0,6))
            ResultNumber=0
            TotalResultNumber=1
            UnitTimes=int(input('輸入精度倒數（e.g.精度0.01->輸入100)：'))
            print('輸入各參數範圍（×精度）')
            DataRange=[int(i) for i in input().split()]
            if test_type == False:
                for i in range(12):
                    DataRange[i]=hiwin_axis_limit[i//2:i%2]
            for j in range(6):
                TotalResultNumber*=(DataRange[j*2+1]-DataRange[j*2])
            print('TotalResultNumber:',TotalResultNumber)
            f=open('/home/brianzx/hiwin_shaking/pos_data/'+filename,'w')
            f.write('TotalResultNumber:'+str(TotalResultNumber))
            f.write('\n')
            f.close()
            # by iteration
            for data1 in range(DataRange[0],DataRange[1]):
                for data2 in range(DataRange[2],DataRange[3]):
                    for data3 in range(DataRange[4],DataRange[5]):
                        for data4 in range(DataRange[6],DataRange[7]):
                            for data5 in range(DataRange[8],DataRange[9]):
                                for data6 in range(DataRange[10],DataRange[11]):
                                    if test_type:
                                        ang_result=HiwinRA605_Model.arm_center.InverseKinematics(\
                                            np.array([data1/UnitTimes,data2/UnitTimes,data3/UnitTimes,data4/UnitTimes,data5/UnitTimes,data6/UnitTimes]))
                                        pos_result=np.around(HiwinRA605_Model.arm_center.ForwardKinematics(ang_result),decimals=2)
                                    else:
                                        pos_result=np.around(HiwinRA605_Model.arm_center.ForwardKinematics(\
                                            np.array([data1/UnitTimes,data2/UnitTimes,data3/UnitTimes,data4/UnitTimes,data5/UnitTimes,data6/UnitTimes])),decimals=2)
                                    ResultNumber+=1
                                    f=open('/home/brianzx/hiwin_shaking/pos_data/'+filename,'a')
                                    f.write('Result No.:'+str(ResultNumber)+'\t\tPosition:'+str(pos_result[0])+', '+str(pos_result[1])+', '+str(pos_result[2])+', '\
                                        +str(pos_result[3])+', '+str(pos_result[4])+', '+str(pos_result[5]))
                                    f.write('\n')
                                    f.close()
                                    cal_end_tm=time.time()
                                    tm_used=cal_end_tm-cal_begin_tm
                                    print('ResultNumber:',ResultNumber,'\tPercentage:{:.2f}%'.format(100*ResultNumber/TotalResultNumber),\
                                        '\tTime used:{:.0f}h{:.0f}m{:.2f}s'.format(tm_used//3600,(tm_used%3600)//60,tm_used%60))
        elif start_cmd_str =='5':
            ln=0
            filename=input('檔名:')
            cal_begin_tm=time.time()
            fw=open('/home/brianzx/hiwin_shaking/pos_data/(sorted)'+filename,'w')
            fw.write('')
            fw.close()
            fr=open('/home/brianzx/hiwin_shaking/pos_data/'+filename,'rt')
            TotalResultNumber=0
            ResultNumber_sorted=0
            all_pos_list=list()
            for line in fr:
                if ln == 0:
                    TotalResultNumber=re.findall(r"\d+",line)
                    print(TotalResultNumber)
                else:
                    pos_list=re.findall(r"[-+]?\d+\.?\d*",line)
                    if pos_list not in all_pos_list:
                        all_pos_list.append(pos_list)
                        fw=open('/home/brianzx/hiwin_shaking/pos_data/(sorted)'+filename,'a')
                        fw.write(str(pos_list[1:]))
                        fw.write('\n')
                        fw.close()
                        ResultNumber_sorted+=1
                    cal_end_tm=time.time()
                    tm_used=cal_end_tm-cal_begin_tm
                    print('ResultNumber:',ln,'\tPercentage:{:.2f}%'.format(100*ln/int(TotalResultNumber[0])),
                        '\tTime used:{:.0f}h{:.0f}m{:.2f}s'.format(tm_used//3600,(tm_used%3600)//60,tm_used%60))
                ln+=1
            fr.close()
            print('TotalResultNumber(sorted):',ResultNumber_sorted)
        elif start_cmd_str =='6':
            ln=0;founded=0
            filename=input('檔名:')
            target_pos=input('輸入目標坐標：')
            target_pos_list=re.findall(r"[-+]?\d+\.?\d*",target_pos)
            print('在(sorted)'+filename+'搜索坐標',target_pos)
            cal_begin_tm=time.time()
            fr=open('/home/brianzx/hiwin_shaking/pos_data/(sorted)'+filename,'rt')
            for line in fr:
                if target_pos_list == re.findall(r"[-+]?\d+\.?\d*",line) :
                    cal_end_tm=time.time()
                    tm_used=cal_end_tm-cal_begin_tm
                    founded=1
                    print('Founded!',target_pos,' At line ',ln,'\tTime used:{:.0f}h{:.0f}m{:.2f}s'.format(tm_used//3600,(tm_used%3600)//60,tm_used%60))
                    break
                ln+=1
            if founded == 0:
                print('Failed for all {} lines!'.format(ln))
        else:
            print('See you next time')
            break
            
if __name__ == "__main__":
    MyKine()
