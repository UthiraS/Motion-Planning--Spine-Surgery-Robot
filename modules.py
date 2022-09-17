#import Packages:

import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *
import plotly.graph_objects as go
# from CollisionFunction import *
from CylinderCollision import *

#Functions
def EuclideanPointDistance(vec1,vec2):
    dist = np.sqrt( np.square(vec1[0]-vec2[0]) +
    np.square(vec1[1]-vec2[1]) + np.square(vec1[2]-vec2[2]))
    return dist



def ComputeRobotKinOffPoints(thetas,val1,val2):
    
    theta1rot = thetas[0]
    Joint1Point = val1
    Joint2Point = np.array([0,-137.8,162.5])
    Joint1UpPoint = np.array([0,0,228.6495])
    Joint_12_Vector = Joint2Point - Joint1Point
    Joint_11_UVector = Vec2UnitVec(Joint1UpPoint - Joint1Point)
    theat12 = (Joint1Point + rodrot(Joint_12_Vector,Joint_11_UVector,theta1rot)) 
    theat34 = (val2 + Joint_12_Vector) 
    return [theat12,theat34]

def ComputeEFPoints(theta4,theta5,theta6,ang1): 
    
    angle = math.pi / 4
    vec56 = theta6 - theta5
    Uvec56 = Vec2UnitVec(vec56)
    
    vec45 = theta5 - theta4
    Uvec45 = Vec2UnitVec(vec45)
    
    crossvec = np.cross(Uvec56,Uvec45)
    EFMidPoint = (theta6 + Uvec56 * 245)
    EFEntryPoint = EFMidPoint +  (rodrot(Uvec45,crossvec,angle) * 32.5264)
    EFTargetPoint = EFEntryPoint + (Vec2UnitVec(EFMidPoint - EFEntryPoint) * 50)

    #theta6 rotation : 
    thet6vecE = (EFEntryPoint - EFMidPoint)
    thet6vecT = (EFTargetPoint - EFMidPoint)

    EFEntryPointF = EFMidPoint + rodrot(thet6vecE,Uvec56,ang1)
    EFTargetPointF = EFMidPoint + rodrot(thet6vecT,Uvec56,ang1)

    return [EFMidPoint,EFEntryPointF,EFTargetPointF]

def ComputeEFPoints_URCT(theta4,theta5,theta6,ang1): 
    
   
    
    
    angle = math.pi / 4
    vec56 = theta6 - theta5
    Uvec56 = Vec2UnitVec(vec56)   

    vec45 = theta5 - theta4
    Uvec45 = Vec2UnitVec(vec45)
    
    # crossvec = np.cross(Uvec56,Uvec45)
    
    # #theta6 rotation : 
    Uvec45 = rodrot(Uvec45,Uvec56,ang1)
    crossvec = np.cross(Uvec56,Uvec45)
    
    theta7 = (Uvec56 * 98.3) + theta6

   
    theta8left = (Uvec45 *(17.5915)) + (crossvec * (16.2497)) + theta7
    theta8right = (Uvec45 *(17.5915)) + (crossvec * (-16.2497))  +theta7

    theta9left = (Uvec56 *49.5624) +theta8left
    theta9right  = (Uvec56 *49.5624) +theta8right

    
    newvec = rodrot(Uvec45,crossvec,-np.pi/6)
    theta10left = ((newvec/Norm(newvec)) * 17.75) + theta9left
    theta10right = ((newvec/Norm(newvec)) * 17.75) + theta9right

    theta8 = (Uvec45 *(17.5915)) + theta7
    theta9 = (Uvec56 *49.5624) + theta8
    theta10 = ((newvec/Norm(newvec)) * 17.75) + theta9
    
    # EFEntryPoint = EFMidPoint +  (rodrot(Uvec45,crossvec,angle) * 32.5264)
    # EFTargetPoint = EFEntryPoint + (Vec2UnitVec(EFMidPoint - EFEntryPoint) * 50)

    # #theta6 rotation : 
    # thet6vecE = (EFEntryPoint - EFMidPoint)
    # thet6vecT = (EFTargetPoint - EFMidPoint)

    # EFEntryPointF = EFMidPoint + rodrot(thet6vecE,Uvec56,ang1)
    # EFTargetPointF = EFMidPoint + rodrot(thet6vecT,Uvec56,ang1)

    # return [EFMidPoint,EFEntryPointF,EFTargetPointF]
    return [theta7,theta8left,theta8right,theta9left,theta9right,theta10left,theta10right,theta8,theta9,theta10]

def JointLocations(thetas):
    
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997 
    d6 = 0.0996 
    
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
    
    theta1 = [0,0,d1]
    
    theta2 = [(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]
               
    theta3 = [np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]
               
    theta4 = [(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]
               
    theta5 = [ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))]
               
    theta6 = [((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))]

    positions = [theta1,theta2,theta3,theta4,theta5,theta6]
    return positions

def JointLocationsEF1(thetas):
   
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
   
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
   
    theta1 = [0,0,d1]
   
    theta2 = [(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]
               
    theta3 = [np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]
               
    theta4 = [(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]
               
    theta5 = [ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))]
               
    theta6 = [((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))]
    
    
    wrist_23 = np.asarray(theta6) - np.asarray(theta5)
    wrist_23_length = VecNorm(wrist_23)
    wrist_23_unit = Vec2UnitVec(wrist_23)

    EndeffectorPoint = (theta6) + (wrist_23_unit * 0.04)
    
    positions = [theta1,theta2,theta3,theta4,theta5,theta6,EndeffectorPoint]
    return positions

def JointLocationsEF(thetas):
   
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
   
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t6 = thetas[5]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
   
    theta1 = np.array ([0,0,d1]) 
   
    theta2 = np.array ([(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]) 
               
    theta3 = np.array ([np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]) 
               
    theta4 = np.array ([(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]) 
               
    theta5 = np.array ([ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))])
               
    theta6 = np.array ([((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))])
    
    #mm conversion :
    
    Stheta1 = theta1 * 1000
    Stheta2 = theta2 * 1000
    Stheta3 = theta3 * 1000
    Stheta4 = theta4 * 1000
    Stheta5 = theta5 * 1000
    Stheta6 = theta6 * 1000
    
    #Other offest points : 
    theta12,theta34 = ComputeRobotKinOffPoints(thetas,Stheta1,Stheta2)

    
    #EndEffector : 
    EFm, EFe,EFt = ComputeEFPoints(Stheta4,Stheta5,Stheta6,t6)
    
    Spositions = [Stheta1,theta12,theta34,Stheta2,Stheta3,Stheta4,Stheta5,Stheta6,EFm,EFe,EFt]

    
    positions = [theta1,theta12/1000,theta34/1000,theta2,theta3,theta4,theta5,theta6,EFm/1000,EFe/1000,EFt/1000]
    
    return positions,Spositions

def JointLocationsEF_URCT(thetas):
   
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
   
    t1 = thetas[0]
    t2 = thetas[1]
    t3 = thetas[2]
    t4 = thetas[3]
    t5 = thetas[4]
    t6 = thetas[5]
    t23 = t2 +  t3
    t234 = t2 + t3 + t4
   
    theta1 = np.array ([0,0,d1]) 
   
    theta2 = np.array ([(a2*np.cos(t1)*np.cos(t2)),
               (a2*np.cos(t2)*np.sin(t1)),
               (d1+(a2*np.sin(t2)))]) 
               
    theta3 = np.array ([np.cos(t1)*((a2*np.cos(t2)) + (a3*np.cos(t23))),
              ((a2*np.cos(t2)) + (a3*np.cos(t23))) *np.sin(t1),
              d1 + (a2*np.sin(t2))+(a3*np.sin(t23))]) 
               
    theta4 = np.array ([(np.cos(t1)*(a2*np.cos(t2)+a3*np.cos(t23)) + d4*np.sin(t1)),
              -d4*np.cos(t1) + ((a2*np.cos(t2)) + (a3*np.cos(t23)))*np.sin(t1),
              d1 + a2*np.sin(t2) + a3*np.sin(t23)]) 
               
    theta5 = np.array ([ d4*np.sin(t1) + (np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              -d4*np.cos(t1) + (np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23))])
               
    theta6 = np.array ([((d4+(d6*np.cos(t5)))*np.sin(t1)) + np.cos(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) -(d6*np.cos(t234)*np.sin(t5))),
              (-np.cos(t1) * (d4+ (d6*np.cos(t5)))) + np.sin(t1) * ((a2*np.cos(t2)) + (a3*np.cos(t23)) + (d5*np.sin(t234)) - (d6*np.cos(t234)*np.sin(t5))),
              d1 - (d5*np.cos(t234)) + (a2*np.sin(t2)) + (a3*np.sin(t23)) - (d6*np.sin(t234)*np.sin(t5))])
    
    #mm conversion :
    
    Stheta1 = theta1 * 1000
    Stheta2 = theta2 * 1000
    Stheta3 = theta3 * 1000
    Stheta4 = theta4 * 1000
    Stheta5 = theta5 * 1000
    Stheta6 = theta6 * 1000
    
    #Other offest points : 
    theta12,theta34 = ComputeRobotKinOffPoints(thetas,Stheta1,Stheta2)

    
    #EndEffector : 
    E7,E8l,E8r,E9l,E9r,E10l,E10r,E8,E9,E10 = ComputeEFPoints_URCT(Stheta4,Stheta5,Stheta6,t6)
    # print( E7,E8l,E8r,E9l,E9r,E10l,E10r)
    
    Spositions = [Stheta1,theta12,theta34,Stheta2,Stheta3,Stheta4,Stheta5,Stheta6,E7,E8l,E8r,E9l,E9r,E10l,E10r,E8,E9,E10]

    
    positions = [theta1,theta12/1000,theta34/1000,theta2,theta3,theta4,theta5,theta6,E7/1000.0,E8l/1000.0,E8r/1000.0,E9l/1000.0,E9r/1000.0,E10l/1000.0,E10r/1000.0,E8/1000.0,E9/1000.0,E10/1000.0]
    
    return positions,Spositions
    

#INVERSE KINEMATICS:

def DHTable2HomTrans(DHTable):
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i in range(np.shape(DHTable)[0]):
        T1 = np.dot(T,DH2HomTrans(DHTable[i]))
        T = T1
    return T

def DH2HomTrans(DHparams):
    [al,a,d,th] = DHparams
    T = [[np.cos(th),-np.sin(th)*np.cos(al),np.sin(th)*np.sin(al),a*np.cos(th)],[np.sin(th),np.cos(th)*np.cos(al),-np.cos(th)*np.sin(al),a*np.sin(th)],[0,np.sin(al),np.cos(al),d],[0,0,0,1]]
    return T

def sol2Rik(x,y,l1,l2):
    a = -2.0*l1*x
    b = -2.0*l1*y
    c = l1**2.0 - l2**2.0 + x**2.0 + y**2.0
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta2 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta12 = np.arctan2((y - l1*np.sin(i))/l2,(x - l1*np.cos(i))/l2)
        theta2[j] = theta12 - i
        j = j+1
    t1t2 = np.column_stack((theta1, theta2))
    return t1t2

# IK Solver:
def solUR5ik(p,R):
    d1 = 0.1625
    a2 = -0.425
    a3 = -0.3922
    d4 = 0.1333
    d5 = 0.0997 #+ 0.025
    d6 = 0.0996 #+ 0.08165
    [x,y,z] = p
    [[nx,ox,ax],[ny,oy,ay],[nz,oz,az]] = R
    a = ay*d6 - y
    b = -ax*d6 + x
    c = -d4
    d = -c/np.sqrt(a**2.0+b**2.0)
    theta1 = [np.arctan2(b,a)+np.arccos(d),np.arctan2(b,a)-np.arccos(d)]
    theta5 = [None]*len(theta1)
    j = 0
    for i in theta1:
        theta5[j] = [np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6),-np.arccos((-d4-y*np.cos(i)+x*np.sin(i))/d6)]
        j = j+1
    
    
    t1t5 = [[theta1[0],theta5[0][0]],[theta1[0],theta5[0][1]],[theta1[1],theta5[1][0]],[theta1[1],theta5[1][1]]]
    theta6 = [None]*np.shape(t1t5)[0]
    j = 0
    for i in range(np.shape(t1t5)[0]):
        theta6[j] = [np.arctan2((oy*np.cos(t1t5[i][0])-ox*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]),(-ny*np.cos(t1t5[i][0])+nx*np.sin(t1t5[i][0]))/np.sin(t1t5[i][1]))]
        j = j+1     
    t1t5t6 = np.hstack((t1t5,theta6))
    k = 0
    t2t3t4 = [None]*np.shape(t1t5)[0]*2  
    TUR5 = np.vstack((np.hstack((R,np.array(p).reshape(3,1))),[0,0,0,1]))
    for i in range(np.shape(t1t5t6)[0]):
        T01 = DHTable2HomTrans([[np.pi/2,0,d1,t1t5t6[i][0]]])
        T56 = DHTable2HomTrans([[-np.pi/2,0,d5,t1t5t6[i][1]],[0,0,d6,t1t5t6[i][2]]])
        T3R = np.dot(np.dot(np.linalg.inv(T01),TUR5),np.linalg.inv(T56))
        
        theta234 = np.arctan2(T3R[1][0],T3R[0][0])
        theta23 = sol2Rik(T3R[0][3],T3R[1][3],a2,a3)
        for j in range(np.shape(theta23)[0]):
            theta4 = theta234 - theta23[j][0] - theta23[j][1]
            t2t3t4[k]=[theta23[j][0],theta23[j][1],theta4]
            k = k+1
    t1 = np.array([val for val in t1t5 for _ in (0, 1)])[:,0]
    t5 = np.array([val for val in t1t5 for _ in (0, 1)])[:,1]
    t6 = np.array([val for val in theta6 for _ in (0, 1)])
    ikUR5 = np.hstack((t1.reshape(8,1),t2t3t4,t5.reshape(8,1),t6.reshape(8,1)))
    return ikUR5


def conversion(joint_angle):
    dumy = [0,-2*math.pi,2*math.pi,0,0,0]
    ss = np.add(joint_angle,dumy)
    return ss

def OrientMatix(entry,target,theta):
     #vector y:
    v_y = np.asarray(target) - np.asarray(entry)
    denom = np.sqrt( np.square(v_y[0]) + np.square(v_y[1]) + np.square(v_y[2]))
    vector_y = -( v_y / denom)       
    #vector z:
    element1 = -( ((vector_y[0]*entry[0]) + (vector_y[1]*entry[1]))  / vector_y[2])
    v_z = np.asarray( [entry[0],entry[1],element1] )
    denom1 = np.sqrt( np.square(v_z[0]) + np.square(v_z[1]) + np.square(v_z[2]))
    vector_z = v_z / denom1   
    #Rotation shift:
    #Rodrigues form
    comp1 = vector_z * np.cos(theta)
    comp2 = np.cross(vector_y,vector_z) * np.sin(theta)
    comp3 = (vector_y * (np.dot(vector_y,vector_z)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    vector_z = Rodgriues
    #vector x
    vector_x = np.asarray ( np.cross(vector_y,vector_z))
    return vector_x,vector_y,vector_z

def PositionRetrival(pos_,orien):
    
    # print(pos_,orien)
    #Tranformation : T 0-7
    PP1 = [float(Decimal(pos_[0]) / Decimal(1000)),float(Decimal(pos_[1]) / Decimal(1000)),float(Decimal(pos_[2]) / Decimal(1000))] 
    pers_scale = np.array([0,0,0,1])
    pos = np.array(PP1)
    ore = np.array(orien)
    H_half = np.hstack((ore,pos.reshape((3,1))))
    Input_H  = np.vstack((H_half,pers_scale))

    #Adding offset in Rotation and Translation : T 6-7

    # offset_xyz = np.array([0.00039,-0.02597,0.21758]) #Translation shift //sep 9/9/2021 demo
    offset_xyz = np.array([0.00116,-0.03261,0.14678])
#     offset_xyz = np.array([0,0,0])
    # offset_xyz = np.array([-0.00051,-0.02669,0.21813])
    Rotation = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
    
    a = np.hstack((Rotation,offset_xyz.reshape(3,1)))
    Trans_6to7 = np.vstack((a,pers_scale))
    TT = np.linalg.inv(Trans_6to7)
    
    #Calibration offset
    offset_cali = np.array([0,0,0])
    Orientation_cali_offest = OrientationEndEffectorCorrection()
    Cali_mat = np.hstack((Orientation_cali_offest,offset_cali.reshape(3,1)))
    Trans_7to8 = np.vstack((Cali_mat,pers_scale))
    TT78 = np.linalg.inv(Trans_7to8)
    
    
    #2 inverse
    TransMat = np.dot(TT78,TT)
    

    #Transformation :  6-7
    T0_6 = np.dot(Input_H,TransMat)
    OREINTATION = T0_6[0:3,0:3]
    Pos = T0_6[0:3,3]
    
   

    return Pos,OREINTATION


def AnglebtwnVecs(vec1,vec2):               
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return angle

def rodrot(vec,axis,theta):
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def VecNorm(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    return norm

def TimeStepAngles(vec1,vec2,theta,Length):
    Stack = []
    vec3 = np.cross(vec1,vec2)
    vec3 = Vec2UnitVec(vec3)
    Inc = float(Decimal(theta)/Decimal(Length/2))
    AngleChange = list(np.arange(0,theta,Inc))
    RepeatAngle = list(np.repeat(theta,(Length - len(AngleChange))))
    Angles = AngleChange + RepeatAngle
    
    for i in range(Length):
        Vec = rodrot(vec1,vec3,Angles[i])
        Stack.append(Vec)
    return Stack

def RetriveTimeStepOrientation(IO,EO,Length):
    TSOrient = []
    x,y,z = IO[:,0],IO[:,1],IO[:,2]
    xe,ye,ze = EO[:,0],EO[:,1],EO[:,2]
    theta1 = AnglebtwnVecs(x,xe)
    theta2 = AnglebtwnVecs(y,ye)
    theta3 = AnglebtwnVecs(z,ze)
    
    C_x = TimeStepAngles(x,xe,theta1,Length)
    C_y = TimeStepAngles(y,ye,theta2,Length)
    C_z = TimeStepAngles(z,ze,theta3,Length)
    
    for i in range(len(C_x)):
        TSOrient.append( np.transpose(np.array(([C_x[i],C_y[i],C_z[i]]))))
        
    return TSOrient


def PathContinuityVerification2(JJ):

    JV = []
    Prev_value = [0,-math.pi/2,math.pi/2,0,math.pi/2,math.pi]
    for i in range(len(JJ)):

        current_value = JJ[i]
        joint1 = Prev_value[1] - current_value[1]
        joint6 = Prev_value[5] - current_value[5]
        
        if (np.abs(joint1) > 1.5 ):
            current_value = conversion(current_value)
        if (np.abs(joint6) > 1.5):
            current_value[5] = (2*math.pi)  + current_value[5]
            
#         print(current_value)
        Prev_value = current_value
        JV.append(current_value)

    return JV


def ChangePathThetas(act,des):
    positive_quad = 2 * math.pi
    negative_quad = 2 * -math.pi

    check1 = negative_quad + des
    check2 = check1 - act
    if np.abs(check2) > 1.5:
        check1 = positive_quad + des
    
    return check1

def PathContinuityVerification(JJ):

    #Initializers :  
    JV = []
    Initial_thetas = [0,-math.pi/2,math.pi/2,math.pi,-math.pi/2,0]


    for i in range(len(JJ)):
        current_value = JJ[i]

        if current_value[1] > 2.5:
            current_value = conversion(current_value)

        for iter1 in range(len(Initial_thetas)):
            value = Initial_thetas[iter1] - current_value[iter1]
            if np.abs(value) > 1.5:
                rev_value = ChangePathThetas(Initial_thetas[iter1],current_value[iter1]) 
                current_value[iter1] = rev_value

        Initial_thetas = current_value

        JV.append(current_value) 
    print(JV)
    return JV




# collision check and singularity : 


#Retrive initial and final orientation: 

# def RetriveInitialFinalOrientation(Initial_point,Entry,Target,theta):
    
#     # Data point convertion and Initial / Final Orientation:

#     Initial_Orientation = np.array([[ 0.21581449, -0., -0.97643438],
#                                   [-0.97643438, -0. , -0.21581449],
#                                   [ 0. ,  1. , -0]])
#     # Initial_Orientation = np.array([[ 0.82375599,  0.01000137, -0.56685628],
#     #    [-0.5669134 ,  0.02500344, -0.82339785],
#     #    [ 0.00593824,  0.99963733,  0.02626664]])

#     EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
#     TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 

#     x,y,z = OrientMatix(EM,TM,theta)
#     Orientation = [x,y,z]
#     Final_Orientation = np.transpose(Orientation)
    
#     print("Final Orientation : {}".format(Final_Orientation))
#     #endeffector correction
#     # Final_Orientation = OrientationEndEffectorCorrection(Final_Orientation)
#     print("Final Orientation Modified : {}".format(Final_Orientation))

    
#     Orient = RetriveTimeStepOrientation(Initial_Orientation,Final_Orientation,17)

#     return Orient

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints


def RetriveIKSolutions(Path,Orientation,solution):
    
    x_move = []
    y_move = []
    z_move = []
    Joint_Values = []
    stack_Fk = []
    print("IK system")
    OR =np.array([[1,0,0],[0,1,0],[0,0,1]])
    for i in range(len(Path[0])):

        PathPoint = [Path[0][i],Path[1][i],Path[2][i]]
        POSI,ORIEE = PositionRetrival(PathPoint,Orientation[i])
        
        
    
        # if i == 0:
        #     ORIEE = np.array([[-0.11160617,  0.7150721 , -0.69008402],
        #                     [ 0.99277022,  0.04936051, -0.10941126],
        #                     [-0.04417404, -0.69730584, -0.71541122]])
        
        ang = solUR5ik(POSI,ORIEE)
        if(i == 19):
            print("orientation")
            # print(Orientation[i])
            OR =Orientation[i]
        check_solutions = len(np.unique(np.isnan(ang[solution])))
       
        if check_solutions == 2 or np.unique(np.isnan(ang[solution])) == True:
            msg = "Out of workspace"
            # return [0,0,0,0]
        print(ang[solution])
        Joint_Values.append(ang[solution])
   
        #FK part
        Fk_values,Fk_values_scaled = JointLocationsEF_URCT(ang[solution])
        
      
        x_,y_,z_ = SepPoints(np.asarray(Fk_values))

        x_move.append(x_)
        y_move.append(y_)
        z_move.append(z_)
        stack_Fk.append(Fk_values_scaled)

    Modified_JV = PathContinuityVerification(Joint_Values) 
    return x_move,y_move,z_move,Modified_JV,stack_Fk,OR



#Logic File:
def ModuleCollision(Pathcoordinates_iter,TimeStepOrientation,EntryList,TargetList,Initial_point,Entry,Target,Needle_radius,ETCList,EntryTargetStringList,statustype):
    print("Status : {}".format(1))
    print("path")
    print(Pathcoordinates_iter)
    soln = [2]
    singularity_status = 1
    for ii in range(len(soln)): 
        solution = soln[ii]
                     
        X_Cods,Y_Cods,Z_Cods,J_values,S_Fk,OR = RetriveIKSolutions(Pathcoordinates_iter,TimeStepOrientation,solution)
        print(len(X_Cods),len(Y_Cods),len(Z_Cods),len(J_values))
        print(J_values[17])
        print(J_values[18])
        print(J_values[19])
        # If coordinates are Nan:
        if X_Cods != 0 and Y_Cods != 0 and Z_Cods != 0: 
            Status,CollisionFrameData = CheckNeedleSelfCollision(EntryList,TargetList,X_Cods,Y_Cods,Z_Cods,Needle_radius,S_Fk,ETCList,EntryTargetStringList,statustype)
            
            if Status == 1:
                singularity_status = Singularity(J_values,5)
                X_Cods,Y_Cods,Z_Cods = EndEffectorFrame(X_Cods,Y_Cods,Z_Cods)
                print("Trajectory has been found! Ready to execute")
                # X_Cods = X_Cods[0:17]
                # Y_Cods = Y_Cods[0:17]
                # Z_Cods = Z_Cods[0:17]
                # J_values = J_values[0:17]
                return X_Cods,Y_Cods,Z_Cods,J_values,Status,CollisionFrameData,OR

            print("Needle collision")        
    # X_Cods = X_Cods[0:17]
    # Y_Cods = Y_Cods[0:17]
    # Z_Cods = Z_Cods[0:17]
    # J_values = J_values[0:17]
    print("Cannot plan a trajectory !")
    return X_Cods,Y_Cods,Z_Cods,J_values,Status,CollisionFrameData,OR


#Singularity check :
def Singularity(jv,step):
    stack = []
    jv_timestep = []
    dummy = []
    for i in range(len(jv)-1):
        mid = np.linspace(jv[i],jv[i+1],step)
        dummy.append(mid)
        for j in range(len(dummy[0])):
            jv_timestep.append(list(dummy[i][j]))
    for i in range(len(jv_timestep)):        
        Positions = JointLocations(jv_timestep[i])
        wristvec1 = np.asarray(Positions[3]) - np.asarray(Positions[2])
        wristvec2 = np.asarray(Positions[5]) - np.asarray(Positions[4])
        Resultatnt = np.cross(Vec2UnitVec( wristvec1), Vec2UnitVec( wristvec2))
        Res = VecNorm(Resultatnt)#wrist
        elbowvec1 = np.asarray(Positions[1]) - np.asarray(Positions[0])
        elbowvec2 = np.asarray(Positions[2]) - np.asarray(Positions[1])
        Resultatnt1 = np.cross(Vec2UnitVec( elbowvec1), Vec2UnitVec( elbowvec2))
        Res1 = VecNorm(Resultatnt1)#elbow
        
        p1 = Positions[0]
        p2 = [0,0.1333,0.1625]
        vector1 = np.subtract(p2,p1)
        p = p1 + 0.5*(vector1)
        p = [p[0],p[1],p[2]+10]
        vec2 = np.subtract(p2,p)

        a,b,c = np.cross(vec2,vector1)
        d = a*p[0] + b*p[1] + c*p[2]
        d1 = a* Positions[2][0] + b*Positions[2][1] + c*Positions[2][2]
        d2 = a* Positions[3][0] + b*Positions[3][1] + c*Positions[3][2]
        if (d1 <= d+0.2 and d1 >= d-0.2) and (d2 <= d+0.2 and d2 >= d-0.2): 
            print("Shoulder")
            retval = 1
            stack.append(retval)
            return 1
        elif Res>=0 and Res<=0.2:
            print("Wrist")
            retval = 1
            stack.append(retval)
            return 1
        elif Res1>=0 and Res1<=0.2:
            print("Elbow")
            retval = 1
            stack.append(retval)
            return 1
        else:
            retval = 0
            stack.append(retval)
    if np.any(stack) == 1:
        return 1
    else: 
        return 0
    
#Recorrection : 
def robo_correction(robo,actual):
    count = 1
    cpv = robo.getl()
    zeros = np.array([0,0,0])
    error = np.asarray(actual) - np.asarray(cpv[0:3])
    correction = np.asarray(error)
    ccv = np.concatenate([correction,zeros])
    print("Algorithm Error : {} ".format(error*1000))
    robo.movel((ccv),relative=True,wait=False)
    time.sleep(0.5)
    while count == 1:
        if robo.is_program_running() == False:
            count = 0
            break
    cpv1 = robo.getl()
    error1 = np.asarray(actual) - np.asarray(cpv1[0:3])
    print("Final System  Error : {}".format(error1*1000))
    return 1


def EndEffectorFrame(Xcods,Ycods,Zcods):
    for i in range(0,len(Xcods)):


        p1 = [Xcods[i][-2],Ycods[i][-2],Zcods[i][-2]]
        p2 = [Xcods[i][-1],Ycods[i][-1],Zcods[i][-1]]
    
        P = p2 + 1.12*np.subtract(p2,p1)
        x1 = P[0]
        y1 = P[1]
        z1 = P[2]
        Xcods[i].append(x1)
        Ycods[i].append(y1)
        Zcods[i].append(z1)

    return Xcods,Ycods,Zcods


# def CalldevicePullback(robo):

#     counter = 0
#     current_joint_values = robo.getj()
#     current_joint_values[1] = current_joint_values[1] - 0.2
#     robo.movej(current_joint_values,0.1,0.1,wait=False)
#     time.sleep(1)
#     while counter == 0:
#         if robo.is_program_running() == False:
#             counter = 1
#             break

#     counter_= 0
#     robo.movej((0,-math.pi/2,math.pi/2,0,math.pi/2,math.pi),0.2,0.2,wait=False)
#     time.sleep(1)
#     while counter_ == 0:
#         if robo.is_program_running() == False:
#             counter_ = 1
#             break

#     return 1


#Orientation correction
def deg2rad(val):
    resu = (val*math.pi)/180
    return resu

def OrientationEndEffectorCorrection():

    
    Home_orientation = np.array([[ 0.00129914, -0.00056073, -0.999999  ],
                                   [ 0.99999866, -0.00099154,  0.0012997 ],
                                   [-0.00099227, -0.99999935,  0.00055944]])

    # ChangeIn_orientation = np.array([[ 0.00341563,  0.70060573, -0.71354043], // ddmp 9/9/2021 
    #                              [ 0.99999412, -0.00216425,  0.00266183],
    #                              [ 0.00032061, -0.71354533, -0.700609  ]])

    ChangeIn_orientation = np.array([[-0.0063458 ,  0.86643587, -0.49924805],
       [ 0.9998427 ,  0.01376649,  0.01118277],
       [ 0.01656205, -0.49909856, -0.86638694]])

    Home_inv = np.linalg.inv(Home_orientation)
    offset = np.dot(Home_inv,ChangeIn_orientation)
#     final_orient = np.dot(O_matrix,offset)

    return offset

# #Pose optimization :

def PoseOptimization(inp_thetas):
    sl = JointLocationsEF1(inp_thetas)
    wrist1 = np.asarray(sl[3]) - np.asarray(sl[2])
    wrist3 = np.asarray(sl[5]) - np.asarray(sl[4])

    value = AnglebtwnVecs(wrist1,wrist3)
    wrist1_3ang = (value * 180)/math.pi
    

    #Extra piece
    factor = 0.05
    if sl[6][2] >=0:
        checker = sl[6][2] + factor
    else:
        checker = sl[6][2] - factor
    
    #conditions
    if sl[3][2] >= checker and wrist1_3ang <= 110 and sl[4][2] >= checker:
        stat = 0
    else :
        stat = 1
        
    #camera condition for right docking
    
    # wrist34 = np.asarray(sl[4]) - np.asarray(sl[3])
    
    # cameracord = np.array([-0.900,0.200,0.750]) #Right
#     cameracord = np.array([-0.750,-0.550,0.750]) #Left
    
    # condition_camera = AnglebtwnVecs(wrist34,cameracord)

    # print("CameraCondition : {}".format((condition_camera * 180)/ math.pi))
    
    return stat 

def ModulePoseOptimization(Entry,Target):

    EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 

    RotationFactor = 0
    iter_ = 0
    factor = 0

    while (iter_ != 1):
        
        factor = factor + 0.2

        for iterval_2 in range(3):
            
            print("RF : {}".format(RotationFactor))
                
            x,y,z = OrientMatix(EM,TM,RotationFactor)
            
            Orientation = [x,y,z]
            Final_Orientation = np.transpose(Orientation)
            POSI,ORIEE = PositionRetrival(Entry,Final_Orientation)
            
            JointAngles = solUR5ik(POSI,ORIEE)
            solution = JointAngles[2]
        
            if np.abs(solution[1]) > 3:
                JointAnglesCorrected = PathContinuityVerification2(JointAngles)
                if JointAnglesCorrected[2][5] > 2 * math.pi :
                    JointAnglesCorrected[2][5] = JointAnglesCorrected[2][5] - (2*math.pi)           
                    final_solution = JointAnglesCorrected[2]
                else :
                    final_solution = JointAnglesCorrected[2]
            else:
                final_solution = solution


            status = PoseOptimization(final_solution)

            if status == 0:
                print("Found solution")
                print("RotationFactor : {}".format(RotationFactor))
                print(final_solution)
                iter_ = 1
                return Final_Orientation

            else :
                if iterval_2 == 0: 
                    RotationFactor = RotationFactor + 0.1
                elif iterval_2 == 1:
                    RotationFactor = - RotationFactor 
            
            if RotationFactor > 2*math.pi:
                print("No solution found")
                iter_ = 1
                return [["False"]]
        
        
        RotationFactor = RotationFactor + factor
        

#
def MultiNeedleSingularityCheck(entry_1,target_1,scale,robo,Origin):
    
    #get current values:
    cpv = robo.getl()
    orien = robo.get_orientation()
    orien = orien.array

    entry = np.asarray(cpv[0:3]) * 1000
    target = entry + (orien[:,1] * scale)
    
    EM = [float(Decimal(entry[0]) / Decimal(1000)),float(Decimal(entry[1]) / Decimal(1000)),float(Decimal(entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(target[0]) / Decimal(1000)),float(Decimal(target[1]) / Decimal(1000)),float(Decimal(target[2]) / Decimal(1000))] 
    
    #Adding Origin offset
    EM =  np.asarray(EM)
    TM = np.asarray(TM) 
    print("Modified entry : {}".format(EM))

    UEntry = np.array(EM) * 1000
    UTarget = np.array(TM) * 1000

    #cpv

    Joint_Values = []

    if scale > 0 :
        vec = UEntry - UTarget
    else : 
        vec = UTarget - UEntry

    Unit_vec = Vec2UnitVec(vec)
    rr = np.round(np.abs(scale)/5)
    values = np.arange(1,np.abs(scale),rr)
    for lst in values:
        
        dir_vec = EM + ((Unit_vec * lst) /1000)
        print("Directional : {}".format(dir_vec))
        #Check singularity
        ang = solUR5ik(dir_vec,orien)

        check_solutions = len(np.unique(np.isnan(ang[0])))
        
        if check_solutions == 2 or np.unique(np.isnan(ang[0])) == True:
            print("Singularity status for multi needle : Found Nan values")
            return 0

        Joint_Values.append(ang[0])
    Modified_JV = PathContinuityVerification(Joint_Values)
    singularity_status = Singularity(Modified_JV,2)
    
    print("Singularity status for multi needle : {}".format(singularity_status))
    
    if singularity_status == 0 :
        dir_vec = EM + ((Unit_vec * scale) /1000)
        Orientation = cpv[3:6]
        value = np.concatenate((dir_vec,Orientation),0)
        robo.movel((value),0.05,0.05,wait=False)
        return 1
    else : 
        return 0
        



    