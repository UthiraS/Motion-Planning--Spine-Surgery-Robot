#import Packages:
import urx
import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *
import plotly.graph_objects as go
import sys




#Defined Functions : 

# Visualization


def Visualizing_Cylinder(Cylinder_Point1,Cylinder_Point2,r,colorscale):
    # Plotting a cylinder given its two end points and radius 
    Cylinder_Point1 =np.array(Cylinder_Point1)
    Cylinder_Point2 = np.array(Cylinder_Point2)
    
    #vector in direction of axis
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1

    #find magnitude of vector
    mag = np.linalg.norm(Cylinderaxis)

    #unit vector in direction of axis
    Cylinderaxis = Cylinderaxis / mag

    #make some vector not in the same direction as Cylinderaxis
    not_Cylinderaxis = np.array([1, 0, 0])
    if (Cylinderaxis == not_Cylinderaxis).all():
        not_Cylinderaxis = np.array([0, 1, 0])

    #make vector perpendicular to Cylinderaxis
    n1 = np.cross(Cylinderaxis, not_Cylinderaxis)
    #normalize n1
    n1 /= np.linalg.norm(n1)
    #make unit vector perpendicular to Cylinderaxis and n1
    n2 = np.cross(Cylinderaxis, n1)

    #create sample space for thetha and radius
    t = np.linspace(0, mag, 2)
    theta = np.linspace(0, 2 * np.pi, 100)
    rsample = np.linspace(0, r, 2)

    #use meshgrid to make 2d arrays
    t, theta2 = np.meshgrid(t, theta)
    rsample,theta = np.meshgrid(rsample, theta)

    #generate coordinates for surface
    # "Tube"
    X, Y, Z = [Cylinder_Point1[i] + Cylinderaxis[i] * t + r * np.sin(theta2) * n1[i] + r* np.cos(theta2) *n2[i] for i in [0, 1, 2]]
    # "Bottom"
    X2, Y2, Z2 = [Cylinder_Point1[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    # "Top"
    X3, Y3, Z3 = [Cylinder_Point1[i] + Cylinderaxis[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        

    #create plotly surface for tube, bottom and top surface of cylinder
    #colorscale='Viridis'
    cyl_tube = go.Surface(x=X, y=Y, z=Z,
                          colorscale=colorscale,
                          showscale=False,opacity=0.8)
    cyl_bottom = go.Surface(x=X2, y=Y2, z=Z2,
                          colorscale=colorscale,
                          showscale=False,opacity=0.8)
    cyl_top = go.Surface(x=X3, y=Y3, z=Z3,
                          colorscale=colorscale,
                          showscale=False,opacity=0.8)
    return(cyl_tube,cyl_bottom,cyl_top)

def plane(fig,p1,p2,p3,p4):
    x = [p1[0],p2[0],p3[0],p4[0]]
    y = [p1[1],p2[1],p3[1],p4[1]]
    z = [p1[2],p2[2],p3[2],p4[2]]

    PlaneData = {
    'type': 'mesh3d',        
    'x': x,
    'y': y,
    'z': z, 
    #'delaunayaxis':'x',
    'color': 'green',
    'opacity': 0.5,
    }
    fig.add_trace(PlaneData)

    return fig

#Robot base cylinder    
def RobotBaseCylinder(r, h, a =0, nt=100, nv =50):
    theta = np.linspace(0, 2*np.pi, nt)
    v = np.linspace(a, a+h, nv )
    theta, v = np.meshgrid(theta, v)
    x = 0+r*np.cos(theta)
    y= 0+r*np.sin(theta)
    z = 0+v
    return x, y, z


def RobotBaseStructure(thetas):
   
    temp_joint_locations = thetas
    sl,_ = JointLocationsEF(temp_joint_locations)
    
    x_p,y_p,z_p = SepPoints(sl)

    x_p=list(np.asarray(x_p) * 1000)
    y_p=list(np.asarray(y_p) * 1000)
    z_p=list(np.asarray(z_p) * 1000)
    
    
    data=go.Scatter3d(x=x_p,
                        y=y_p,
                        z=z_p,marker=dict(
                size=[58,58,48,48,37.5,37.5,37.5,37.5,23.5],
                opacity = 0,
                color = ('rgb(22, 96, 167)'),              
                colorscale='Viridis',  

            ),
                line = dict(
                colorscale="Viridis",
                width = 50,
                ),
            )
    fig = go.Figure(data = data)
    
    Rangevalue = 1200
    fig.update_layout(scene=dict(zaxis=dict(range=[-Rangevalue,Rangevalue],autorange=False),      
                        yaxis=dict(range=[-Rangevalue,Rangevalue],autorange=False),
                    xaxis=dict(range=[-Rangevalue,Rangevalue],autorange=False)))              
    fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

    fig.add_trace(data)
    Data = go.Mesh3d(
       
        x=[400, 400, 0, 0, 400, 400, 0, 0],
        y=[400, -450, -450, 400, 400, -450, -450, 400],
        z=[0, 0, 0, 0, -700, -700, -700, -700],
        colorbar_title='z',
       
        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        name='y',
        showscale=True
    )
    fig.add_trace(Data)
   
    #Base cylinder
    r1 = 45
    a1 = 0
    h1 = 150
    x1, y1, z1 = RobotBaseCylinder(r1, h1, a=a1)
    colorscale = [[0, 'red'],
                    [1, 'red']]
    cyl1 = go.Surface(x=x1, y=y1, z=z1,
                    colorscale=colorscale,
                    showscale=False,)
    #############auto -925 925

    fig.add_trace(cyl1)

    #Plane data:

    # height = 0
    # Pp1 = [-800,-750,height]
    # Pp2 = [-250,-750,height]
    # Pp3 = [-800,750,height]
    # Pp4 = [-250,750,height]

    # fig = plane(fig,Pp1,Pp2,Pp3,Pp4)

    return fig


def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")

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

def Norm(vec):   
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))    
    return norm

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

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
    
    Spositions = [Stheta1,theta12,theta34,Stheta2,Stheta3,Stheta4,Stheta5,Stheta6,E7,E8l,E8r,E9l,E9r,E10l,E10r,E8,E9,E10]

    
    positions = [theta1,theta12/1000,theta34/1000,theta2,theta3,theta4,theta5,theta6,E7/1000.0,E8l/1000.0,E8r/1000.0,E9l/1000.0,E9r/1000.0,E10l/1000.0,E10r/1000.0,E8/1000.0,E9/1000.0,E10/1000.0]
    
    return positions,Spositions

def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

def rodrot(vec,axis,theta):
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

    
def GenerateCylinders(values):

    CylinderTop =[]
    CylinderBottom =[]
    Radii =[]

    #Cylinder 1

    Cylinder1Point1 =np.array([0,0,0])
    Cylinder1Point2 =values[0]+np.array([0,0,66.1495])
    Cylinder1Radius =116.00/2

    CylinderBottom.append(Cylinder1Point1)
    CylinderTop.append(Cylinder1Point2)
    Radii.append(Cylinder1Radius)

    #Cylinder 2

    r12vec = values[1] - values[0]
    r12vec = Vec2UnitVec(r12vec) 
    offset12 = 14.2
    r12vec1 = r12vec *(Cylinder1Radius + offset12/2) + values[0]
    r12vec2 = r12vec *(Cylinder1Radius + offset12 + 128.534) + values[0]
    Cylinder2Point1 =np.array(r12vec1)
    Cylinder2Point2 =np.array(r12vec2)
    Cylinder2Radius =116.00/2

    CylinderBottom.append(Cylinder2Point1)
    CylinderTop.append(Cylinder2Point2)
    Radii.append(Cylinder2Radius)

    #Cylinder 3

    r24vec = values[2] - values[1]
    r24vec = Vec2UnitVec(r24vec) 
    offset23 = 13.4
    r23vec1 = r24vec *(Cylinder2Radius+offset23/4) + values[1]
    r23vec2 = r24vec *(Cylinder2Radius+offset23 + 277.8+offset23+10) + values[1]
    Cylinder3Point1 =np.array(r23vec1)
    Cylinder3Point2 =np.array(r23vec2)
    Cylinder3Radius =85.00/2

    CylinderBottom.append(Cylinder3Point1)
    CylinderTop.append(Cylinder3Point2)
    Radii.append(Cylinder3Radius)

    #Cylinder 4
    
    r4J3vec = values[3] - values[2]
    r4J3vec = Vec2UnitVec(r4J3vec) 
    r4J3vec1 = r4J3vec *(-48) + values[2]
    r4J3vec2 = r4J3vec *(131.0756+65.1344) + values[2]
    Cylinder4Point1 =np.array(r4J3vec1)
    Cylinder4Point2 =np.array(r4J3vec2)
    Cylinder4Radius =116.00/2

    CylinderBottom.append(Cylinder4Point1)
    CylinderTop.append(Cylinder4Point2)
    Radii.append(Cylinder4Radius)

    #Cylinder 5

    r6J3vec = values[4] - values[3]
    r6J3vec = Vec2UnitVec(r6J3vec) 
    r6J3vec1 = r6J3vec * (65) + values[3]
    r6J3vec2 = r6J3vec * (77+263.20+5.60+5) + values[3]
    Cylinder5Point1 =np.array(r6J3vec1)
    Cylinder5Point2 =np.array(r6J3vec2)
    Cylinder5Radius =75.00/2

    CylinderBottom.append(Cylinder5Point1)
    CylinderTop.append(Cylinder5Point2)
    Radii.append(Cylinder5Radius)

    #Cylinder 6

    r67vec = values[5] - values[4]
    r67vec = Vec2UnitVec(r67vec) 
    r67vec1 = r67vec * (-56.05) + values[4]
    r67vec2 = r67vec * (88.20) + values[4]
    Cylinder6Point1 =np.array(r67vec1)
    Cylinder6Point2 =np.array(r67vec2)
    Cylinder6Radius =75.00/2

    CylinderBottom.append(Cylinder6Point1)
    CylinderTop.append(Cylinder6Point2)
    Radii.append(Cylinder6Radius)

    #Cylinder 7

    r78vec = values[6] - values[5]
    r78vec = Vec2UnitVec(r78vec)
    r78vec1 = r78vec *(-56.03) + values[5]
    r78vec2 = r78vec *(58) + values[5]
    Cylinder7Point1 =np.array(r78vec1)
    Cylinder7Point2 =np.array(r78vec2)
    Cylinder7Radius =75.00/2

    CylinderBottom.append(Cylinder7Point1)
    CylinderTop.append(Cylinder7Point2)
    Radii.append(Cylinder7Radius)

    #Cylinder 8

    r89vec= values[7] - values[6]
    r89vec= Vec2UnitVec(r89vec) 
    r89vec1 = r89vec * (-56.03) + values[6]
    r89vec2 = r89vec * (99.6+47)  + values[6]
    Cylinder8Point1 =np.array(r89vec1)
    Cylinder8Point2 =np.array(r89vec2)
    Cylinder8Radius =75.00/2

    CylinderBottom.append(Cylinder8Point1)
    CylinderTop.append(Cylinder8Point2)
    Radii.append(Cylinder8Radius)


    # #Cylinder9

    
    # tcpendeffvec= values[8] - values[7]
    # tcpendeffvec= Vec2UnitVec(tcpendeffvec) 
    # tcpendeffvec1 = tcpendeffvec * (4)  + values[7]
    # tcpendeffvec2 = tcpendeffvec * (47)  + values[7]
    # Cylinder9Point1 =np.array(tcpendeffvec1 )
    # Cylinder9Point2 =np.array(tcpendeffvec2)
    # Cylinder9Radius =75.00/2

    # CylinderBottom.append(Cylinder9Point1)
    # CylinderTop.append(Cylinder9Point2)
    # Radii.append(Cylinder9Radius)


    #Cylinder91

    
    tcpendeffvec= values[8] - values[7]
    tcpendeffvec= Vec2UnitVec(tcpendeffvec) 
    tcpendeff1vec1 = tcpendeffvec * (52)  + values[7]
    tcpendeff1vec2 = tcpendeffvec * (225) + values[7]
    Cylinder91Point1 =np.array(tcpendeff1vec1)
    Cylinder91Point2 =np.array(tcpendeff1vec2)
    Cylinder91Radius =25.00/2

    CylinderBottom.append(Cylinder91Point1)
    CylinderTop.append(Cylinder91Point2)
    Radii.append(Cylinder91Radius)


    #Cylinder10

    
   
    endeffvec1 = values[9]
    endeffvec2 = values[10]
    Cylinder10Point1 =np.array(endeffvec1)
    Cylinder10Point2 =np.array(endeffvec2)
    Cylinder10Radius =25.00/2

    CylinderBottom.append(Cylinder10Point1)
    CylinderTop.append(Cylinder10Point2)
    Radii.append(Cylinder10Radius)

    return(CylinderBottom,CylinderTop,Radii)

def GenerateCylinders_URCT(values):

    CylinderTop =[]
    CylinderBottom =[]
    Radii =[]

    #Cylinder 1

    Cylinder1Point1 =np.array([0,0,0])
    Cylinder1Point2 =values[0]+np.array([0,0,66.1495])
    Cylinder1Radius =116.00/2

    CylinderBottom.append(Cylinder1Point1)
    CylinderTop.append(Cylinder1Point2)
    Radii.append(Cylinder1Radius)

    #Cylinder 2

    r12vec = values[1] - values[0]
    r12vec = Vec2UnitVec(r12vec) 
    offset12 = 14.2
    r12vec1 = r12vec *(Cylinder1Radius + offset12/2) + values[0]
    r12vec2 = r12vec *(Cylinder1Radius + offset12 + 128.534) + values[0]
    Cylinder2Point1 =np.array(r12vec1)
    Cylinder2Point2 =np.array(r12vec2)
    Cylinder2Radius =116.00/2

    CylinderBottom.append(Cylinder2Point1)
    CylinderTop.append(Cylinder2Point2)
    Radii.append(Cylinder2Radius)

    #Cylinder 3

    r24vec = values[2] - values[1]
    r24vec = Vec2UnitVec(r24vec) 
    offset23 = 13.4
    r23vec1 = r24vec *(Cylinder2Radius+offset23/4) + values[1]
    r23vec2 = r24vec *(Cylinder2Radius+offset23 + 277.8+offset23+10) + values[1]
    Cylinder3Point1 =np.array(r23vec1)
    Cylinder3Point2 =np.array(r23vec2)
    Cylinder3Radius =85.00/2

    CylinderBottom.append(Cylinder3Point1)
    CylinderTop.append(Cylinder3Point2)
    Radii.append(Cylinder3Radius)

    #Cylinder 4
    
    r4J3vec = values[3] - values[2]
    r4J3vec = Vec2UnitVec(r4J3vec) 
    r4J3vec1 = r4J3vec *(-48) + values[2]
    r4J3vec2 = r4J3vec *(131.0756+65.1344) + values[2]
    Cylinder4Point1 =np.array(r4J3vec1)
    Cylinder4Point2 =np.array(r4J3vec2)
    Cylinder4Radius =116.00/2

    CylinderBottom.append(Cylinder4Point1)
    CylinderTop.append(Cylinder4Point2)
    Radii.append(Cylinder4Radius)

    #Cylinder 5

    r6J3vec = values[4] - values[3]
    r6J3vec = Vec2UnitVec(r6J3vec) 
    r6J3vec1 = r6J3vec * (65) + values[3]
    r6J3vec2 = r6J3vec * (77+263.20+5.60+5) + values[3]
    Cylinder5Point1 =np.array(r6J3vec1)
    Cylinder5Point2 =np.array(r6J3vec2)
    Cylinder5Radius =75.00/2

    CylinderBottom.append(Cylinder5Point1)
    CylinderTop.append(Cylinder5Point2)
    Radii.append(Cylinder5Radius)

    #Cylinder 6

    r67vec = values[5] - values[4]
    r67vec = Vec2UnitVec(r67vec) 
    r67vec1 = r67vec * (-56.05) + values[4]
    r67vec2 = r67vec * (88.20) + values[4]
    Cylinder6Point1 =np.array(r67vec1)
    Cylinder6Point2 =np.array(r67vec2)
    Cylinder6Radius =75.00/2

    CylinderBottom.append(Cylinder6Point1)
    CylinderTop.append(Cylinder6Point2)
    Radii.append(Cylinder6Radius)

    #Cylinder 7

    r78vec = values[6] - values[5]
    r78vec = Vec2UnitVec(r78vec)
    r78vec1 = r78vec *(-56.03) + values[5]
    r78vec2 = r78vec *(58) + values[5]
    Cylinder7Point1 =np.array(r78vec1)
    Cylinder7Point2 =np.array(r78vec2)
    Cylinder7Radius =75.00/2

    CylinderBottom.append(Cylinder7Point1)
    CylinderTop.append(Cylinder7Point2)
    Radii.append(Cylinder7Radius)

    #Cylinder 8

    r89vec= values[7] - values[6]
    r89vec= Vec2UnitVec(r89vec) 
    r89vec1 = r89vec * (-56.03) + values[6]
    r89vec2 = r89vec * (99.6)  + values[6]
    Cylinder8Point1 =np.array(r89vec1)
    Cylinder8Point2 =np.array(r89vec2)
    Cylinder8Radius =75.00/2

    CylinderBottom.append(Cylinder8Point1)
    CylinderTop.append(Cylinder8Point2)
    Radii.append(Cylinder8Radius)

    #Cylinder 9

    endeffvec = values[8] - values[7]
    endeffvec = Vec2UnitVec(endeffvec)
    endeffvec1 = endeffvec*5 +values[7]
    endeffvec2 = endeffvec*96.3 + values[7]
    Cylinder9Point1 = np.array(endeffvec1)
    Cylinder9Point2 = np.array(endeffvec2)
    Cylinder9Radius = 63.00/2

    CylinderBottom.append(Cylinder9Point1)
    CylinderTop.append(Cylinder9Point2)
    Radii.append(Cylinder9Radius)


    #Cylinder 10

    Cylinder10Point1 =np.array(values[9])
    Cylinder10Point2 =np.array(values[11])
    Cylinder10Radius = 18.50/2

    CylinderBottom.append(Cylinder10Point1)
    CylinderTop.append(Cylinder10Point2)
    Radii.append(Cylinder10Radius)

    #Cylinder 11 
    Cylinder11Point1 =np.array(values[10])
    Cylinder11Point2 =np.array(values[12])
    Cylinder11Radius = 18.50/2

    CylinderBottom.append(Cylinder11Point1)
    CylinderTop.append(Cylinder11Point2)
    Radii.append(Cylinder11Radius)

    #Cylinder 12 
    cylin12vec = values[13]- values[11]
    cylin12vec = Vec2UnitVec(cylin12vec)
    cylin12vec1 = (cylin12vec*5) +values[11]
    cylin12vec2= (cylin12vec*17.25) +values[11]
    Cylinder12Point1 =np.array(cylin12vec1)
    Cylinder12Point2 =np.array(cylin12vec2)
    Cylinder12Radius = 7.00/2

    CylinderBottom.append(Cylinder12Point1)
    CylinderTop.append(Cylinder12Point2)
    Radii.append(Cylinder12Radius)


    #Cylinder 13 
    cylin13vec = values[14]- values[12]
    cylin13vec = Vec2UnitVec(cylin13vec)
    cylin13vec1 = (cylin13vec*5) +values[12]
    cylin13vec2= (cylin13vec*17.25) +values[12]
    Cylinder13Point1 =np.array(cylin13vec1)
    Cylinder13Point2 =np.array(cylin13vec2)
    Cylinder13Radius = 7.00/2

    CylinderBottom.append(Cylinder13Point1)
    CylinderTop.append(Cylinder13Point2)
    Radii.append(Cylinder13Radius)


    #Cylinder 14
    Cylinder14Point1 =np.array(values[15])
    Cylinder14Point2 =np.array(values[16])
    Cylinder14Radius = 8.50/2

    CylinderBottom.append(Cylinder14Point1)
    CylinderTop.append(Cylinder14Point2)
    Radii.append(Cylinder14Radius)

    #Cylinder 15
    
    cylin15po1 = np.array((values[13]-values[11])/2) +values[11]
    cylin15po2 = np.array((values[14]-values[12])/2) +values[12]
    # print(cylin15po1)
    # print(cylin15po2)
    cylin15vec = cylin15po2 - cylin15po1
    # print(cylin15vec)
    cylin15len = Norm(cylin15vec)
    cylin15vec = Vec2UnitVec(cylin15vec)
    cylin15vec1 = cylin15vec*5 + cylin15po1
    cylin15vec2 = cylin15vec*(cylin15len-5) +cylin15po1
    # print(cylin15vec1)
    # print(cylin15vec2)
    Cylinder15Point1 =np.array(cylin15vec1)
    Cylinder15Point2 =np.array(cylin15vec2)
    Cylinder15Radius = Norm((np.array(values[14])-np.array(values[12])))/2.0
    # print(Cylinder15Point1.shape)
    # print(Cylinder15Point2.shape)
    # print(Cylinder15Point1)
    # print(Cylinder15Point2)
    # print(Cylinder15Radius)
    CylinderBottom.append(Cylinder15Point1)
    CylinderTop.append(Cylinder15Point2)
    Radii.append(Cylinder15Radius)

    # # #Cylinder9

    
    # # tcpendeffvec= values[8] - values[7]
    # # tcpendeffvec= Vec2UnitVec(tcpendeffvec) 
    # # tcpendeffvec1 = tcpendeffvec * (4)  + values[7]
    # # tcpendeffvec2 = tcpendeffvec * (47)  + values[7]
    # # Cylinder9Point1 =np.array(tcpendeffvec1 )
    # # Cylinder9Point2 =np.array(tcpendeffvec2)
    # # Cylinder9Radius =75.00/2

    # # CylinderBottom.append(Cylinder9Point1)
    # # CylinderTop.append(Cylinder9Point2)
    # # Radii.append(Cylinder9Radius)


    # #Cylinder91

    
    # tcpendeffvec= values[8] - values[7]
    # tcpendeffvec= Vec2UnitVec(tcpendeffvec) 
    # tcpendeff1vec1 = tcpendeffvec * (52)  + values[7]
    # tcpendeff1vec2 = tcpendeffvec * (225) + values[7]
    # Cylinder91Point1 =np.array(tcpendeff1vec1)
    # Cylinder91Point2 =np.array(tcpendeff1vec2)
    # Cylinder91Radius =25.00/2

    # CylinderBottom.append(Cylinder91Point1)
    # CylinderTop.append(Cylinder91Point2)
    # Radii.append(Cylinder91Radius)


    # #Cylinder10

    
   
    # endeffvec1 = values[9]
    # endeffvec2 = values[10]
    # Cylinder10Point1 =np.array(endeffvec1)
    # Cylinder10Point2 =np.array(endeffvec2)
    # Cylinder10Radius =25.00/2

    # CylinderBottom.append(Cylinder10Point1)
    # CylinderTop.append(Cylinder10Point2)
    # Radii.append(Cylinder10Radius)

    return(CylinderBottom,CylinderTop,Radii)     


def PlotFunction_Cylinders(CylinderBottom,CylinderTop,Radii,cjv):
    
    
    Plot_Figure = go.Figure()
    Plot_Figure.update_layout(plot_bgcolor='rgb(0,0,0,0)')
    Plot_Figure.update_xaxes(showgrid=False, zeroline=False)
    Plot_Figure.update_yaxes(showgrid=False, zeroline=False)
    #Needles-Robot
   
    colorscale= 'viridis'
    for i in range(len(CylinderBottom)):   
        Cylinder1_Point1 =CylinderBottom[i]
        Cylinder1_Point2 =CylinderTop[i]
        Radius_1 = Radii[i]    
        cyl_tube_1,cyl_bottom_1,cyl_top_1= Visualizing_Cylinder(Cylinder1_Point1, Cylinder1_Point2, Radius_1,colorscale)   
            
        Plot_Figure.add_trace(cyl_tube_1)
        Plot_Figure.add_trace(cyl_bottom_1)
        Plot_Figure.add_trace(cyl_top_1)  

    ShowCurrentPlot(Plot_Figure)

if __name__ == '__main__':
    #robot connection : 
    # robo = urx.Robot("172.16.101.224")
    # cjv = robo.getj()
    
    # # cjv = [-0.4830415884601038,
    # #     -1.403720812206604,
    # #     2.1135738531695765,
    # #     0.1315895754047851,
    # #     1.2138118743896484,
    # #     2.7860336303710938]
    # cjv = [ -0.5666477936566775,
    #         -0.13834939682472808,
    #         0.05094664158056192,
    #         -1.7558517732076324,
    #         -1.7655042854114067,
    #         0.7537769191337145]
    dumy,values = JointLocationsEF_URCT(cjv)
    print(values)
    
    CylinderBottom,CylinderTop,Radii = GenerateCylinders_URCT(values)
    PlotFunction_Cylinders(CylinderBottom,CylinderTop,Radii,cjv)
    # plot_figure = RobotBaseStructure(cjv)
    # ShowCurrentPlot(plot_figure)


    