#import Packages:
import urx
import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *
import plotly.graph_objects as go
import sys


#robot connection : 
# robo = urx.Robot("172.16.101.224")

#Defined Functions : 

# Visualization
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

def ComputeEFPoints(theta4,theta5,theta6): 
    
    angle = math.pi / 4
    vec56 = theta6 - theta5
    Uvec56 = Vec2UnitVec(vec56)
    vec45 = theta4 - theta5
    Uvec45 = Vec2UnitVec(vec45)
    crossvec = np.cross(Uvec56,Uvec45)
    EFMidPoint = (theta6 + Uvec56 * 245)
    EFEntryPoint = EFMidPoint +  (rodrot(Uvec45,crossvec,angle) * 32.5264)
    EFTargetPoint = EFEntryPoint + (Vec2UnitVec(EFMidPoint - EFEntryPoint) * 50)
    return [EFMidPoint,EFEntryPoint,EFTargetPoint]

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
    EFm, EFe,EFt = ComputeEFPoints(Stheta4,Stheta5,Stheta6)
    
    Spositions = [Stheta1,theta12,theta34,Stheta2,Stheta3,Stheta4,Stheta5,Stheta6,EFm,EFe,EFt]

    
    positions = [theta1,theta12/1000,theta34/1000,theta2,theta3,theta4,theta5,theta6,EFm/1000,EFe/1000,EFt/1000]
    
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

    
    


if __name__ == '__main__':
    
    cjv = robo.getj()
    dumy,values = JointLocationsEF(cjv)
    print(values)
    
    plot_figure = RobotBaseStructure(cjv)
    ShowCurrentPlot(plot_figure)


    