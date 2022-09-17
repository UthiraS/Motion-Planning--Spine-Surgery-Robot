import plotly.graph_objects as go
import numpy as np
import math
import plotly.express as px
from decimal import *
from numpy import *
from scipy.linalg import norm

from CylinderCollision import *
from modules import *
from PlotsFunction import *

def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")

def EuclideanDistance(vec1,vec2):
    E2Distance = np.sqrt( np.square(vec1[0]- vec2[0]) + np.square(vec1[1]- vec2[1]) + np.square(vec1[2]- vec2[2]))
    return E2Distance

#Rodrigues form :
def rodrot(vec,axis,theta):
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

def AnglebtwnVecs(vec1,vec2):
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    # print("Angle : {}".format((angle*180)/math.pi))
    return angle

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

# ********PATHS***********

def SmoothPathPoints(a,b,h,pt1):
   
    global distance
    distance = []
    c = [(a[0]+b[0])/2,(a[1]+b[1])/2]
    vector = np.subtract(b,a)
    dist = norm(vector)
    div = dist*6
    tmin = 0
    tmax = np.pi
    cmin = a[0]
    cmax = b[0]
    bmin = a[1]
    bmax = b[1]
    C = linspace(cmin, cmax, div)
    t = linspace(tmin, tmax, div)
    B = linspace(bmin, bmax, div)
    y = (h)*(sin((t)) ) + B


    for i in range(0,len(C)-1):
        for j in range(0,len(pt1)):
            if C[i+1] >= pt1[j][0] and C[i] <= pt1[j][0]:
                magni = np.sqrt((C[i]-pt1[j][0])**2 + (B[i]-pt1[j][1])**2)
                magni1 = np.sqrt((C[i]-C[i+1])**2 + (y[i]-y[i+1])**2)
                p1 = [C[i],y[i]]
                p2 = [C[i+1],y[i+1]]
                v = np.subtract(p2,p1)
                scale = (magni/magni1)
                d = scale + magni
                l2 = p1 + (d*(v))
                if pt1[j] == b:
                    l2 = b
                dist = np.sqrt((pt1[j][0]-l2[0])**2 + (pt1[j][1]-l2[1])**2)
                distance.append(dist)
   
    return distance



def SubPathPoints(VecMag,SF):
    CenDist =  SF
    Factors = [0,0.15,0.3,0.45,0.6,0.75,0.85,0.95]
    hstack1 = list(np.array(Factors) * CenDist)
    hstack2 = list(np.flip(hstack1))
    hstack1.append(CenDist)
    GroundDistances = hstack1 + hstack2

    Div_factor2 = float(Decimal(VecMag) / Decimal(16))
    istack = list(np.arange(0,VecMag,Div_factor2)) 
    istack.append(VecMag)
    istack = list(np.flip(istack))
    
    return GroundDistances,istack

def SubPathPoints2(VecMag,SF):

    # SF = 100
    #Needed Changes:
    Div_factor2 = float(Decimal(VecMag) / Decimal(16))
    istack = list(np.arange(0,VecMag,round(Div_factor2,4)))
    istack.append(VecMag)
 
    istack = list(np.flip(istack))
    if int(istack[0]) == int(istack[1]):
        istack.remove(istack[1])

    Out = []
    In = []

    for i in istack:
        In.append(i)
        In.append(0)
        Out.append(In)
        In = []
 
    if int(Out[0][0]) == int(Out[1][0]):
        Out.remove(Out[1])
    start = [0,0]
    end = [VecMag,0]

    GroundDistances = SmoothPathPoints(start,end,SF,Out)
    if GroundDistances[-1] != 0.0:
        GroundDistances.remove(GroundDistances[-1])

    return GroundDistances,istack


def GeneratePathPoints(InitialPoint,FinalPoint,flag,theta,CentreDistance):
    
    points = []
    PlanarPoint = [FinalPoint[0],FinalPoint[1],InitialPoint[2]]
    PlanarVector = np.asarray(PlanarPoint) - np.asarray(InitialPoint)
    
    PathVector = np.asarray(FinalPoint) - np.asarray(InitialPoint)
    LCAxisVectorY = np.cross(PathVector,PlanarVector)
    LCAxisVectorZ = np.cross(LCAxisVectorY,PlanarVector)

    E2Distance = np.sqrt( np.square(FinalPoint[0]- InitialPoint[0]) + np.square(FinalPoint[1]- InitialPoint[1]) + np.square(FinalPoint[2]- InitialPoint[2]))
    ActualAngle = AnglebtwnVecs(PathVector,PlanarVector)

    UaxisX = Vec2UnitVec(PlanarVector)
    UaxisY = Vec2UnitVec(LCAxisVectorY)
    UaxisZ = Vec2UnitVec(LCAxisVectorZ)
        
    RotationVector1 = UaxisX
    AxisVector = UaxisY
    LCX = rodrot(RotationVector1 * E2Distance,AxisVector,-ActualAngle)
    RotationVector2 = UaxisZ
    LCY = rodrot(RotationVector2 * E2Distance,AxisVector,-ActualAngle)
    
    CuLCX = Vec2UnitVec(LCX)

    if flag == 1:
        LCY = rodrot(LCY,CuLCX,theta)

    LCZ = np.cross(LCX,LCY)


    # PathVectorMD = InitialPoint + (Vec2UnitVec(LCX) * E2Distance/2)
    PathGenVector = Vec2UnitVec(LCY)
    PathGenAxis = Vec2UnitVec(LCZ)
    
    #Choose smoothing or not:
    # Gpoints,ipoints = SubPathPoints2( E2Distance,CentreDistance)
    Gpoints,ipoints = SubPathPoints2( E2Distance,CentreDistance)
    
    T_vector = Vec2UnitVec(PathVector)
    
    for i in range(len(ipoints)):
    
        Rp = InitialPoint + (T_vector  * ipoints[i]) + (PathGenVector * Gpoints[i])
        points.append(Rp)
    
    aapoints = np.asarray(points)
    xpoints = list(aapoints[:,0])
    ypoints = list(aapoints[:,1])
    zpoints = list(aapoints[:,2])
    
    return xpoints,ypoints,zpoints,points


def PathPlanner(Initial_point, ICD, Final_point, EntryList, TargetList, threshold,Needle_radius, Plot_Figure):

    #variables:
    inc_distance = ICD
    loop_begins = 1

    while (loop_begins == 1):

        if inc_distance > threshold :
            print("Path Cannot be Found, try different planning")
            Plot_Figure = PlotFunction_CylinderPoints(EntryList,TargetList,Needle_radius,Plot_Figure)
            return 0,Plot_Figure,0,0,0

        loop_begins_1 = 1
        inc_distance += 10
        
        xpoints,ypoints,zpoints,Tpoints = GeneratePathPoints(Initial_point,Final_point,1,0,inc_distance)
             
        PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
        
        PPstatus1 = Cylinder_Point_Collision(EntryList,TargetList,Needle_radius,Tpoints)
        
        
        
        if PPstatus1 == 0: #no collision
            loop_begins = 0
            loop_begins_1 = 0
            [xpoints_,ypoints_,zpoints_],PATHSTACKS = PathOptimization(Initial_point,Final_point,0,inc_distance,EntryList, TargetList,Needle_radius,Plot_Figure)  
            
            if xpoints_[0] != 0 and ypoints_[0] != 0:
                PathCoordinates = [np.flip(xpoints_),np.flip(ypoints_),np.flip(zpoints_)]
                print("Found a Optimal Collision Free Path")
                Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints_,ypoints_,zpoints_)
                #Plot_Figure = PlotFunction_CylinderPoints(EntryList,TargetList,Needle_radius,Plot_Figure)

            else: 
                print("Found a Collision Free Path")
                Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
                #Plot_Figure = PlotFunction_CylinderPoints(EntryList,TargetList,Needle_radius,Plot_Figure)
            
            break 
        
        val = 0
        while (loop_begins_1 == 1):
            
            val = val + 0.1
            xpoints,ypoints,zpoints,Tpoints = GeneratePathPoints(Initial_point,Final_point,1,val,inc_distance)
            PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
            PPstatus1 = Cylinder_Point_Collision(EntryList,TargetList,Needle_radius,Tpoints)
            
            if PPstatus1 == 0:#no collision
                loop_begins_1 = 0
                [xpoints_,ypoints_,zpoints_],PATHSTACKS = PathOptimization(Initial_point,Final_point,val,inc_distance,EntryList, TargetList,Needle_radius,Plot_Figure) 
                
                if xpoints_ != 0 and ypoints_ != 0:
                    PathCoordinates = [np.flip(xpoints_),np.flip(ypoints_),np.flip(zpoints_)]
                
                    print("Found a Optimal Collision Free Path (Angle Change)")
                    Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints_,ypoints_,zpoints_)
                    #Plot_Figure = PlotFunction_CylinderPoints(EntryList,TargetList,Needle_radius,Plot_Figure)
                    loop_begins = 0
                else:
                    print("Found a Collision Free Path (Angle Change)")
                    Plot_Figure,pathpoints = TracePath(Plot_Figure,xpoints,ypoints,zpoints)
                    #Plot_Figure = PlotFunction_CylinderPoints(EntryList,TargetList,Needle_radius,Plot_Figure)
                    loop_begins = 0
                
                break
    
            if val > 2*math.pi:
                loop_begins_1 = 0
                
        
    return PathCoordinates, Plot_Figure, pathpoints,PATHSTACKS,inc_distance


def PathOptimization(Initial_point,Final_point,Angle,inc_distance,EntryList, TargetList,Needle_radius,Plot_Figure):

    initial_threshold = 0
    Optimized_coordinates = [0,0,0]
    Optimized_stack = []
    Pathstackvalue = 1
    Final_optimized_stack = []

    for B_p in np.arange(Angle,2*math.pi,0.1):
        xpoints,ypoints,zpoints,Tpoints = GeneratePathPoints(Initial_point,Final_point,1,B_p,inc_distance)
        PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
        PPstatus_1 = Cylinder_Point_Collision(EntryList,TargetList,Needle_radius,Tpoints)
        # print(PPstatus_1)
        #finding least distance:
        if PPstatus_1 == 0: #no collision
            I_point = 0
            for j in range(len(EntryList)):
                for i in range(4):
                    P_point = [xpoints[i+1],ypoints[i+1],zpoints[i+1]]
                    O_point = EntryList[j]
                    dist_ = EuclideanDistance(P_point,O_point)
                    I_point = I_point + dist_
            I_point = I_point / (len(EntryList) * 4)
            
            if I_point > initial_threshold :
                initial_threshold = I_point
                Optimized_coordinates = PathCoordinates
                Optimized_stack.append(B_p)
    
    
    if len(Optimized_stack) < Pathstackvalue :

        return Optimized_coordinates,Final_optimized_stack 
    else:
        Dummyvalue = Optimized_stack[-Pathstackvalue:]
        for sp in Dummyvalue:
            xpoints,ypoints,zpoints,Tpoints = GeneratePathPoints(Initial_point,Final_point,1,sp,inc_distance)
            PathCoordinates = [np.flip(xpoints),np.flip(ypoints),np.flip(zpoints)]
            Final_optimized_stack.append(PathCoordinates)
            
            
        return Optimized_coordinates,Final_optimized_stack
        
               