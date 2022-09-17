#!/usr/bin/env python
# coding: utf-8

import math
import numpy as np
from numpy.lib.function_base import _diff_dispatcher
import plotly
import plotly.graph_objects as go

from numpy.linalg import norm
from RobotSelfCollisionModified import GenerateCylinders, GenerateCylinders_URCT

# from PlotsFunction import *
np.seterr(divide='ignore', invalid='ignore')



# prerequisite functions
def AnglebtwnVecs(vec1,vec2):   
    #Angle between vec1 and vec2
    angle = np.arccos(np.dot(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
    #print("Angle : {}".format((angle*180)/math.pi))
    return angle

def Norm(vec):   
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))    
    return norm

def rodrot(vec,axis,theta):
    # Rotating vec wrt axis by thetha angle using Rodrigues equation
    comp1 = vec * np.cos(theta)
    comp2 = np.cross(axis,vec) * np.sin(theta)
    comp3 = (axis * (np.dot(axis,vec)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    return Rodgriues

def Vec2UnitVec(vec):
    #Unit vector in the direction of vec
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def projection_AonB_(a,b):
    #Projection of vector a on b
    b_norm = np.sqrt(sum(b**2))
    proj_of_a_on_b = (np.dot(a, b)/b_norm**2)*b 
    return(proj_of_a_on_b)

#Distance calculation function    

def calculate_distance_projection(Cylinder_Point1,Cylinder_Point2,r,CollisionPoint):
    #Finding projection of collision vector onto the cylindrical axis 
    #and calculating the shortest distance between collision point and cylindrical axis

    Cylinderaxis = Cylinder_Point2 -Cylinder_Point1   
    

    Projectionpoint = projection_AonB_(CollisionPoint-Cylinder_Point1,Cylinderaxis) + Cylinder_Point1
    squared_distance=np.linalg.norm(Projectionpoint-CollisionPoint)-r
    
    #print(squared_distance)
    ratio = (r/(squared_distance + r))
    point = (1-ratio)*Projectionpoint + ratio*(CollisionPoint)

    return(squared_distance,point) 





def calculate_distance_rotation(Cylinder_Point1,Cylinder_Point2,r,CollisionPoint,case):
    #Rotating the collision vector onto the axis perpendicular the cylindrical axis 
    #and calculating the shortest distance between collision point and cylindrical axis

    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1
    CollisionVector = CollisionPoint - Cylinder_Point2
    
    U_CollisionVec = Vec2UnitVec(CollisionVector)
    U_AxisVec = Vec2UnitVec(Cylinderaxis)
    
    angle = AnglebtwnVecs(CollisionVector,Cylinderaxis)
    rotation_angle = np.pi/2 - angle
    
    Rotational_axis = np.cross(U_AxisVec,U_CollisionVec)
    Rotational_axis = Vec2UnitVec(Rotational_axis)
    
    rotatedvec = rodrot(U_CollisionVec,Rotational_axis,rotation_angle)
    
    final_vec = rotatedvec * r
    if(case == "top"):
        final_vec1 = Cylinder_Point2 + final_vec
    elif(case == "bottom"):
        final_vec1 = Cylinder_Point1 + final_vec
    point = final_vec1
    
    squared_distance = np.linalg.norm(CollisionPoint-final_vec1)
    
    return(squared_distance,point)





def calculate_distance_rotation_projection(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint,case):
    #Rotating the collision vector onto the axis perpendicular the cylindrical axis and finding its projection on the rotated
    #and calculating the shortest distance between collision point and cylindrical axis
    
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1
    CollisionVector = CollisionPoint - Cylinder_Point2
    
    U_CollisionVec = Vec2UnitVec(CollisionVector)
    U_AxisVec = Vec2UnitVec(Cylinderaxis)
    
    angle = AnglebtwnVecs(CollisionVector,Cylinderaxis)
    rotation_angle = np.pi/2 - angle
    
    Rotational_axis = np.cross(U_AxisVec,U_CollisionVec)
    Rotational_axis = Vec2UnitVec(Rotational_axis)
    
    rotatedvec = rodrot(U_CollisionVec,Rotational_axis,rotation_angle)
    
    final_vec = rotatedvec * r
    #final_vec1 = Cylinder_Point2 + final_vec
    
    final_unit_vec = Vec2UnitVec(rotatedvec)
    checking_vector = projection_AonB_(CollisionVector,final_vec)
    if(case == "top"):
        checking_vector1 = Cylinder_Point2 + checking_vector 
    elif(case == "bottom"):
        checking_vector1 = Cylinder_Point1 + checking_vector 
    
    point = checking_vector1
    squared_distance = np.linalg.norm(CollisionPoint-checking_vector1)
    
    return(squared_distance,point)





def points_in_cylinder(Cylinder_Point1,Cylinder_Point2,r,CollisionPoint):
   #checking if a Collision point lies inside,on or outside and printing the distance from the surface 
    
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1  
    Cylinderaxisdown = Cylinder_Point1 - Cylinder_Point2 
    const = r * np.linalg.norm(Cylinderaxis)
    
    # Collision Point lies inside the cylindrical surface
    if(np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) > 0 and np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) < 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point1, Cylinderaxis)) < const):
        return -1 ,0,CollisionPoint

    # Collision Point lies on the cylindrical surface
    elif(np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) == 0 or np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) == 0 or np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point1, Cylinderaxis)) == const ):      
        return 0, 0,CollisionPoint

    # Collision Point lies outside the cylindrical surface above the cylinder
    elif(np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) < 0 or np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) > 0 or np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point1, Cylinderaxis)) > const ):
       
       # Case A....Point is between the planes of circular ends of cylinder and outside the cylindrical surface 
        if(np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) > 0 and np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) < 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point2, Cylinderaxis)) > const):

            
            squared_distance,point = calculate_distance_projection(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint)   
            return 1, squared_distance, point 
        
       # Case B .. Point is above the top surface of the cylinder and within the extened cylindrical circumference or outside)
        elif((np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) > 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point2, Cylinderaxis)) < const)):
            
            case = "top"
            squared_distance,point = calculate_distance_rotation_projection(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint,case)
            return 1, squared_distance, point 
    
       # Case C .. Point is above the top surface of the cylinder and outside the extened cylindrical circumference or outside)
        elif((np.dot(CollisionPoint - Cylinder_Point2, Cylinderaxis) > 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point2, Cylinderaxis)) > const)):
        
            case = "top"
            squared_distance,point = calculate_distance_rotation(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint,case)   
            return 1, squared_distance, point
        
        #Case D .. Point is below the bottom surface of the cylinder and inside the extened cylindrical circumference or outside)
        elif((np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) < 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point1, Cylinderaxis)) < const)):
            
            case ="bottom"
            squared_distance,point = calculate_distance_rotation_projection(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint,case)
            return 1, squared_distance, point
    
       # Case E .. Point is below the bottom surface of the cylinder and outside the extened cylindrical circumference or outside)
        elif((np.dot(CollisionPoint - Cylinder_Point1, Cylinderaxis) < 0 and np.linalg.norm(np.cross(CollisionPoint - Cylinder_Point1, Cylinderaxis)) > const)):
        
            case ="bottom"
            squared_distance,point = calculate_distance_rotation(Cylinder_Point1, Cylinder_Point2, r, CollisionPoint,case)   
            return 1, squared_distance, point

def Points_checking(Cylinder_Point1,Cylinder_Point2,r,Point):
    #checking if a Collision point lies inside,on or outside 
    
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1  
    const = r * np.linalg.norm(Cylinderaxis)
    
    # Intersection Point lies inside the cylindrical surface
    if(np.dot(Point - Cylinder_Point1, Cylinderaxis) > 0 and np.dot(Point - Cylinder_Point2, Cylinderaxis) < 0 and np.linalg.norm(np.cross(Point - Cylinder_Point1, Cylinderaxis)) < const):
        #print("Point lies inside the cylinder")
        return -1 

    # Intersection Point lies on the cylindrical surface
    elif(np.dot(Point - Cylinder_Point1, Cylinderaxis) == 0 or np.dot(Point - Cylinder_Point2, Cylinderaxis) == 0 or np.linalg.norm(np.cross(Point - Cylinder_Point1, Cylinderaxis)) == const ):      
        #print("Point lies on the cylinder")
        return 0

    # Intersection Point lies outside the cylindrical surface
    elif(np.dot(Point - Cylinder_Point1, Cylinderaxis) < 0 or np.dot(Point - Cylinder_Point2, Cylinderaxis) > 0 or np.linalg.norm(np.cross(Point - Cylinder_Point1, Cylinderaxis)) > const ):
        #print("Point lies outside the cylinder")
        return 1


#Cylinder to Lines function
def LinesCylinder(Cylinder_Point1,Cylinder_Point2,r,numlines,NumPoints):
    #Approximating a cylinder as combination of n lines 
    
    
    ##generating three perpendicular on top surface end point
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1
    U_AxisVec = Vec2UnitVec(Cylinderaxis)
    mag = Norm(Cylinderaxis)

    #unit vector in direction of axis
    Cylinderaxis = Cylinderaxis / mag

    #make some vector not in the same direction as Cylinderaxis
    not_Cylinderaxis = np.array([1, 0, 0])
    if (Cylinderaxis == not_Cylinderaxis).all():
        not_Cylinderaxis = np.array([0, 1, 0])

    #make vector perpendicular to Cylinderaxis
    n1 = np.cross(Cylinderaxis, not_Cylinderaxis)
    n2 = np.cross(Cylinderaxis, n1)
    
    
   
    ##generating three perpendicular on bottom surface end point
    #by shifting the unit vector to below
    Cylinderaxisdown = Cylinder_Point2 - Cylinder_Point1
    U_AxisVecDown = Vec2UnitVec(Cylinderaxisdown)
    mag_down = Norm(Cylinderaxisdown)

    #unit vector in direction of axis
    Cylinderaxisdown = Cylinderaxisdown / mag_down

    #make some vector not in the same direction as Cylinderaxis
    not_Cylinderaxisdown = np.array([1, 0, 0])
    if (Cylinderaxisdown == not_Cylinderaxisdown).all():
        not_Cylinderaxisdown = np.array([0, 1, 0])

    #make vector perpendicular to Cylinderaxis
    n1_down = np.cross(Cylinderaxisdown, not_Cylinderaxisdown)
    n2_down = np.cross(Cylinderaxisdown, n1_down)
        
   
    
    
    Points_Top=[]
    Points_Bottom=[]
    Pts_T=[]
    Pts_B=[]
    
    
    min_angle = (2*np.pi)/numlines
    #generating rotated lines in both top surface and bottom surface
    angles =[i*min_angle for i in range(1,numlines+1)]
    
    for theta in angles:
        #Rodrigues rotation in top surface
        rotatedvec = rodrot(n1,U_AxisVec,theta)
        rotatedvec = rotatedvec/norm(rotatedvec)        
        final_vec = rotatedvec *r  
        final_vec2 = final_vec + Cylinder_Point2        
        
        
        Points_Top.append(final_vec.tolist)
        pt_top_final = final_vec2.tolist()
        Pts_T.append(pt_top_final)
        
        #Rodrigues rotation in bottom surface
        rotatedvec_down = rodrot(n1_down,U_AxisVecDown,theta)
        rotatedvec_down =rotatedvec_down/norm(rotatedvec_down)
        final_vec_down = rotatedvec_down *r  
        final_vec2_down = final_vec_down + Cylinder_Point1        
        
        Points_Bottom.append(final_vec_down)
        pt_bottom_final = final_vec2_down.tolist()
        Pts_B.append(pt_bottom_final)
   
    return(Pts_T,Pts_B)

def LinesCylinderPlot(Cylinder_Point1,Cylinder_Point2,r,numlines,NumPoints):
    #Approximating a cylinder as combination of n lines 
    
    
    ##generating three perpendicular on top surface end point
    Cylinderaxis = Cylinder_Point2 - Cylinder_Point1
    U_AxisVec = Vec2UnitVec(Cylinderaxis)
    mag = Norm(Cylinderaxis)

    #unit vector in direction of axis
    Cylinderaxis = Cylinderaxis / mag

    #make some vector not in the same direction as Cylinderaxis
    not_Cylinderaxis = np.array([1, 0, 0])
    if (Cylinderaxis == not_Cylinderaxis).all():
        not_Cylinderaxis = np.array([0, 1, 0])

    #make vector perpendicular to Cylinderaxis
    n1 = np.cross(Cylinderaxis, not_Cylinderaxis)
    n2 = np.cross(Cylinderaxis, n1)
    
    
   
    ##generating three perpendicular on bottom surface end point
    #by shifting the unit vector to below
    Cylinderaxisdown = Cylinder_Point2 - Cylinder_Point1
    U_AxisVecDown = Vec2UnitVec(Cylinderaxisdown)
    mag_down = Norm(Cylinderaxisdown)

    #unit vector in direction of axis
    Cylinderaxisdown = Cylinderaxisdown / mag_down

    #make some vector not in the same direction as Cylinderaxis
    not_Cylinderaxisdown = np.array([1, 0, 0])
    if (Cylinderaxisdown == not_Cylinderaxisdown).all():
        not_Cylinderaxisdown = np.array([0, 1, 0])

    #make vector perpendicular to Cylinderaxis
    n1_down = np.cross(Cylinderaxisdown, not_Cylinderaxisdown)
    n2_down = np.cross(Cylinderaxisdown, n1_down)
        
   
    Points_Top=[]
    Points_Bottom=[]
    Pts_T=[]
    Pts_B=[]
    
    
    min_angle = (2*np.pi)/numlines
    #generating rotated lines in both top surface and bottom surface
    angles =[i*min_angle for i in range(1,numlines+1)]
    
    for theta in angles:
        #Rodrigues rotation in top surface
        rotatedvec = rodrot(n1,U_AxisVec,theta)
        rotatedvec = rotatedvec/norm(rotatedvec)        
        final_vec = rotatedvec *r  
        final_vec2 = final_vec + Cylinder_Point2        
        
        
        Points_Top.append(final_vec.tolist)
        pt_top_final = final_vec2.tolist()
        Pts_T.append(pt_top_final)
        
        #Rodrigues rotation in bottom surface
        rotatedvec_down = rodrot(n1_down,U_AxisVecDown,theta)
        rotatedvec_down =rotatedvec_down/norm(rotatedvec_down)
        final_vec_down = rotatedvec_down *r  
        final_vec2_down = final_vec_down + Cylinder_Point1        
        
        Points_Bottom.append(final_vec_down)
        pt_bottom_final = final_vec2_down.tolist()
        Pts_B.append(pt_bottom_final)
        
    Px =[]
    Py =[]
    Pz =[]
    for i in range(numlines):        
        # Printing approximated lines with interpolated points in between 
        Points=Interpolation_line(Pts_B[i],Pts_T[i],NumPoints)
        
        
        for j in range(len(Points)):            
            Px.append(Points[j][0])
            Py.append(Points[j][1])
            Pz.append(Points[j][2])
        Px.append(Pts_B[i][0])
        Px.append(Pts_T[i][0])
        Py.append(Pts_B[i][1])
        Py.append(Pts_T[i][1])
        Pz.append(Pts_B[i][2])
        Pz.append(Pts_T[i][2])
            
    Line_Segment = go.Scatter3d(x = Px,
                                y = Py,
                                z = Pz, 
                                mode='markers',                              
                                marker=dict(
                                size=4,
                                color='black',               
                                colorscale='Viridis',  
                                line=dict(width=1,color='DarkSlateGrey'),
                                opacity=1))

      
    return(Line_Segment)
     
#Line to Points function

def Interpolation_line(A,B, numpoints):
    A = np.array(A)
    B= np.array(B)
    #generating all the intermediate points on the line segment with end points A and B
    Vector = B-A
    UnitVector =Vec2UnitVec(Vector)
    Magnitude= Norm(Vector)
   
    Points=[]
    pts=[]
    # Points.append(A)
    PrevVector =A
    scale = Magnitude/(numpoints+1)
    UnitVector1 = (UnitVector)*scale
   
    for j in range(numpoints):
        NewUnitVector = PrevVector +(j+1)*UnitVector1         
        NewUnitvector1 = NewUnitVector+A
        Points.append(NewUnitVector)
        k= NewUnitVector.tolist()
        pts.append(k)
    
    return(Points)

# Cylinder- Line intersection checking function
def Distance_Points(Cylinder_Point1,Cylinder_Point2,r,CollisionPoints):
    
    #Checking if each point in a set of collision point lies inside,on or outside the cylinder
    #checking the distance of each point from cylindrical surface
    distance = 0    
    flag=0
    distances=[]
    
    for point in CollisionPoints:
        position ,distance,point1 =points_in_cylinder(Cylinder_Point1,Cylinder_Point2,r,point)
        #creating a array of distance of each point       
        distances.append(distance)
        
        #if distance of a point from cylindrical surface if 0 or negative, the line intersects the cylinder
        if((distance == 0)or(distance <0)):
            flag= -1
            break
            #print("intersecting")
        
    #checking if distance of any point on the line from cylindrical surface is less than 0.1 and
    #distance of previous point and next point are both greater than the distance of the current point
    #then, there is possible intersection
    for k in range(len(distances)-1):
        if((distances[k]<0.1) and (distances[k]<distances[k-1]) and (distances[k]<distances[k+1])):
            flag = -1
            #print("possible intersection")
            
    #flag = -1 implies intersection       
    if(flag==-1):        
        return -1
    #flag ==0 implies there is no intersection
    elif(flag==0):        
        return 1


#Functions 

#Cylinder- Cylinder intersection checking function
def Cylinder_Cylinder(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Cylinder2_Point1,Cylinder2_Point2,Radius_2,NumLines,NumPoints):
     
    
    
    intersected =0
    #Line Segment Points Top and Bottom
    Points_Top,Points_Bottom=LinesCylinder(Cylinder2_Point1,Cylinder2_Point2,Radius_2,NumLines,NumPoints)
    Points_Top.append(Cylinder2_Point1)
    Points_Bottom.append(Cylinder2_Point2)
    

   
    if(Points_checking(Cylinder2_Point1 ,Cylinder2_Point2,Radius_2,Cylinder2_Point1)==-1):
            intersected =1
           
       
    elif(Points_checking(Cylinder2_Point1 ,Cylinder2_Point2,Radius_2,Cylinder2_Point2)==-1):
            intersected=1
        
    else:
        for i in range(len(Points_Top)):
            Line_p1 = Points_Top[i]
            Line_p2 = Points_Bottom[i]
            
            
            if((Points_checking(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Line_p1) == -1)or(Points_checking(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Line_p1) == 0)or(Points_checking(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Line_p2) == -1)or(Points_checking(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Line_p2) == 0)):
                # print("Line segment intersecting with Cylinder Case A,!")
                intersected =1
                break
            
            #interpolated the lines into set of points and check distance, based on distance check if there is possible collision
            else:   
                #print(Distance_Points(Cylinder1_Point1,Cylinder1_Point2,Radius_1,Interpolation_line(Line_p1,Line_p2,NumPoints)))       
                if(Distance_Points(Cylinder1_Point1,Cylinder1_Point2,Radius_1,Interpolation_line(Line_p1,Line_p2,NumPoints))==-1):
                    
                    intersected = 1
                    # print("possible intersection")
                    break
                elif(Distance_Points(Cylinder1_Point1,Cylinder1_Point2,Radius_1,Interpolation_line(Line_p1,Line_p2,NumPoints))==1):
                    
                    intersected = 0                         

    return(intersected)

def Cylinder_Point_Collision(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,CollisionPoints):
    flags =[]
    #Adding all the (Collision Cylinders to plot)
    
    for i in range(len(Cylinder1_Point_Bottom)):   
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]     
        distances=[]
        statuses=[]
        flag =0
        for j in range(len(CollisionPoints)):
            
            status,distance, Point = points_in_cylinder(Cylinder1_Point1,Cylinder1_Point2,Radius_1,CollisionPoints[j])        
            
            statuses.append(status)
            distances.append(distance)
            #print(status,distance)
       
        if((-1 in statuses) or (0 in statuses)):
            flag =1 # intersection
            break
            
        elif(1 in statuses):
            flag =0 # no intersection
        
        flags.append(flag)

    if(0 in flags):
        # print(0)       
        return 0
    elif(1 in flags):
        # print(1)
        return 1
   
def WhichNeedle(A,ETCList,leng):
    # print("Inside Which Needle")
    
    for i in range(len(ETCList)):
        
        for j in range(leng):
            # print("A",A)
            # print(ETCList[i][j])
            if np.array_equal(A,ETCList[i][j]) is True:
                flag = i                
                break
    
    return flag

def CC_Collision(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder2_Point_Bottom,Cylinder2_Point_Top,Radii_2,NumLines,NumPoints):
    intersected =0
    # print("Inside Robot and needle collision check")
    for i in range(len(Cylinder1_Point_Bottom)):        
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]      
        
        for j in range(len(Cylinder2_Point_Bottom)):         
            
            Cylinder2_Point1 =Cylinder2_Point_Bottom[j]        
            Cylinder2_Point2 =Cylinder2_Point_Top[j]            
            Radius_2 = Radii_2[j]     
           
            if(Cylinder_Cylinder(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Cylinder2_Point1,Cylinder2_Point2,Radius_2,NumLines,NumPoints) ==1 ):
                #print("(",i+1," , ",j+1,")") 
                # if(statustype == "Plate"):
                #     counter = WhichNeedle(Cylinder2_Point1,ETCList,4) 
                #     print("Collision between Robot and Needle placed in "+EntryTargetStringList[counter])
                # elif(statustype == "restAPI"):
                #     counter = WhichNeedle(Cylinder2_Point1,ETCList,2) 
                #     print("Collision between Robot and Needle "+EntryTargetStringList[counter])
                # counter = WhichNeedle(Cylinder2_Point1,ETCList)                
                
                # print("Collision between Robot and Needle placed in "+EntryTargetStringList[counter])
                intersected =1 
                break    

    if(intersected == 1):
        # print(1) #Intersection
        return 1
        
    elif(intersected == 0):
        # print(0)    #No intersection
        return 0
def Cylinder_Cylinder_Collision(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder2_Point_Bottom,Cylinder2_Point_Top,Radii_2,NumLines,NumPoints,ETCList,EntryTargetStringList,statustype):
    intersected =0
    # print("Inside Robot and needle collision check")
    for i in range(len(Cylinder1_Point_Bottom)):        
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]      
        
        for j in range(len(Cylinder2_Point_Bottom)):         
            
            Cylinder2_Point1 =Cylinder2_Point_Bottom[j]        
            Cylinder2_Point2 =Cylinder2_Point_Top[j]            
            Radius_2 = Radii_2[j]     
           
            if(Cylinder_Cylinder(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Cylinder2_Point1,Cylinder2_Point2,Radius_2,NumLines,NumPoints) ==1 ):
                #print("(",i+1," , ",j+1,")") 
                if(statustype == "Plate"):
                    counter = WhichNeedle(Cylinder2_Point1,ETCList,4) 
                    print("Collision between Robot and Needle placed in "+EntryTargetStringList[counter])
                elif(statustype == "restAPI"):
                    counter = WhichNeedle(Cylinder2_Point1,ETCList,2) 
                    print("Collision between Robot and Needle "+EntryTargetStringList[counter])
                # counter = WhichNeedle(Cylinder2_Point1,ETCList)                
                
                # print("Collision between Robot and Needle placed in "+EntryTargetStringList[counter])
                intersected =1 
                break    

    if(intersected == 1):
        # print(1) #Intersection
        return 1
        
    elif(intersected == 0):
        # print(0)    #No intersection
        return 0


def CheckNeedleSelfCollision(Entry_list,Target_list,X_Cods,Y_Cods,Z_Cods,Needle_radius,S_Fk,ETCList,EntryTargetStringList,statustype):

    collision_check_count = 3 #No.of path points collision check
    startpoint = 20 - collision_check_count  #(Totalpathpoint - checkpoints)

    
    for cds in range(collision_check_count):
        
        current_frame = [X_Cods[startpoint],Y_Cods[startpoint],Z_Cods[startpoint]]
        
        current_data = S_Fk[startpoint]
        # print(X_Cods[startpoint],Y_Cods[startpoint],Z_Cods[startpoint])
        # print(S_Fk[startpoint])
        #Computing collision for last n joints : 
        NoOfJntsConsiders = 9
        NoOfJts = 15 - NoOfJntsConsiders
        
        Starts,ends,radis = GenerateCylinders_URCT(current_data)
       
       
        Needle_result = Cylinder_Cylinder_Collision(Starts[NoOfJts:],ends[NoOfJts:],radis[NoOfJts:],Entry_list,Target_list,Needle_radius,10,10,ETCList,EntryTargetStringList,statustype)
        print(Needle_result)
        
        startpoint = startpoint + 1
            
        if Needle_result == 1 :
            
            return 0,current_frame

    return 1,current_frame

def PullBackCollision(Entry_list,Target_list,points,Needle_radius,S_Fk):

      
    for i in range(len(points[0])):
        # print(len(points))
        # current_frame = [points[:i]]
        # print(np.array(S_Fk).shape)
        current_data = S_Fk[i]
        # print(points[i])
        # print(S_Fk[i])
        #Computing collision for last n joints : 
        NoOfJntsConsiders = 9
        NoOfJts = 15 - NoOfJntsConsiders
        
        Starts,ends,radis = GenerateCylinders_URCT(current_data)
       
       
        Needle_result = CC_Collision(Starts[NoOfJts:],ends[NoOfJts:],radis[NoOfJts:],Entry_list,Target_list,Needle_radius,10,10)
        print(Needle_result)
        
        # startpoint = startpoint + 1
            
        if Needle_result == 1 :
            
            return 1

    return 0     
# def PlotFunction_CylinderCylinder(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder_Robot_Point_Bottom,Cylinder_Robot_Point_Top,Radii_Robot,NumLines,NumPoints):
    
#     Plot_Figure = go.Figure()
    
#     #Needles-Robot
   
#     colorscale= 'viridis'
#     for i in range(len(Cylinder1_Point_Bottom)):   
#         Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
#         Cylinder1_Point2 =Cylinder1_Point_Top[i]
#         Radius_1 = Radii_1[i]    
#         cyl_tube_1,cyl_bottom_1,cyl_top_1= Visualizing_Cylinder(Cylinder1_Point1, Cylinder1_Point2, Radius_1,colorscale)   
            
#         Plot_Figure.add_trace(cyl_tube_1)
#         Plot_Figure.add_trace(cyl_bottom_1)
#         Plot_Figure.add_trace(cyl_top_1)  

#     colorscale= 'reds'
#     for i in range(len(Cylinder_Robot_Point_Bottom)):
#         Cylinder_Point1 = Cylinder_Robot_Point_Bottom[i]
#         Cylinder_Point2 = Cylinder_Robot_Point_Top[i]
#         r = Radii_Robot[i]
#         cyl_tube_2,cyl_bottom_2,cyl_top_2= Visualizing_Cylinder(Cylinder_Point1,Cylinder_Point2,r,colorscale)
#         LinesPoint=LinesCylinderPlot(Cylinder_Point1,Cylinder_Point2,r,NumLines,NumPoints)
#         Plot_Figure.add_trace(LinesPoint)
#         Plot_Figure.add_trace(cyl_tube_2)
#         Plot_Figure.add_trace(cyl_bottom_2)
#         Plot_Figure.add_trace(cyl_top_2)      
        
#     ShowCurrentPlot(Plot_Figure)    

# def EntryTarget_Collision(TargetPoint,EntryPoint,CollisionPoints):
#     Radius = 8
#     distances=[]
#     statuses=[]
#     flag =0
#     for j in range(len(CollisionPoints)):
            
#         status,distance, Point = points_in_cylinder(TargetPoint,EntryPoint,Radius,CollisionPoints[j])        
            
#         statuses.append(status)
#         distances.append(distance)
#             #print(status,distance)
       
#     if((-1 in statuses) or (0 in statuses)):
#         return 1 # intersection
        
            
#     elif(1 in statuses):
#         # no intersection
#         return 0    
  


        