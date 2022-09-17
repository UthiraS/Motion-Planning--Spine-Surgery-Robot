########################################################################

# READ ME
# This code contains End to end workflow testing [python to C#]
# Modules addressed are :
#  1. Path Planning and recorrection,
#  2. Collision avoidance and recorrection.
#  3. Singulatiy

########################################################################

# Python Required Packages

import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *

# From other sources 
from modules import *
from PathPlanningModule import *
from PlotsFunction import *
from ActualRobotMovement import *
# from TrajectoryProfiling import *
# from CylinderCollision import Cylinder_Point_Collision

def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")


def FunctMain(Entry,Target,EntryTargetRadius,EntryList,TargetList,Needle_radius,ETCList,EntryTargetStringList,OldEntry,Plot_Figure,statustype): 

    # Data point convertion and Initial / Final Orientation:
    print("Entry :", Entry)
    print("OldEntry :", OldEntry)
    # Left docking : 
    Initial_point = [-320.38002235, -277.23222217,  712.70369695]

    Initial_Orientation = np.array([[ 0.38780087,  0.01655112, -0.92159457],
       [-0.92031888, -0.04860872, -0.38813704],
       [-0.05122163,  0.99868076, -0.00361819]])

    # Right docking
    # Initial_point = [-356.35485219,  121.17460263,  669.11007836]

    # Initial_Orientation = np.array([[-0.61926941, -0.02338683, -0.78483021],
    #    [-0.78094386, -0.085327  ,  0.61874551],
    #    [-0.0814377 ,  0.9960785 ,  0.03457661]])


    EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 
    
    #Path Retrival
    ICD = 50 #Initial centre distance
    Final_point = Entry
    WST = 300 #Workspace threshold

    
    #Planner : 
    print("Started the thread :  Path planning")
    PathCoordinates, Plot_Figure, pathpoints,PATHSTACKS,ICD_revisied = PathPlanner(Initial_point, ICD, Final_point, EntryList, TargetList, WST, Needle_radius,Plot_Figure)
    print("Path Plannning completed")

    if PathCoordinates != 0:

        print("Started the thread : Collision Avoidance")
        
        pose_flag =0
        #Pose optimization :
        Final_Orientation = ModulePoseOptimization(Entry,Target)
        if(Final_Orientation[0][0] == "False"):
            print("Pose Optimization failed! Plan a different trajectory!")
            pose_flag = 1
            return 1,1,[],[],[],Plot_Figure,pose_flag
            
        Orient = RetriveTimeStepOrientation(Initial_Orientation,Final_Orientation,17)
        Orient.append(Final_Orientation)
        Orient.append(Final_Orientation)
        Orient.append(Final_Orientation)

        #Iterate path for collision check and singularity
                
        if len(PATHSTACKS) == 0:
            Pathcoordinates_iter = PathCoordinates
            
            for i in range(3):
                Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], Entry[i])
                Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], (Entry[i]+OldEntry[i])/2)
                Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], OldEntry[i])
           
            A = np.array([Pathcoordinates_iter[0][17],Pathcoordinates_iter[1][17],Pathcoordinates_iter[2][17]])
            B = np.array([Pathcoordinates_iter[0][19],Pathcoordinates_iter[1][19],Pathcoordinates_iter[2][19]])
            C = np.array([Pathcoordinates_iter[0][18],Pathcoordinates_iter[1][18],Pathcoordinates_iter[2][18]])
            print(A)
            print(C)
            print(B)
            print("distance",Norm(B-A))
            print("po")
            print(np.array(Pathcoordinates_iter).shape)
            print(np.array(Orient).shape)
            X_Cods,Y_Cods,Z_Cods,Final_Jvalues,Singularity_stat,CollidingFrame,dummy2 = ModuleCollision(Pathcoordinates_iter,Orient,EntryList,TargetList,Initial_point,Entry,Target,Needle_radius,ETCList,EntryTargetStringList,statustype)
            X = X_Cods[17:]
            Y = Y_Cods[17:]
            Z = Z_Cods[17:]
            # print(X_Cods.shape)
            Plot_Figure,pathpoints1 = TracePath(Plot_Figure,X,Y,Z)

            print("Needle colliding alot. Try diff needle")
            return 1,1,[],[],Final_Orientation,Plot_Figure,pose_flag
        
        else:

            for p_ in range(len(PATHSTACKS)):
                Pathcoordinates_iter = PATHSTACKS[p_]     

                   
                for i in range(3):
                
                    Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], Entry[i])
                    Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], (Entry[i]+OldEntry[i])/2)
                    Pathcoordinates_iter[i] =np.append(Pathcoordinates_iter[i], OldEntry[i])
                A = np.array([Pathcoordinates_iter[0][17],Pathcoordinates_iter[1][17],Pathcoordinates_iter[2][17]])
                B = np.array([Pathcoordinates_iter[0][19],Pathcoordinates_iter[1][19],Pathcoordinates_iter[2][19]])
                C = np.array([Pathcoordinates_iter[0][18],Pathcoordinates_iter[1][18],Pathcoordinates_iter[2][18]])
                print(A)
                print(C)
                print(B)
                print("distance",Norm(B-A))
                print("po")
                print(np.array(Pathcoordinates_iter).shape)
                print(np.array(Orient).shape)
                   
                X_Cods,Y_Cods,Z_Cods,Final_Jvalues,S_Status,CollidingFrame,F_Orient = ModuleCollision(Pathcoordinates_iter,Orient,EntryList,TargetList,Initial_point,Entry,Target,Needle_radius,ETCList,EntryTargetStringList,statustype)
                X = X_Cods[17:]
                Y = Y_Cods[17:]
                Z = Z_Cods[17:]
                # print(np.array(X_Cods).shape)
                Plot_Figure,pathpoints1 = TracePath(Plot_Figure,X,Y,Z)
                print(np.array(Final_Jvalues).shape)

                #Checking Pullback collision
                current_orientation = np.array(F_Orient) #TCP Orientation at 20th point
                vector2 = -np.array(Final_Orientation[:,2]) - np.array(OldEntry)
                vector2 = vector2 /Norm(vector2)
                vector2 = vector2*65                            
                vector2_vis = vector2 +OldEntry                           

                #finding 3 intermittent points in the pull back path
                
                points1=Interpolation_line(OldEntry,vector2_vis,3)
                points1.append(vector2_vis)                          
                xc,yc,zc = SepPoints(points1)
                #path and orientation coordinates in pullback path
                points =np.array([xc,yc,zc])
                orientation_array=np.array([current_orientation,current_orientation,current_orientation,current_orientation])                           

                # print(points.shape)  
                # print(orientation_array.shape)                                                   
                #Retriving second solution and corresponding joint locations 
                soln = [2]  
                for ii in range(len(soln)): 
                    solution = soln[ii]
                    
                    X_Cods,Y_Cods,Z_Cods,J_values,S_Fk,final_orient= RetriveIKSolutions(points,orientation_array,solution)                                
                    # print(np.array(J_values).shape)
                    statuspullback =PullBackCollision(np.array(EntryList),np.array(TargetList),points,np.array(Needle_radius),S_Fk)
               
                #finding points in upward pullback
                vectorup = np.array([vector2_vis[0],vector2_vis[1],vector2_vis[2]+100])
                points2 = Interpolation_line(vector2_vis,vectorup,3)
                points2.append(vectorup) 
                xx,yy,zz = SepPoints(points2)
                #path and orientation coordinates in pullback path
                points_up =np.array([xx,yy,zz])
                orientation_up_array=np.array([current_orientation,current_orientation,current_orientation,current_orientation])   
                soln = [2]  
                for ii in range(len(soln)): 
                    solution = soln[ii]
                    
                    X_Cods_up,Y_Cods_up,Z_Cods_up,J_values_up,S_Fk_up,final_orient_up= RetriveIKSolutions(points_up,orientation_up_array,solution)
                    # print("upward pullback points")
                    # print(J_values_up[0])

                for i in range(len(J_values_up)):
                  J_values.append(J_values_up[i])  
                J_values.append([0.05329287052154541, -1.7553278408446253, 2.017887894307272, 4.0472752290913085, -1.5544632116900843, 0.007856130599975586])
                print(J_values)
                # print(np.array(J_values).shape)
                 
                stat_inp = 0
                
                Singularity_stat = 0 #dummysets
                
                if Singularity_stat == 0  or Singularity_stat == 1:
                    
                    if S_Status == 1:                                                                               

                        return 0,statuspullback, Final_Jvalues,J_values, Final_Orientation, Plot_Figure,pose_flag

                    else : 
                        # ShowCurrentPlot(Plot_Figure)
                        return 1,statuspullback, Final_Jvalues,J_values, Final_Orientation, Plot_Figure,pose_flag

    else : 

        print("Path collision : Change the entry angle")

        return 1,1,[],[],[],Plot_Figure,-1




                        