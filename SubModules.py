#Import Packages: 
import math
import math3d as m3d
import numpy as np
import time
from decimal import *
from modules import *
from PlotsFunction import *
from PathPlanningModule import *
from ActualRobotMovement import *
from FunctMain import *

def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def GoToPosition(robo,x,y,z):
    print("incoming")
    count = 1
    status = 1

    x = float(Decimal(x) / Decimal(1000))
    y = float(Decimal(y) / Decimal(1000))
    z = float(Decimal(z) / Decimal(1000))
    cpv = [-0.42078522626792736,
            -0.14122482407052936,
            0.15250089479459986,
            -1.9271780579918678,
            -2.1351417044602883,
            0.39154973830739315]

    cpv[0] = cpv[0] + x
    cpv[1] = cpv[1] + y
    cpv[2] = cpv[2] + z
    
    print(cpv)
    robo.movel((cpv),0.07,0.07 ,wait=False)
    time.sleep(1.5)
    while count == 1:
        if robo.is_program_running() == False:
            count = 0
            break
    return status

def CheckDeviceMovementStatus(Origin,Entry,Target,EntryTargetRadius,EntryList,TargetList,Needle_radius,robo):
    
    Final_Jvalues = 0
    # #initialize Simulated robot
    Plot_Figure = RobotBaseStructure()

    # Data point convertion and Initial / Final Orientation:
    # Left docking : 
    Initial_point = [-320.38002235, -277.23222217,  712.70369695]

    Initial_Orientation = np.array([[ 0.38780087,  0.01655112, -0.92159457],
                                    [-0.92031888, -0.04860872, -0.38813704],
                                    [-0.05122163,  0.99868076, -0.00361819]])

    # EM = [float(Decimal(Entry[0]) / Decimal(1000)),float(Decimal(Entry[1]) / Decimal(1000)),float(Decimal(Entry[2]) / Decimal(1000))] 
    # TM = [float(Decimal(Target[0]) / Decimal(1000)),float(Decimal(Target[1]) / Decimal(1000)),float(Decimal(Target[2]) / Decimal(1000))] 

    # #Adding Origin offset
    # EM = np.asarray(Origin) + np.asarray(EM)
    # TM = np.asarray(Origin) + np.asarray(TM) 


    # Plot_Entry = np.array(EM) * 1000
    # Plot_Target = np.array(TM) * 1000
    
    #Plot Current Entry and Target
    # Plot_Figure,ETpoints = TraceEntTarpoints(Plot_Figure,Plot_Entry,Plot_Target) 
    
    # print("Modified Entry : {}, Modified Target : {}".format(EM,TM))
    #Path Retrival  
    ICD = 30 #Initiail centre distance
    Final_point = np.asarray(Entry) * 1000
    WST = 300 #Workspace threshold
    
    #Planner : 
    print("Started the thread :  Path planning")
    PathCoordinates, Plot_Figure, pathpoints,PATHSTACKS,ICD_revisied = PathPlanner(Initial_point, ICD, Final_point, EntryList, TargetList, WST, Needle_radius,Plot_Figure)

    if PathCoordinates != 0:
        
        print("Started the thread : Collision Avoidance")

        
        #Pose optim :
        Final_Orientation = ModulePoseOptimization(Entry,Target)
        
        Orient = RetriveTimeStepOrientation(Initial_Orientation,Final_Orientation,17)

        #Iterate path for collision check and singularity
        # ShowCurrentPlot(Plot_Figure)
        
        if len(PATHSTACKS) == 0:
            Pathcoordinates_iter = PathCoordinates
            X_Cods,Y_Cods,Z_Cods,Final_Jvalues,Singularity_stat,CollidingFrame = ModuleCollision(Pathcoordinates_iter,Orient,EntryList,TargetList,Initial_point,Entry,Target,Needle_radius)
            print("Ideally this error should'not come!")
            return [0,Final_Jvalues]
        
        else:

            for p_ in range(len(PATHSTACKS)):
                Pathcoordinates_iter = PATHSTACKS[p_]
                X_Cods,Y_Cods,Z_Cods,Final_Jvalues,S_Status,CollidingFrame = ModuleCollision(Pathcoordinates_iter,Orient,EntryList,TargetList,Initial_point,Entry,Target,Needle_radius)
                
                #MovementFrame
                
                # Plot_Figure = plot_frames(Plot_Figure,X_Cods,Y_Cods,Z_Cods,Final_Jvalues)
                
                # print("Showing Final graph")         
                # ShowCurrentPlot(Plot_Figure)
                
                return[1,Final_Jvalues]



    else:
        print("Path collision : Change the entry angle")
        

    return[0,Final_Jvalues]


def VRHomePosition(robo):
    count = 1
    status = 1
    val = [-0.020525280629293263,
            -1.9967347584166468,
            2.3735931555377405,
            3.978508635158203,
            -1.5262778441058558,
            -0.11610013643373662]

    robo.movej((val),0.3,0.3,wait=False)
    print('Robot moving to Home position')
    time.sleep(2)
    while count == 1:
        if robo.is_program_running() == False:
            count = 0
            break
    return status


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


def GetCurrentPositionEntryToTarget(robo):

    cpv = robo.getl()
    orient = robo.get_orientation()
    O_matrix = orient.array

    ChangeIn_orientation = np.array([[ 0.00128335, -0.0002887 , -0.99999913],
                                [ 0.99992514,  0.01216854,  0.00127974],
                                [ 0.01216816, -0.99992592,  0.0003043 ]])


    Home_orientation = np.array([[ 0.00125812,  0.00059466, -0.99999903],
                                [ 0.99999773, -0.0017219 ,  0.00125709],
                               [-0.00172115, -0.99999834, -0.00059683]])
    
    Home_inv = np.linalg.inv(Home_orientation)
    offset = np.dot(Home_inv,ChangeIn_orientation)
    #Need offset inverse for fetching point
    final_orient = np.dot(O_matrix,np.linalg.inv(offset))
    

    entry = np.asarray(cpv[0:3])*1000
    target = entry + (final_orient[:,1] *142.75)
    print("Acquired Position : {} , Modified Position : {}".format(entry,target))
    return target,cpv


def CDR(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,movement):
    
    print("Input to CDR!")
    print(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius)

    #creating a list for current needles and placed needles in order to print 
    total_length = len(EntryList)    
    NeedleList =[]
    ETCList =[]
    print(total_length)
    for i in range(total_length):
        
        val =str(i+1)
        NeedleList.append(val)
        dm  =[]
        dm.append(TargetList[i])
        dm.append(EntryList[i]) 
        ETCList.append(dm)
    
    current_needle =str(total_length+1)
    NeedleList.append(current_needle)  

    #Calling necessary plot functions
    Plot_Figure = RobotBaseStructure()    
    # Plot_Figure =PlotFunction_Cylinders(np.array(Entry),np.array(Target),np.array(EntryTargetRadius),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),Plot_Figure,'viridis')
    
    #Finding "New_Entry" that is 100 mm above actual "Entry"
    Entry = Entry[0]
    Target = Target[0]
    vec = np.array(Entry) - np.array(Target)
    vec = vec/np.linalg.norm(vec)
    vec = vec *100
    New_Entry = vec + Entry

    print(Entry)
    print(New_Entry)
    status2,statuspullback ,FInalJ,PullBackJ,Final_Orientation,Plot_Figure,pose_flag = FunctMain(np.array(New_Entry),np.array(Target),np.array(EntryTargetRadius[0]),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),ETCList,NeedleList,Entry,Plot_Figure,statustype = "restAPI")
    if(status2 ==0): #if there is no R-N collision 
        if(statuspullback == 0): #if there is no collision during PullBack
            print("No colliison during pullback !") 
            if(pose_flag==0):
                print("Found Solution for Pose Optimization!")           
                if(movement == 1):
                    print("Executing the movement !")
                    
                    # # Trajectory 1
                    
                    # # status = ActualMovement(FInalJ[0:17],0.15,0.15,0.02,robo)
                    # # time.sleep(0.5)
                    # TrajectoryProfiling(FInalJ[0:17])
                    # # time.sleep(0.5)
                    
                    # count_1 = 1
                    # while(count_1 == 1):
                    #     if robo.is_program_running() == False:
                    #         count_1 = 0
                    #         time.sleep(0.2)

                    # # Trajectory 2                     
                    # v= np.array(np.array(Entry)-np.array(New_Entry))
                    # Position =[(v[0])/1000.0,v[1]/1000.0,v[2]/1000.0,0,0,0]  
                    # time.sleep(1.5)                      
                    # robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)

                    # count_2 = 1
                    # while(count_2 == 1):
                    #     if robo.is_program_running() == False:
                    #         count_2 = 0
                    #         time.sleep(0.2)
                    # print("Robot Positioned") 
                    # time.sleep(5.0)
                    
                    # #PullBack
                    # SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                    #                     -1.970654150048727,
                    #                     1.1324918905841272,
                    #                     4.770189034729757,
                    #                     -1.6497204939471644,
                    #                     0.0055446624755859375] 
                    # vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry)
                    # vector2 = vector2 /Norm(vector2)
                    # vector2 = vector2*45    

                    # # Move 45 mm in Z wrt tool       
                    # Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]                        
                    # robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                    # time.sleep(1.5)
                    
                    # count_3 = 1
                    # while(count_3 == 1):
                    #     if robo.is_program_running() == False:
                    #         count_3 = 0
                    #         time.sleep(0.2)

                    # # Move 100 mm upwards wrt robot base
                    # robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
                    # time.sleep(1.5)

                    # count_4 = 1
                    # while(count_4 == 1):
                    #     if robo.is_program_running() == False:
                    #         count_4 = 0
                    #         time.sleep(0.2)                                
                    # time.sleep(1.0)

                    # # Move to Home Position
                    # robo.movej((SafeHomePosition),0.2,0.2,wait=False)
                    # time.sleep(1.0)
                    # return 1,1, FInalJ[0:17],PullBackJ
                    # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                    # print("Showing Final graph")         
                    # ShowCurrentPlot(Plot_Figure)
                else:
                    print("Not Moving the Robot!")
                    return 1,1, FInalJ,PullBackJ
            elif(pose_flag == 1):
                print("Pose Optimization failed!")
                return 0 ,-2,[],[]
            
            
        else :
            print("Collision During PullBack!. ") 
            # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
            # print("Showing Final graph")         
            # ShowCurrentPlot(Plot_Figure)
            return 0 ,-1,[],[]
    else: 

        print("Robot - needle collision!. Try planning away from needle")
        # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
        # print("Showing Final graph")         
        # ShowCurrentPlot(Plot_Figure)
        return 0,0,[],[]

    





def DPB(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,robo):

    print("Input to CDR!")
    print(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius)

    #creating a list for current needles and placed needles in order to print 
    total_length = len(EntryList)
   
    NeedleList =[]
    ETCList =[]
    print(total_length)
    for i in range(total_length):
        
        val =str(i+1)
        NeedleList.append(val)
        dm  =[]
        dm.append(TargetList[i])
        dm.append(EntryList[i]) 
        ETCList.append(dm)
    
    current_needle =str(total_length+1)
    NeedleList.append(current_needle)

    vec = np.array(Entry[0]) - np.array(Target[0])
    vec = vec/np.linalg.norm(vec)
    vec = vec *100
    New_Entry = vec + Entry[0]

    # calling necessary plot functions
    # Plot_Figure = RobotBaseStructure()    
    status2,statuspullback ,FInalJ,PullBackJ,Final_Orientation,Plot_Figure = FunctMain(np.array(New_Entry),np.array(Target[0]),np.array(EntryTargetRadius[0]),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),robo,ETCList,NeedleList,Entry[0],Plot_Figure,statustype = "restAPI")
    if(statuspullback == 0): #if there is no collision during pullback
        print("No collision During Pullback!")
        
        SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                            -1.970654150048727,
                            1.1324918905841272,
                            4.770189034729757,
                            -1.6497204939471644,
                            0.0055446624755859375] 
        vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry[0])
        vector2 = vector2 /Norm(vector2)
        vector2 = vector2*45  

        # Move 45 mm in z wrt tool        
        Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0] 
        robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
        time.sleep(1.5)
        
        count_1 = 1
        while(count_1 == 1):
            if robo.is_program_running() == False:
                count_1 = 0
                time.sleep(0.2)

        # Move 100 mm upwards wrt robot base
        robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
        time.sleep(1.5)

        count_2 = 1
        while(count_2 == 1):
            if robo.is_program_running() == False:
                count_2 = 0
                time.sleep(0.2)                                
        time.sleep(1.0)
        # Move to Home Position
        robo.movej((SafeHomePosition),0.2,0.2,wait=False)
        time.sleep(1.0)

        # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
        # print("Showing Final graph")         
        # ShowCurrentPlot(Plot_Figure)
        return 1, FInalJ[0:17],PullBackJ
   
    else:
        print("Collision encountered during Pullback !")
        # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
        # print("Showing Final graph")         
        # ShowCurrentPlot(Plot_Figure)
        return 0,[],[]
        
            
   

