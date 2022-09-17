##run with file with existing entry, target, entry target radiud, entrylist, targetlist, entrytargetlistradius

import urx
import math
import numpy as np
import time
np.set_printoptions(suppress=True)
from decimal import *

# From other sources 
from modules import *
from PlateRegistration import *
from PlotsFunction import *
from FunctMain import *

Origin = [-0.42078522626792736,
          -0.14122482407052936,
          0.15250089479459986,
          ]
#Robot Connect: 


def CDR(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,robo,sta_inp,movement,intpullback):
    
   


    print(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius)
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

    

    h =int(input("Do you want to hit home? Press 1 to continue :"))
    if(h == 1):
        count = 1
        while(count == 1):
            if robo.is_program_running() == False:
                count = 0
                time.sleep(0.2)                                
            time.sleep(1.0)
        robo.movej((0.0532689094543457, -1.7553278408446253, 2.0179107824908655, 4.0472752290913085, -1.5544269720660608, 0.007856130599975586),0.8,0.8,wait=False)
    Plot_Figure = RobotBaseStructure()    
    Plot_Figure =PlotFunction_Cylinders(np.array(Entry),np.array(Target),np.array(EntryTargetRadius),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),Plot_Figure,'viridis')
    Entry = Entry[0]
    Target = Target[0]
    vec = np.array(Entry) - np.array(Target)
    vec = vec/np.linalg.norm(vec)
    vec = vec *100
    New_Entry = vec + Entry

    print(Entry)
    print(New_Entry)
    status2,statuspullback ,FInalJ,Final_Orientation,Plot_Figure = FunctMain(np.array(New_Entry),np.array(Target),np.array(EntryTargetRadius[0]),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),robo,ETCList,NeedleList,Entry,Plot_Figure,statustype = "restAPI")
    if(status2 ==0) or (sta_inp == 1): #if there is no R-N collision or you want to overwrite collision

        if(statuspullback == 0):
            print("No colliison during pullback !")
            if(movement == 1):
                print("Making the movement!")
                # Trajectory 1
                
                status = ActualMovement(FInalJ[0:17],0.15,0.15,0.02,robo)
                time.sleep(0.5)
                # TrajectoryProfiling(FInalJ[0:17])
                # time.sleep(0.5)
                
                count_1 = 1
                while(count_1 == 1):
                    if robo.is_program_running() == False:
                        count_1 = 0
                        time.sleep(0.2)
                # Trajectory 2 
                        
                v= np.array(np.array(Entry)-np.array(New_Entry))
                Position =[(v[0])/1000.0,v[1]/1000.0,v[2]/1000.0,0,0,0]  
                time.sleep(1.5)                      
                robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                print("...")

                count_2 = 1
                while(count_2 == 1):
                    if robo.is_program_running() == False:
                        count_2 = 0
                        time.sleep(0.2)
                print("Robot Positioned") 
                time.sleep(0.5)
                
                    
                
                intpullback= int(input("Do you wnat to pullback ? Press 1 to continue")) 
                if(intpullback == 1):      
                    SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                                        -1.970654150048727,
                                        1.1324918905841272,
                                        4.770189034729757,
                                        -1.6497204939471644,
                                        0.0055446624755859375] 
                    vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry)
                    vector2 = vector2 /Norm(vector2)
                    vector2 = vector2*45   
                    Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]                        
                    robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                    time.sleep(1.5)
                    
                    count_3 = 1
                    while(count_3 == 1):
                        if robo.is_program_running() == False:
                            count_3 = 0
                            time.sleep(0.2)

                    # Move 100 mm upwards wrt robot base
                    robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
                    time.sleep(1.5)

                    count_1 = 1
                    while(count_1 == 1):
                        if robo.is_program_running() == False:
                            count_1 = 0
                            time.sleep(0.2)                                
                    time.sleep(1.0)
                    robo.movej((SafeHomePosition),0.2,0.2,wait=False)
                    time.sleep(1.0)

                    Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                    print("Showing Final graph")         
                    ShowCurrentPlot(Plot_Figure)
                    
               
                return 1, FInalJ[0:17]
        else :
            print("Collision During PullBack!. ")
            
            if(intpullback == 1):
                print("Overwriting Pullback Collision !")
                if(movement == 1):
                    print("Making the movement!")
                    # Trajectory 1
                    
                    status = ActualMovement(FInalJ[0:17],0.15,0.15,0.02,robo)
                    time.sleep(0.5)
                    # TrajectoryProfiling(FInalJ[0:17])
                    # time.sleep(0.5)
                    
                    count_1 = 1
                    while(count_1 == 1):
                        if robo.is_program_running() == False:
                            count_1 = 0
                            time.sleep(0.2)
                    # Trajectory 2 
                            
                    v= np.array(np.array(Entry)-np.array(New_Entry))
                    Position =[(v[0])/1000.0,v[1]/1000.0,v[2]/1000.0,0,0,0]  
                    time.sleep(1.5)                      
                    robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                    print("...")
                    

                    count_2 = 1
                    while(count_2 == 1):
                        if robo.is_program_running() == False:
                            count_2 = 0
                            time.sleep(0.2)
                    print("Robot Positioned") 
                   
                    
                    intpullback= int(input("Do you wnat to pullback ? Press 1 to continue")) 
                    if(intpullback == 1):  
                        SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                                            -1.970654150048727,
                                            1.1324918905841272,
                                            4.770189034729757,
                                            -1.6497204939471644,
                                            0.0055446624755859375] 
                        vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry)
                        vector2 = vector2 /Norm(vector2)
                        vector2 = vector2*45          
                        Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]                        
                        robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                        time.sleep(1.5)
                        
                        count_2 = 1
                        while(count_2 == 1):
                            if robo.is_program_running() == False:
                                count_2 = 0
                                time.sleep(0.2)

                        # Move 100 mm upwards wrt robot base
                        robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
                        time.sleep(1.5)

                        count_1 = 1
                        while(count_1 == 1):
                            if robo.is_program_running() == False:
                                count_1 = 0
                                time.sleep(0.2)                                
                        time.sleep(1.0)
                        robo.movej((SafeHomePosition),0.2,0.2,wait=False)
                        time.sleep(1.0)

                        Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                        print("Showing Final graph")         
                        ShowCurrentPlot(Plot_Figure)
                        
                   
                    return 1, FInalJ[0:17]
            else:
                print("Pullback Collision !")
                Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                print("Showing Final graph")         
                ShowCurrentPlot(Plot_Figure)
                return 0 ,[]
    else: 

        print("Robot - needle collision!. Try planning away from needle")
        Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
        print("Showing Final graph")         
        ShowCurrentPlot(Plot_Figure)
        return 0, []

    





def DPB(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,robo,intpullback):

    
    print(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius)
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
    vec = np.array(Entry) - np.array(Target)
    vec = vec/np.linalg.norm(vec)
    vec = vec *100
    New_Entry = vec + Entry
    Plot_Figure = RobotBaseStructure()    
    status2,statuspullback ,FInalJ,Final_Orientation = FunctMain(np.array(New_Entry[0]),np.array(Target[0]),np.array(EntryTargetRadius[0]),np.array(EntryList),np.array(TargetList),np.array(EntryTargetListRadius),robo,ETCList,NeedleList,Entry[0],Plot_Figure,statustype = "api")
    if(statuspullback == 0):
        print("No collision During Pullback!")
        vec = np.array(Entry) - np.array(Target)
        vec = vec/np.linalg.norm(vec)
        vec = vec *100
        New_Entry = vec + Entry
        SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                            -1.970654150048727,
                            1.1324918905841272,
                            4.770189034729757,
                            -1.6497204939471644,
                            0.0055446624755859375] 
        vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry[0])
        vector2 = vector2 /Norm(vector2)
        vector2 = vector2*45          
        Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]    

                       
        robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
        time.sleep(1.5)
        
        count_2 = 1
        while(count_2 == 1):
            if robo.is_program_running() == False:
                count_2 = 0
                time.sleep(0.2)

        # Move 100 mm upwards wrt robot base
        robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
        time.sleep(1.5)

        count_1 = 1
        while(count_1 == 1):
            if robo.is_program_running() == False:
                count_1 = 0
                time.sleep(0.2)                                
        time.sleep(1.0)
        robo.movej((SafeHomePosition),0.2,0.2,wait=False)
        time.sleep(1.0)
        return 1, FInalJ[0:17]
    else:        
        if(intpullback == 1):
            print("Overwriting Pullback Collision !")
            SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                            -1.970654150048727,
                            1.1324918905841272,
                            4.770189034729757,
                            -1.6497204939471644,
                            0.0055446624755859375] 
            vector2 = -np.array(Final_Orientation[:,2]) - np.array(Entry[0])
            vector2 = vector2 /Norm(vector2)
            vector2 = vector2*45          
            Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]  
            robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
            time.sleep(1.5)
            
            count_2 = 1
            while(count_2 == 1):
                if robo.is_program_running() == False:
                    count_2 = 0
                    time.sleep(0.2)

            # Move 100 mm upwards wrt robot base
            robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
            time.sleep(1.5)

            count_1 = 1
            while(count_1 == 1):
                if robo.is_program_running() == False:
                    count_1 = 0
                    time.sleep(0.2)                                
            time.sleep(1.0)
            robo.movej((SafeHomePosition),0.2,0.2,wait=False)
            time.sleep(1.0)
            return 1, FInalJ[0:17]
        else:
            print("Collision encountered during Pullback !")
            return 0,[]
    # Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
    # print("Showing Final graph")         
    # ShowCurrentPlot(Plot_Figure)
    

if __name__ == '__main__':
    # #Sample Input 1  
    Entry =np.array([[-573.940857713845,152.431152368458,-31.4594994725036]])
    Target =np.array([[-576.842720189387,150.138132531451,-181.413896721134]])
    EntryTargetRadius =np.array([[1.25]])
    EntryList = np.array([[-594.7513572872767, -512.0279696392333, 72.1426515293526], [-604.1222398248747, -512.4804195113744, 109.99739825390145], 
                        ])
    TargetList = np.array([[-553.9039205849268, -510.0557522478493, -92.86521880842443], [-595.9527524844046, -512.0859760330976, 76.99582418634606], 
     ])
    EntryTargetListRadius = np.array([[0.62],
       [7.55],
       ])
    ## Sample Input 2
    # Entry =np.array([[-577.184824, 187.194083, -33.035845]])
    # Target =np.array([[-577.498148, 157.718062, -180.110880]])
    # EntryTargetRadius =np.array([[1.25]])
    # EntryList = np.array([[-573.991323, 138.981074, -29.858527],[-667.761862, 143.404058, -38.741193],[-625.473133, 92.502134, -35.265179],[-633.621938, 146.953915, -33.540158],[-573.940857713845,152.431152368458,-31.4594994725036]])
    # TargetList = np.array([[-575.188845, 136.843334, -179.838512],[-605.667524, 147.228028, -175.231744],[-595.111525, 128.021959, -177.801191],[-596.587293, 148.369837, -178.889496],[-576.842720189387,150.138132531451,-181.413896721134]])
    # EntryTargetListRadius = np.array([[1.25],[1.25],[1.25],[1.25],[1.25]])

    
    # Entry =np.array([[-419.58794468, -605.25852063,   21.3391293 ]])
    # Target =np.array([[-368.15498081352365, -642.6473978769642, -132.4627758464786]])
    # EntryTargetRadius =np.array([[1.25]])   
    # EntryList =np.array([[-420.06719764766945, -633.7850276071496, 23.279247438179993], [-432.38689897690637, -634.0552980501019, 60.28130026023385], [-288.97119769638357, -582.1040033108357, 24.383364616059737], [-277.18478998881034, -582.0657127975851, 61.559689436938044]])
    # TargetList = np. array([[-368.2236947939167, -632.6476813269088, -132.4319974864308], [-421.6466465360332, -633.8196776639383, 28.023100364084343], [-338.5705057715669, -582.2651366272961, -132.06124799555357], [-287.4601197851563, -582.0990942706753, 29.149560105915924]])
    # EntryTargetListRadius =np.array([[0.62],[2.0],[3.0],[4.0]])

    time.sleep(2)
    robo = urx.Robot("172.16.101.224")
    while(1):

        try:			
            print("1-Check Device Reachability,2-Device Pull Back,Any other key - Exit") 
            val = int(input("Enter your value: "))								
            if(val== 1):
                print("Check Device Reachability!!")					
                sta_inp=input("Do you want to overwrite when encountered with collision? press 1 to overwrite :")
                movement = input("Do you want to execute the movement ? press 1 to continue :")
                intpullback = int(input("Do you want to overwrite pullback and collide ? Press 1 to continue :"))
                status,finalJ =CDR(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,robo,sta_inp,movement,intpullback)
                if(status == 1):
                    print("Success!")
                    print(finalJ)
                elif(status ==0):
                    print("Fail!")
            								
            elif(val == 2):
                print('Device Pull Back !!')
                intpullback = int(input("Do you want to overwrite pullback and collide ? Press 1 to continue :"))
                status,finalJ =DPB(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,robo,intpullback)
                if(status == 1):
                    print("Success!")
                    print(finalJ)
                elif(status ==0):
                    print("Fail!")
        
            			    
        except KeyboardInterrupt:	
            exit()

        