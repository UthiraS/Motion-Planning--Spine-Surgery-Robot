# Python Packages : 
# program to visualize robot as set of cylinders
import math
import numpy as np
import datetime
np.set_printoptions(suppress=True)
from flask import Flask
from flask import request
import json
import time
from decimal import *
import traceback

from PlotsFunction import *
from SubModules import *
from ActualRobotMovement import ActualMovement

#Flask variable
app = Flask(__name__)

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector
#Global Variables:
# global EntryList 
# EntryList = [[12345,3456,4566]]
# global TargetList 
# TargetList = [[12567,2322,1122]]

#Robot Connect: 
time.sleep(1)
# robo = urx.Robot("172.16.101.224")

#initialize Simulated robot
Plot_Figure = RobotBaseStructure()

#Global Variables:
FinalJointMovements = []
End_point = []

Origin = [-0.42078522626792736,
          -0.14122482407052936,
          0.15250089479459986,
          ]
Initial_point = np.asarray(Origin) * 1000

#Return Statement : 
def makeReturnStatement(value  ,status=0, error=''):
  state = False
  if status == 1:
    state = True
  ret = {
    'Error': error,
    'IsValidPlan': status,
    'JVs': value
  }
    
  return json.dumps(ret, indent=4, sort_keys=False)

def makeReturnStatement2(value  ,pullbackvalue,status=0, error=''):   
  
  state = False
  if status == 1:
    state = True

  ret = {
    'Error': error,
    'IsValidPlan': status,
    'PosJVs': value,
    'PullBackJVs':pullbackvalue
  }
    
  return json.dumps(ret, indent=4, sort_keys=False)

@app.route('/GoToPointRegistration',methods=["GET"])
def api_GoToPointRegistration():
    if "points" in request.args:
        try:
            inputStr = request.args["points"]
            retVal = json.loads(inputStr)
            entry = retVal["entry"]
            print(entry["x"], entry["y"], entry["z"])
            RegistrationStatus =  GoToPosition(robo,entry["x"], entry["y"], entry["z"])

            if RegistrationStatus == 1:
              return makeReturnStatement( value=1,status = 1,error='Success')     

        except Exception as ve:    
          return makeReturnStatement(value=1,status = 1, error='bad no of arguments with GET method')


@app.route('/CheckProbePositionerCollision',methods=["GET"])
def api_CheckProbePositionerCollision():
    if "probes" in request.args:
        try:
                        
            inputStr = request.args["probes"]
            retVal = json.loads(inputStr) 
            print(inputStr)
            print(retVal)

            #Current Probe
            current_value = retVal["CurrentProbe"]
            
            Curr_Needle_Entry =  current_value["EntryPoint"]
            Curr_Needle_Target = current_value["TargetPoint"]
            Curr_Needle_Radius = current_value["Needle_Diameter"]
            Curr_Holder_Radius = current_value["Handle_Diameter"]
            Curr_Holder_Length = current_value["Handle_Length"]
            print(Curr_Needle_Entry)
            print(Curr_Needle_Target)
            print(Curr_Needle_Radius)
            print(Curr_Holder_Radius)
            print(Curr_Holder_Length)
            # uncomment when not using rest API from browser

            # Curr_Needle_Entry = [float(Decimal(Curr_Needle_Entry[0]) / Decimal(1000)),float(Decimal(Curr_Needle_Entry[1]) / Decimal(1000)),float(Decimal(Curr_Needle_Entry[2]) / Decimal(1000))] 
            # Curr_Needle_Target = [float(Decimal(Curr_Needle_Target[0]) / Decimal(1000)),float(Decimal(Curr_Needle_Target[1]) / Decimal(1000)),float(Decimal(Curr_Needle_Target[2]) / Decimal(1000))] 

            # #Adding Origin offset
            # Curr_Needle_Entry = np.asarray(Origin) + np.asarray(Curr_Needle_Entry)
            # Curr_Needle_Target = np.asarray(Origin) + np.asarray(Curr_Needle_Target) 

            # Curr_Needle_Entry = np.array(Curr_Needle_Entry) * 1000
            # Curr_Needle_Target = np.array(Curr_Needle_Target) * 1000


           
            Entry =   [Curr_Needle_Entry]
            Target = [Curr_Needle_Target]            
            EntryTargetRadius =[[Curr_Needle_Radius]]
            print(Entry)
            print(Target)
            print(EntryTargetRadius)
            Vec = np.array(Entry[0]) -np.array(Target[0])
            print(Vec)
            Needle_Length =Norm(Vec) 
            Vec = Vec2UnitVec(Vec)
            Holder_Entry = Vec *(Needle_Length+5) +Target
            Holder_Target = Vec *(Needle_Length+5+Curr_Holder_Length) +Target
            Holder_Radius = Curr_Holder_Radius
            Entry.append(Holder_Entry.ravel())
            Target.append(Holder_Target.ravel())
            EntryTargetRadius.append([Holder_Radius])
            Entry = np.array(Entry)
            Target = np.array(Target)
            EntryTargetRadius = np.array(EntryTargetRadius)




            #Placed Probe
            placed_value = retVal["PlacedProbe"]

            Pla_Needle_Entry = placed_value["EntryPoint"]
            Pla_Needle_Target =placed_value["TargetPoint"]
            Pla_Needle_Radius =placed_value["Needle_Diameter"]
            Pla_Holder_Radius =placed_value["Handle_Diameter"]
            Pla_Holder_Length =placed_value["Handle_Length"]

            print(Pla_Needle_Entry)
            print(Pla_Needle_Target)
            print(Pla_Needle_Radius)
            print(Pla_Holder_Radius)
            print(Pla_Holder_Length)
            
            
            
            # uncomment when not using rest API from browser

            # Pla_Needle_Entry = [float(Decimal(Pla_Needle_Entry[0]) / Decimal(1000)),float(Decimal(Pla_Needle_Entry[1]) / Decimal(1000)),float(Decimal(Pla_Needle_Entry[2]) / Decimal(1000))] 
            # Pla_Needle_Target = [float(Decimal(Pla_Needle_Target[0]) / Decimal(1000)),float(Decimal(Pla_Needle_Target[1]) / Decimal(1000)),float(Decimal(Pla_Needle_Target[2]) / Decimal(1000))] 

            # #Adding Origin offset
            # Pla_Needle_Entry = np.asarray(Origin) + np.asarray(Pla_Needle_Entry)
            # Pla_Needle_Target = np.asarray(Origin) + np.asarray(Pla_Needle_Target) 

            # Pla_Needle_Entry = np.array(Pla_Needle_Entry) * 1000
            # Pla_Needle_Target = np.array(Pla_Needle_Target) * 1000
            

             
            EntryList =   [Pla_Needle_Entry]
            TargetList =  [Pla_Needle_Target] 
            EntryTargetListRadius = [[Pla_Needle_Radius]]
            Pla_Vec = np.array(EntryList[0]) -np.array(TargetList[0])
            Pla_Needle_Length =Norm(Pla_Vec) 
            Pla_Vec = Vec2UnitVec(Pla_Vec)
            Pla_Holder_Entry = Pla_Vec *(Pla_Needle_Length+5) +TargetList
            Pla_Holder_Target = Pla_Vec *(Pla_Needle_Length+5+Pla_Holder_Length) +TargetList
            Pla_Holder_Radius = Pla_Holder_Radius
            EntryList.append(Pla_Holder_Entry.ravel())
            TargetList.append(Pla_Holder_Target.ravel())
            EntryTargetListRadius.append([Pla_Holder_Radius])
            EntryList = np.array(EntryList)
            TargetList = np.array(TargetList)
            EntryTargetListRadius = np.array(EntryTargetListRadius)
            
            

            # Entry =np.array([[-419.58794468, -605.25852063,   21.3391293 ]])
            # Target =np.array([[-368.15498081352365, -642.6473978769642, -132.4627758464786]])
            # EntryTargetRadius =np.array([[1.25]])   
            # EntryList =np.array([[-420.06719764766945, -633.7850276071496, 23.279247438179993], [-432.38689897690637, -634.0552980501019, 60.28130026023385], [-288.97119769638357, -582.1040033108357, 24.383364616059737], [-277.18478998881034, -582.0657127975851, 61.559689436938044]])
            # TargetList = np. array([[-368.2236947939167, -632.6476813269088, -132.4319974864308], [-421.6466465360332, -633.8196776639383, 28.023100364084343], [-338.5705057715669, -582.2651366272961, -132.06124799555357], [-287.4601197851563, -582.0990942706753, 29.149560105915924]])
            # EntryTargetListRadius =np.array([[0.62]])


            #by default keeping executing no movement of the robot
            movement = 0
            print("input")
            print("Entry :",Entry, " Target : ",Target, " EnrtyTargetRadius :",EntryTargetRadius," EntryList :",EntryList," TargetList :",TargetList," EntryTargetListRadius : ",EntryTargetListRadius)
            status,value1,Final_Jvalues,PullBackJ = CDR(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,movement)
           
            if status == 1:
              
              print("Success")
              finalJ =[]
              for elem in Final_Jvalues:
                
                finalJ.append(tuple(list(elem)))
              print(finalJ)
              PBJ =[]
              for elem1 in PullBackJ:
                
                PBJ.append(tuple(list(elem1)))
              print(PBJ) 
              
              return makeReturnStatement2(value =finalJ,pullbackvalue =PBJ,status = 1,error = 'Robot Positioned!') 
            elif status ==0:
              if value1 == -1:
                print("PullBack Collision")
                return makeReturnStatement2(value =[],pullbackvalue =[],status = 0,error = 'Collision during Pullback!')
              elif value1 == 0:    
                print("Robot Needle Collision")            
                return makeReturnStatement2(value=[],pullbackvalue =[],status = 0,error="Robot - needle collision!. Try planning away from needle!")
              elif value1 == -2:    
                print("Pose Optimization Failed")            
                return makeReturnStatement2(value=[],pullbackvalue =[],status = 0,error="Pose Optimization Failed!. Try different plan!")
               
        except Exception as ve:    
          print("Exception")
          print(ve.message)
          traceback.print_exc()
          return makeReturnStatement2(value=[],pullbackvalue =[],status = 0,error=ve.message)
    else: 
      print("Bad Arguments")
      return makeReturnStatement2(value=[],pullbackvalue =[],status = 0, error='bad no of arguments with GET method')



@app.route('/CalculateSendValues',methods=["GET"])
def api_CalculateSendValues():
    if "probes" in request.args:
        try:
                        
            inputStr = request.args["probes"]
            retVal = json.loads(inputStr) 
            print("InputStr :",inputStr)
            print("RetVal : ",retVal)
            
            #Current Probe
            current_value = retVal["CurrentProbe"]
            print("Current value : ",current_value)
            Curr_Needle_Entry =  current_value["EntryPoint"]
            Curr_Needle_Target = current_value["TargetPoint"]
            Curr_Needle_Radius = current_value["Needle_Diameter"]
            Curr_Holder_Radius = current_value["Handle_Diameter"]
            Curr_Holder_Length = current_value["Handle_Length"]
            print(Curr_Needle_Entry)
            print(Curr_Needle_Target)
            print(Curr_Needle_Radius)
            print(Curr_Holder_Radius)
            print(Curr_Holder_Length)

            # Curr_Needle_Entry =  np.asarray(current_value[0])
            # Curr_Needle_Target = np.asarray(current_value[1])
            # Curr_Needle_Radius = np.asarray(current_value[2])
            # Curr_Holder_Radius = np.asarray(current_value[3])
            # Curr_Holder_Length = np.asarray(current_value[4])

            # uncomment when not using rest API from browser

            # Curr_Needle_Entry = [float(Decimal(Curr_Needle_Entry[0]) / Decimal(1000)),float(Decimal(Curr_Needle_Entry[1]) / Decimal(1000)),float(Decimal(Curr_Needle_Entry[2]) / Decimal(1000))] 
            # Curr_Needle_Target = [float(Decimal(Curr_Needle_Target[0]) / Decimal(1000)),float(Decimal(Curr_Needle_Target[1]) / Decimal(1000)),float(Decimal(Curr_Needle_Target[2]) / Decimal(1000))] 

            # #Adding Origin offset
            # Curr_Needle_Entry = np.asarray(Origin) + np.asarray(Curr_Needle_Entry)
            # Curr_Needle_Target = np.asarray(Origin) + np.asarray(Curr_Needle_Target) 

            # Curr_Needle_Entry = np.array(Curr_Needle_Entry) * 1000
            # Curr_Needle_Target = np.array(Curr_Needle_Target) * 1000


            # print(Curr_Needle_Entry,Curr_Needle_Target,Curr_Needle_Radius,Curr_Holder_Radius,Curr_Holder_Length)
            Entry =   [Curr_Needle_Entry]
            Target = [Curr_Needle_Target]            
            EntryTargetRadius =[[Curr_Needle_Radius]]
            print(Entry)
            print(Target)
            print(EntryTargetRadius)
            Vec = np.array(Entry[0]) -np.array(Target[0])
            print(Vec)
            Needle_Length =Norm(Vec) 
            Vec = Vec2UnitVec(Vec)
            Holder_Entry = Vec *(Needle_Length+5) +Target
            Holder_Target = Vec *(Needle_Length+5+Curr_Holder_Length) +Target
            Holder_Radius = Curr_Holder_Radius
            Entry.append(Holder_Entry.ravel())
            Target.append(Holder_Target.ravel())
            EntryTargetRadius.append([Holder_Radius])
            Entry = np.array(Entry)
            Target = np.array(Target)
            EntryTargetRadius = np.array(EntryTargetRadius)



            #putting dummy values for Placed Probe very far from workspace
            EntryList =np.array([[1234,3456,6789]])
            TargetList = np. array([[1234,3456,6890]])
            EntryTargetListRadius =np.array([[3.2]])

            movement =0
            print("input")
            print("Entry :",Entry, " Target : ",Target, " EnrtyTargetRadius :",EntryTargetRadius," EntryList :",EntryList," TargetList :",TargetList," EntryTargetListRadius : ",EntryTargetListRadius)
            status,value,Final_Jvalues,PullBackJ = CDR(Entry,Target,EntryTargetRadius,EntryList,TargetList,EntryTargetListRadius,movement)
            # FinalJList = list(Final_Jvalues)
            # final_json =json.dumps(FinalJList)
            # print(final_json)
            # print(Final_Jvalues.type)
             
            if status == 1:      
              print("Success")    
              print(Final_Jvalues)  
              finalJ =[]
              for elem in Final_Jvalues:
                print(tuple(list(elem)))
                finalJ.append(tuple(list(elem)))
              print(finalJ)  
              
              PBJ =[]
              for elem1 in PullBackJ:                
                PBJ.append(tuple(list(elem1)))
              print(PBJ) 
                          
              return makeReturnStatement2(value =finalJ,pullbackvalue = PBJ, status = 1,error = 'Sucess!') 
            elif status ==0:   
              print("Fail")           
              return makeReturnStatement2(value =[],pullbackvalue = [],status = 0,error = 'Fail!')
          

        except Exception as ve: 
            
          print("Exception") 
          print(ve.message)
          print(ve)
          traceback.print_exc()
          return makeReturnStatement2(value=[],pullbackvalue = [],status = 0,error=ve.message)
    else: 
      print("Bad Arguments")
      return makeReturnStatement2(value=[],pullbackvalue = [],status = 0, error='bad no of arguments with GET method') 

@app.route('/gotoHome',methods=["GET"])
def gotoHome():



  status = VRHomePosition(robo)

  return makeReturnStatement(value=1,status = 1,error = 'Success')

# @app.route('/GetDeviceStatus',methods=["GET"])
# def robot_status():

#   flag = 0
#   if robo.is_program_running() == True:
#     flag = 1
#     return makeReturnStatement(value=0,status = 0,error="Robot is running")
#   else:
#     return makeReturnStatement(value=1,status = 1,error="Success")


@app.route('/getcurrentlocation',methods=["GET"])
def GCP():

  pos,ang = GetCurrentPositionEntryToTarget(robo)
  op = {
      'tPos' :{
        'x' : pos[0],
        'y' : pos[1],
        'z' : pos[2],
      },
      'rPos' : {
        'x' : ang[3],
        'y' : ang[4],
        'z' : ang[5],
      }
    }
  return json.dumps(op, indent=4, sort_keys=True)



# Changes needed to be done by suhil or thiru 

# ********************************************************************************
# @app.route('/StartFreeDrive',methods=["GET"])
# def api_freeOn():
#   robo.set_freedrive(1)
#   return makeReturnStatement(value=1,status = 1,error='Success')

# @app.route('/StopFreeDrive',methods=["GET"])
# def api_freeOff():
#   robo.stop()
#   return makeReturnStatement(value=1,status = 1,error='Success')


# @app.route('/CancelAxisMovement',methods=["GET"])
# def api_CancelAxismovement():
#   robo.stop()
#   time.sleep(0.4)
#   robo.stop()
#   time.sleep(0.4)
#   robo.stop()
#   return makeReturnStatement(value=1,status = 1,error='Success')
# ********************************************************************************


@app.route('/DevicePullback',methods=["GET"])
def api_devicePullback():

  inputStr = request.args["points"]
  retVal = json.loads(inputStr) 

  #current Entry and Target
  entry_pos = retVal["EntryPoint"]
  target_pos = retVal["TargetPoint"]

  EntryMod = [float(Decimal(entry_pos[0]) / Decimal(1000)),float(Decimal(entry_pos[1]) / Decimal(1000)),float(Decimal(entry_pos[2]) / Decimal(1000))] 
  TargtMod = [float(Decimal(target_pos[0]) / Decimal(1000)),float(Decimal(target_pos[1]) / Decimal(1000)),float(Decimal(target_pos[2]) / Decimal(1000))]
  
  Entry = np.asarray(Origin) + np.asarray(EntryMod)
  Target = np.asarray(Origin) + np.asarray(TargtMod) 

  # typepullback = retVal["PullBackType"]

  # print('Entry : {} , Target : {}, PullBackType : {}'.format(EntryMod,TargtMod,typepullback))


  Needles_Entry = retVal["ConfirmedEntry"]
  Needles_Target = retVal["ConfirmedTarget"]
  Needles_Radius = retVal["RadiusList"]
  EntryList= []
  TargetList = []
  for iter1 in range(len(Needles_Entry)):
    C_entry = Needles_Entry[iter1]
    C_target = Needles_Target[iter1]
    
    EM = [float(Decimal(C_entry[0]) / Decimal(1000)),float(Decimal(C_entry[1]) / Decimal(1000)),float(Decimal(C_entry[2]) / Decimal(1000))] 
    TM = [float(Decimal(C_target[0]) / Decimal(1000)),float(Decimal(C_target[1]) / Decimal(1000)),float(Decimal(C_target[2]) / Decimal(1000))] 

    #Adding Origin offset
    EM = np.asarray(Origin) + np.asarray(EM)
    TM = np.asarray(Origin) + np.asarray(TM) 

    Stack_ent = np.array(EM) * 1000
    Stack_tar = np.array(TM) * 1000
    EntryList.append(Stack_ent)
    TargetList.append(Stack_tar)
  
  status,FinalJ =  DPB(np.array([(Entry).tolist()]),np.array([(Target).tolist()]),np.array([1.25]),np.array(EntryList),np.array(TargetList),np.array(Needles_Radius),robo)

  if(status == 1):
    return makeReturnStatement(value=FinalJ,status = 1,error='Success')
  elif(status ==0):
    return makeReturnStatement(value=0,status=0,error="PullBack Collision!")



#MultiNeedle Planning
@app.route('/positonForSubNeedle',methods=["GET"])
def api_MutliNeedlePlanning():

  inputStr = request.args["points"]
  retVal = json.loads(inputStr) 

  Needle_Entry = retVal["EntryPoint"]
  Needle_Target = retVal["TargetPoint"]
  Needle_length = retVal["NeedleLength"]

  Needle_Entry = np.asarray(Needle_Entry)
  Needle_Target = np.asarray(Needle_Target)

  if np.round(Needle_length) == 0:
    print("Zero needle length : {}".format(Needle_length))
    return makeReturnStatement(value=0,status = 0,error='Zero needle length')

  print("NeedleEntry : {}, NeedleTarget : {}, NeedleLength : {}".format(Needle_Entry,Needle_Target,Needle_length))
  stat = MultiNeedleSingularityCheck(Needle_Entry,Needle_Target,Needle_length,robo,Origin)
  if stat == 1:
    return makeReturnStatement(value=1,status = 1,error='Success')
  else : 
    return makeReturnStatement(value=0,status = 0,error='Failure')



if __name__ == '__main__':
  app.run(debug=True,host='172.16.101.172')
  





  
