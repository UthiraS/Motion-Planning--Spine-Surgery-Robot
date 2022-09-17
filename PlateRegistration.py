#!/usr/bin/env python
# coding: utf-8


import math
import numpy as np
import itertools
import re
from numpy.linalg import norm
from PlateRegistration import *
from FunctMain import *
from PlotsFunction import *
np.seterr(divide='ignore', invalid='ignore')


def Vec2UnitVec(vec):
    #Unit vector in the direction of vec
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def Norm(vec):   
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))    
    return norm


#Considering three points on top surface of base plate



#left #09/09/2021 demo
# Origin = np.array([-0.5878180141414003, -0.5217109512684367, -0.042474842397228])
# X = np.array([-0.48767146355216706, -0.5171215783737045, -0.04362048180039472])
# Y = np.array([-0.5925378418261752, -0.4218173485492944, -0.04241230433655698])

#left #13/09/2021 UR_CT
Origin = np.array([-0.5144807560234042, -0.6028558244822343, -0.08524842397228])
X = np.array([-0.4138833943302772, -0.594950107214501, -0.08733048180039472])
Y = np.array([-0.522332191496022, -0.5026007619967137, -0.08507230433655698])

# #Right
# Origin = np.array([-0.728738406540405, 0.1791135478153069, 0.14304980205986])
# X = np.array([-0.62923606825265687, 0.1730681435216147, 0.143130109219841])
# Y = np.array( [-0.7354491219599027,  0.2791585784332293,  0.14343724132669])

v1= np.array(X-Origin)
v2 =np.array(Y-Origin)
v1 =Vec2UnitVec(v1)
v2 =Vec2UnitVec(v2)
v3 = np.cross(v1,v2) # Assuming third axis is pointing upwards
v3 =Vec2UnitVec(v3)

Vec1 = np.array([[v1[0]],[v1[1]],[v1[2]],[0]])                
Vec2 = np.array([[v2[0]],[v2[1]],[v2[2]],[0]])

Vec3 = np.array([[v3[0]],[v3[1]],[v3[2]],[0]])
Origin1 = np.array([[Origin[0]],[Origin[1]],[Origin[2]],[1]])
COBmatrix=np.hstack((Vec1,Vec2,Vec3,Origin1))

# print(Vec1)
# print(Vec2)
# print(Vec3)
# print(Origin1)

# Change of basis matrix 


# print("Change of basis : ",COBmatrix)

#print("Modified change of basis :",M)
# Ploting three basis vectors defined at origin 0 of top plate
# fig = go.Figure()
# #Vec1
# S1 = go.Scatter3d(x = [Origin[0],v1[0]],
#                   y = [Origin[1],v1[1]],
#                   z = [Origin[2],v1[2]],                  
#                   marker=dict(
#                   size=1,
#                   color='blue',               
#                   colorscale='Viridis',  
#                   line=dict(width=1,color='DarkSlateGrey'),
#                   opacity=1))
# fig.add_trace(S1)

# #Vec2
# S2 = go.Scatter3d(x = [Origin[0],v2[0]],
#                   y = [Origin[1],v2[1]],
#                   z = [Origin[2],v2[2]],                  
#                   marker=dict(
#                   size=3,
#                   color='red',               
#                   colorscale='Viridis',  
#                   line=dict(width=1,color='DarkSlateGrey'),
#                   opacity=1))
# fig.add_trace(S2)
# #Vec3
# S2 = go.Scatter3d(x = [Origin[0],v3[0]],
#                   y = [Origin[1],v3[1]],
#                   z = [Origin[2],v3[2]],                  
#                   marker=dict(
#                   size=3,
#                   color='green',               
#                   colorscale='Viridis',  
#                   line=dict(width=1,color='DarkSlateGrey'),
#                   opacity=1))
# fig.add_trace(S2)
# #fig.show()



#Values entered in mm based on design in CAD Model, translated from origin
#Entry points
Ec1r1 = np.array([[0.009],[0.01006],[0],[1]]) 
Ec1r2 = np.array([[0.009],[0.02006],[0],[1]]) 
Ec1r3 = np.array([[0.009],[0.03006],[0],[1]])
Ec1r4 = np.array([[0.009],[0.04006],[0],[1]]) 
Ec1r5 = np.array([[0.009],[0.05006],[0],[1]]) 
Ec1r6 = np.array([[0.009],[0.06006],[0],[1]])
Ec1r7 = np.array([[0.009],[0.07006],[0],[1]]) 
Ec1r8 = np.array([[0.009],[0.08006],[0],[1]]) 
Ec1r9 = np.array([[0.009],[0.09006],[0],[1]])

Ec2r1 = np.array([[0.022],[0.01006],[0],[1]])
Ec2r2 = np.array([[0.022],[0.02006],[0],[1]]) 
Ec2r3 = np.array([[0.022],[0.03006],[0],[1]]) 
Ec2r4 = np.array([[0.022],[0.04006],[0],[1]]) 
Ec2r5 = np.array([[0.022],[0.05006],[0],[1]])
Ec2r6 = np.array([[0.022],[0.06006],[0],[1]]) 
Ec2r7 = np.array([[0.022],[0.07006],[0],[1]]) 
Ec2r8 = np.array([[0.022],[0.08006],[0],[1]]) 
Ec2r9 = np.array([[0.022],[0.09006],[0],[1]]) 

Ec3r1 = np.array([[0.035],[0.01006],[0],[1]])
Ec3r2 = np.array([[0.035],[0.02006],[0],[1]]) 
Ec3r3 = np.array([[0.035],[0.03006],[0],[1]])  
Ec3r4 = np.array([[0.035],[0.04006],[0],[1]])  
Ec3r5 = np.array([[0.035],[0.05006],[0],[1]])  
Ec3r6 = np.array([[0.035],[0.06006],[0],[1]])  
Ec3r7 = np.array([[0.035],[0.07006],[0],[1]]) 
Ec3r8 = np.array([[0.035],[0.08006],[0],[1]]) 
Ec3r9 = np.array([[0.035],[0.09006],[0],[1]])  

Ec4r1 = np.array([[0.065],[0.01006],[0],[1]])  
Ec4r2 = np.array([[0.065],[0.02006],[0],[1]])  
Ec4r3 = np.array([[0.065],[0.03006],[0],[1]])  
Ec4r4 = np.array([[0.065],[0.04006],[0],[1]])  
Ec4r5 = np.array([[0.065],[0.05006],[0],[1]])  
Ec4r6 = np.array([[0.065],[0.06006],[0],[1]]) 
Ec4r7 = np.array([[0.065],[0.07006],[0],[1]])  
Ec4r8 = np.array([[0.065],[0.08006],[0],[1]])  
Ec4r9 = np.array([[0.065],[0.09006],[0],[1]]) 


Ec5r1 = np.array([[0.078],[0.01006],[0],[1]]) 
Ec5r2 = np.array([[0.078],[0.02006],[0],[1]]) 
Ec5r3 = np.array([[0.078],[0.03006],[0],[1]])  
Ec5r4 = np.array([[0.078],[0.04006],[0],[1]])  
Ec5r5 = np.array([[0.078],[0.05006],[0],[1]]) 
Ec5r6 = np.array([[0.078],[0.06006],[0],[1]]) 
Ec5r7 = np.array([[0.078],[0.07006],[0],[1]])  
Ec5r8 = np.array([[0.078],[0.08006],[0],[1]]) 
Ec5r9 = np.array([[0.078],[0.09006],[0],[1]])  

Ec6r1 = np.array([[0.091],[0.01006],[0],[1]]) 
Ec6r2 = np.array([[0.091],[0.02006],[0],[1]]) 
Ec6r3 = np.array([[0.091],[0.03006],[0],[1]]) 
Ec6r4 = np.array([[0.091],[0.04006],[0],[1]]) 
Ec6r5 = np.array([[0.091],[0.05006],[0],[1]])  
Ec6r6 = np.array([[0.091],[0.06006],[0],[1]]) 
Ec6r7 = np.array([[0.091],[0.07006],[0],[1]])  
Ec6r8 = np.array([[0.091],[0.08006],[0],[1]])  
Ec6r9 = np.array([[0.091],[0.09006],[0],[1]]) 





#Values entered in mm based on design in CAD Model
#assumption length between top and bottom plate is exactly 80 mm
#Target points
Tc1r1 = np.array([[0.035],[0.01006],[-0.050],[1]]) 
Tc1r2 = np.array([[0.035],[0.02006],[-0.050],[1]])  
Tc1r3 = np.array([[0.035],[0.03006],[-0.050],[1]]) 
Tc1r4 = np.array([[0.035],[0.04006],[-0.050],[1]])  
Tc1r5 = np.array([[0.035],[0.05006],[-0.050],[1]]) 
Tc1r6 = np.array([[0.035],[0.06006],[-0.050],[1]])  
Tc1r7 = np.array([[0.035],[0.07006],[-0.050],[1]])  
Tc1r8 = np.array([[0.035],[0.08006],[-0.050],[1]])  
Tc1r9 = np.array([[0.035],[0.09006],[-0.050],[1]]) 

Tc2r1 = np.array([[0.065],[0.01006],[-0.050],[1]])  
Tc2r2 = np.array([[0.065],[0.02006],[-0.050],[1]])  
Tc2r3 = np.array([[0.065],[0.03006],[-0.050],[1]])  
Tc2r4 = np.array([[0.065],[0.04006],[-0.050],[1]]) 
Tc2r5 = np.array([[0.065],[0.05006],[-0.050],[1]]) 
Tc2r6 = np.array([[0.065],[0.06006],[-0.050],[1]])  
Tc2r7 = np.array([[0.065],[0.07006],[-0.050],[1]])  
Tc2r8 = np.array([[0.065],[0.08006],[-0.050],[1]])  
Tc2r9 = np.array([[0.065],[0.09006],[-0.050],[1]])  





# Finding the coordinates of Entry points in robot space
Ec1r1 = np.matmul(COBmatrix, Ec1r1) 
Ec1r2 = np.matmul(COBmatrix, Ec1r2)
Ec1r3 = np.matmul(COBmatrix, Ec1r3) 
Ec1r4 = np.matmul(COBmatrix, Ec1r4) 
Ec1r5 = np.matmul(COBmatrix, Ec1r5) 
Ec1r6 = np.matmul(COBmatrix, Ec1r6)
Ec1r7 = np.matmul(COBmatrix, Ec1r7)
Ec1r8 = np.matmul(COBmatrix, Ec1r8) 
Ec1r9 = np.matmul(COBmatrix, Ec1r9)


Ec2r1 = np.matmul(COBmatrix, Ec2r1) 
Ec2r2 = np.matmul(COBmatrix, Ec2r2) 
Ec2r3 = np.matmul(COBmatrix, Ec2r3) 
Ec2r4 = np.matmul(COBmatrix, Ec2r4) 
Ec2r5 = np.matmul(COBmatrix, Ec2r5) 
Ec2r6 = np.matmul(COBmatrix, Ec2r6) 
Ec2r7 = np.matmul(COBmatrix, Ec2r7) 
Ec2r8 = np.matmul(COBmatrix, Ec2r8) 
Ec2r9 = np.matmul(COBmatrix, Ec2r9) 


Ec3r1 = np.matmul(COBmatrix, Ec3r1) 
Ec3r2 = np.matmul(COBmatrix, Ec3r2) 
Ec3r3 = np.matmul(COBmatrix, Ec3r3) 
Ec3r4 = np.matmul(COBmatrix, Ec3r4) 
Ec3r5 = np.matmul(COBmatrix, Ec3r5) 
Ec3r6 = np.matmul(COBmatrix, Ec3r6)
Ec3r7 = np.matmul(COBmatrix, Ec3r7) 
Ec3r8 = np.matmul(COBmatrix, Ec3r8) 
Ec3r9 = np.matmul(COBmatrix, Ec3r9) 



Ec4r1 = np.matmul(COBmatrix, Ec4r1) 
Ec4r2 = np.matmul(COBmatrix, Ec4r2) 
Ec4r3 = np.matmul(COBmatrix, Ec4r3) 
Ec4r4 = np.matmul(COBmatrix, Ec4r4) 
Ec4r5 = np.matmul(COBmatrix, Ec4r5) 
Ec4r6 = np.matmul(COBmatrix, Ec4r6)
Ec4r7 = np.matmul(COBmatrix, Ec4r7) 
Ec4r8 = np.matmul(COBmatrix, Ec4r8) 
Ec4r9 = np.matmul(COBmatrix, Ec4r9) 



Ec5r1 = np.matmul(COBmatrix, Ec5r1) 
Ec5r2 = np.matmul(COBmatrix, Ec5r2) 
Ec5r3 = np.matmul(COBmatrix, Ec5r3) 
Ec5r4 = np.matmul(COBmatrix, Ec5r4) 
Ec5r5 = np.matmul(COBmatrix, Ec5r5) 
Ec5r6 = np.matmul(COBmatrix, Ec5r6)
Ec5r7 = np.matmul(COBmatrix, Ec5r7)
Ec5r8 = np.matmul(COBmatrix, Ec5r8) 
Ec5r9 = np.matmul(COBmatrix, Ec5r9) 


Ec6r1 = np.matmul(COBmatrix, Ec6r1) 
Ec6r2 = np.matmul(COBmatrix, Ec6r2) 
Ec6r3 = np.matmul(COBmatrix, Ec6r3) 
Ec6r4 = np.matmul(COBmatrix, Ec6r4) 
Ec6r5 = np.matmul(COBmatrix, Ec6r5) 
Ec6r6 = np.matmul(COBmatrix, Ec6r6) 
Ec6r7 = np.matmul(COBmatrix, Ec6r7) 
Ec6r8 = np.matmul(COBmatrix, Ec6r8) 
Ec6r9 = np.matmul(COBmatrix, Ec6r9) 




# Finding the coordinates of Target points in robot space
Tc1r1 = np.matmul(COBmatrix, Tc1r1) 
Tc1r2 = np.matmul(COBmatrix, Tc1r2)
Tc1r3 = np.matmul(COBmatrix, Tc1r3) 
Tc1r4 = np.matmul(COBmatrix, Tc1r4) 
Tc1r5 = np.matmul(COBmatrix, Tc1r5) 
Tc1r6 = np.matmul(COBmatrix, Tc1r6)
Tc1r7 = np.matmul(COBmatrix, Tc1r7)
Tc1r8 = np.matmul(COBmatrix, Tc1r8) 
Tc1r9 = np.matmul(COBmatrix, Tc1r9)


Tc2r1 = np.matmul(COBmatrix, Tc2r1) 
Tc2r2 = np.matmul(COBmatrix, Tc2r2) 
Tc2r3 = np.matmul(COBmatrix, Tc2r3) 
Tc2r4 = np.matmul(COBmatrix, Tc2r4) 
Tc2r5 = np.matmul(COBmatrix, Tc2r5) 
Tc2r6 = np.matmul(COBmatrix, Tc2r6) 
Tc2r7 = np.matmul(COBmatrix, Tc2r7) 
Tc2r8 = np.matmul(COBmatrix, Tc2r8) 
Tc2r9 = np.matmul(COBmatrix, Tc2r9) 

##

Ec1r1 = (Ec1r1[0:3].T.ravel()*1000).tolist()
Ec1r2 = (Ec1r2[0:3].T.ravel()*1000).tolist()
Ec1r3 = (Ec1r3[0:3].T.ravel()*1000).tolist()
Ec1r4 = (Ec1r4[0:3].T.ravel()*1000).tolist()
Ec1r5 = (Ec1r5[0:3].T.ravel()*1000).tolist()
Ec1r6 = (Ec1r6[0:3].T.ravel()*1000).tolist()
Ec1r7 = (Ec1r7[0:3].T.ravel()*1000).tolist()
Ec1r8 = (Ec1r8[0:3].T.ravel()*1000).tolist()
Ec1r9 = (Ec1r9[0:3].T.ravel()*1000).tolist()


Ec2r1 = (Ec2r1[0:3].T.ravel()*1000).tolist()
Ec2r2 = (Ec2r2[0:3].T.ravel()*1000).tolist()
Ec2r3 = (Ec2r3[0:3].T.ravel()*1000).tolist()
Ec2r4 = (Ec2r4[0:3].T.ravel()*1000).tolist()
Ec2r5 = (Ec2r5[0:3].T.ravel()*1000).tolist()
Ec2r6 = (Ec2r6[0:3].T.ravel()*1000).tolist()
Ec2r7 = (Ec2r7[0:3].T.ravel()*1000).tolist()
Ec2r8 = (Ec2r8[0:3].T.ravel()*1000).tolist()
Ec2r9 = (Ec2r9[0:3].T.ravel()*1000).tolist()


Ec3r1 = (Ec3r1[0:3].T.ravel()*1000).tolist()
Ec3r2 = (Ec3r2[0:3].T.ravel()*1000).tolist()
Ec3r3 = (Ec3r3[0:3].T.ravel()*1000).tolist()
Ec3r4 = (Ec3r4[0:3].T.ravel()*1000).tolist()
Ec3r5 = (Ec3r5[0:3].T.ravel()*1000).tolist()
Ec3r6 = (Ec3r6[0:3].T.ravel()*1000).tolist()
Ec3r7 = (Ec3r7[0:3].T.ravel()*1000).tolist()
Ec3r8 = (Ec3r8[0:3].T.ravel()*1000).tolist()
Ec3r9 = (Ec3r9[0:3].T.ravel()*1000).tolist()


Ec4r1 = (Ec4r1[0:3].T.ravel()*1000).tolist()
Ec4r2 = (Ec4r2[0:3].T.ravel()*1000).tolist()
Ec4r3 = (Ec4r3[0:3].T.ravel()*1000).tolist()
Ec4r4 = (Ec4r4[0:3].T.ravel()*1000).tolist()
Ec4r5 = (Ec4r5[0:3].T.ravel()*1000).tolist()
Ec4r6 = (Ec4r6[0:3].T.ravel()*1000).tolist()
Ec4r7 = (Ec4r7[0:3].T.ravel()*1000).tolist()
Ec4r8 = (Ec4r8[0:3].T.ravel()*1000).tolist()
Ec4r9 = (Ec4r9[0:3].T.ravel()*1000).tolist()

Ec5r1 = (Ec5r1[0:3].T.ravel()*1000).tolist()
Ec5r2 = (Ec5r2[0:3].T.ravel()*1000).tolist()
Ec5r3 = (Ec5r3[0:3].T.ravel()*1000).tolist()
Ec5r4 = (Ec5r4[0:3].T.ravel()*1000).tolist()
Ec5r5 = (Ec5r5[0:3].T.ravel()*1000).tolist()
Ec5r6 = (Ec5r6[0:3].T.ravel()*1000).tolist()
Ec5r7 = (Ec5r7[0:3].T.ravel()*1000).tolist()
Ec5r8 = (Ec5r8[0:3].T.ravel()*1000).tolist()
Ec5r9 = (Ec5r9[0:3].T.ravel()*1000).tolist()


Ec6r1 = (Ec6r1[0:3].T.ravel()*1000).tolist()
Ec6r2 = (Ec6r2[0:3].T.ravel()*1000).tolist()
Ec6r3 = (Ec6r3[0:3].T.ravel()*1000).tolist()
Ec6r4 = (Ec6r4[0:3].T.ravel()*1000).tolist()
Ec6r5 = (Ec6r5[0:3].T.ravel()*1000).tolist()
Ec6r6 = (Ec6r6[0:3].T.ravel()*1000).tolist()
Ec6r7 = (Ec6r7[0:3].T.ravel()*1000).tolist()
Ec6r8 = (Ec6r8[0:3].T.ravel()*1000).tolist()
Ec6r9 = (Ec6r9[0:3].T.ravel()*1000).tolist()


Tc1r1 = (Tc1r1[0:3].T.ravel()*1000).tolist()
Tc1r2 = (Tc1r2[0:3].T.ravel()*1000).tolist()
Tc1r3 = (Tc1r3[0:3].T.ravel()*1000).tolist()
Tc1r4 = (Tc1r4[0:3].T.ravel()*1000).tolist()
Tc1r5 = (Tc1r5[0:3].T.ravel()*1000).tolist()
Tc1r6 = (Tc1r6[0:3].T.ravel()*1000).tolist()
Tc1r7 = (Tc1r7[0:3].T.ravel()*1000).tolist()
Tc1r8 = (Tc1r8[0:3].T.ravel()*1000).tolist()
Tc1r9 = (Tc1r9[0:3].T.ravel()*1000).tolist()

Tc2r1 = (Tc2r1[0:3].T.ravel()*1000).tolist()
Tc2r2 = (Tc2r2[0:3].T.ravel()*1000).tolist()
Tc2r3 = (Tc2r3[0:3].T.ravel()*1000).tolist()
Tc2r4 = (Tc2r4[0:3].T.ravel()*1000).tolist()
Tc2r5 = (Tc2r5[0:3].T.ravel()*1000).tolist()
Tc2r6 = (Tc2r6[0:3].T.ravel()*1000).tolist()
Tc2r7 = (Tc2r7[0:3].T.ravel()*1000).tolist()
Tc2r8 = (Tc2r8[0:3].T.ravel()*1000).tolist()
Tc2r9 = (Tc2r9[0:3].T.ravel()*1000).tolist()



EntryTarget = { 'Ec1r1': Ec1r1, 'Ec1r2':Ec1r2,'Ec1r3':Ec1r3,'Ec1r4':Ec1r4,'Ec1r5':Ec1r5,'Ec1r6':Ec1r6,'Ec1r7':Ec1r7,'Ec1r8':Ec1r8,'Ec1r9':Ec1r9,
                'Ec2r1': Ec2r1, 'Ec2r2':Ec2r2,'Ec2r3':Ec2r3,'Ec2r4':Ec2r4,'Ec2r5':Ec2r5,'Ec2r6':Ec2r6,'Ec2r7':Ec2r7,'Ec2r8':Ec2r8,'Ec2r9':Ec2r9,
                'Ec3r1': Ec3r1, 'Ec3r2':Ec3r2,'Ec3r3':Ec3r3,'Ec3r4':Ec3r4,'Ec3r5':Ec3r5,'Ec3r6':Ec3r6,'Ec3r7':Ec3r7,'Ec3r8':Ec3r8,'Ec3r9':Ec3r9,
                'Ec4r1': Ec4r1, 'Ec4r2':Ec4r2,'Ec4r3':Ec4r3,'Ec4r4':Ec4r4,'Ec4r5':Ec4r5,'Ec4r6':Ec4r6,'Ec4r7':Ec4r7,'Ec4r8':Ec4r8,'Ec4r9':Ec4r9,
                'Ec5r1': Ec5r1, 'Ec5r2':Ec5r2,'Ec5r3':Ec5r3,'Ec5r4':Ec5r4,'Ec5r5':Ec5r5,'Ec5r6':Ec5r6,'Ec5r7':Ec5r7,'Ec5r8':Ec5r8,'Ec5r9':Ec5r9,
                'Ec6r1': Ec6r1, 'Ec6r2':Ec6r2,'Ec6r3':Ec6r3,'Ec6r4':Ec6r4,'Ec6r5':Ec6r5,'Ec6r6':Ec6r6,'Ec6r7':Ec6r7,'Ec6r8':Ec6r8,'Ec6r9':Ec6r9,
                'Tc1r1': Tc1r1, 'Tc1r2':Tc1r2,'Tc1r3':Tc1r3,'Tc1r4':Tc1r4,'Tc1r5':Tc1r5,'Tc1r6':Tc1r6,'Tc1r7':Tc1r7,'Tc1r8':Tc1r8,'Tc1r9':Tc1r9,
                'Tc2r1': Tc2r1, 'Tc2r2':Tc2r2,'Tc2r3':Tc2r3,'Tc2r4':Tc2r4,'Tc2r5':Tc2r5,'Tc2r6':Tc2r6,'Tc2r7':Tc2r7,'Tc2r8':Tc2r8,'Tc2r9':Tc2r9}

def ScaledEntryTarget(Entry, Target,ScalingFactor):  

    NeedleRadius =  [1.24/2]    
    Holder_Radius = [15.10/2]

    Entry = np.array(Entry)
    Target = np.array(Target)

    Distance = Norm(Entry-Target)
    Vector1 = Entry - Target
    Vector = Vec2UnitVec(Vector1)
    print(Distance+30+50)
    ModifiedEntry =(Vector *(Distance+30+50+ScalingFactor)) + Target
    ModifiedEntry1= (Vector *(Distance+30+50+ScalingFactor+5)) +Target
    ModifiedEntry2 = (Vector *(Distance+30+50+ScalingFactor+39)) +Target
    NeedleHolderPoint1 = ModifiedEntry1
    NeedleHolderPoint2 = ModifiedEntry2

    
    NewEntry =np.array((Vector *(Distance+30+50+ScalingFactor+100)) + Target)
    OldEntry = np.array((Vector *(Distance+30+50+ScalingFactor)) + Target)

    return(Target,ModifiedEntry,NeedleRadius,NeedleHolderPoint1,NeedleHolderPoint2,Holder_Radius,OldEntry,NewEntry)
   
# def ScaledEntryTarget(Entry, Target):  

#     NeedleRadius =  [1.24/2]    
#     Holder_Radius = [15.10/2]

#     Entry = np.array(Entry)
#     Target = np.array(Target)

#     Distance = Norm(Entry-Target)
#     Vector1 = Entry - Target
#     Vector = Vec2UnitVec(Vector1)
    
#     ModifiedEntry =(Vector *(170)) + Target
#     ModifiedEntry1= (Vector *(170+5)) +Target
#     ModifiedEntry2 = (Vector *(170+39)) +Target
#     NeedleHolderPoint1 = ModifiedEntry1
#     NeedleHolderPoint2 = ModifiedEntry2

    
#     NewEntryScaled =np.array([(Vector *(170+100)) + Target])
#     NewEntry = np.array([(Vector *(170)) + Target])

#     return(Target,ModifiedEntry,NeedleRadius,NeedleHolderPoint1,NeedleHolderPoint2,Holder_Radius,NewEntry,NewEntryScaled)


def EntryTargetComputation():    
    EntryList =[]
    TargetList = []
    Needle_radius =[]

   
    ETCList =[]
    EntryStringList =[]
    TargetStringList =[] 
    EntryTargetStringList =[]

    Counter =1    
    i = 1
    try:
        while True:
            h =int(input("Do you want to hit home? Press 1 to continue :"))
            if(h == 1):
                count = 1
                while(count == 1):
                    if robo.is_program_running() == False:
                        count = 0
                        time.sleep(0.2)                                
                    time.sleep(1.0)
                robo.movej((0.0532689094543457, -1.7553278408446253, 2.0179107824908655, 4.0472752290913085, -1.5544269720660608, 0.007856130599975586),0.4,0.4,wait=False)
                
            print("Iteration" +str(i)+" : ")
            CurrentEntryList =[]
            CurrentTargetList=[]
            CurrentRadii=[]
            print("***")
            print("Entry ROW AND COLUMN :")
            Ec = input("Entry column number (1-6):")
            Er = input("Entry row number (1-9):")
            EntryString = "Ec"+str(Ec)+"r"+str(Er)
            if((Ec>=1)and(Ec<=6)and(Er>=1)and(Er<=9)):                    #and (EntryString not in EntryStringList)
                print(EntryString,EntryTarget[EntryString])
                
            else:
                print("Wrong input !")
                continue


            print("Target ROW AND COLUMN :")
            Tc = input("Target column number (1-2):")
            Tr = input("Target row number (1-9):")
            TargetString = "Tc"+str(Tc)+"r"+str(Tr)
            if((Tc>=1)and(Tc<=2)and(Tr>=1)and(Tr<=9)):                 #and (TargetString not in TargetStringList)       
                print(TargetString,EntryTarget[TargetString])  
                TargetStringList.append(TargetString)
                EntryStringList.append(EntryString)
                dummy = EntryString + ' ' + TargetString
                EntryTargetStringList.append(dummy)
            else:
                print("Wrong input !")
                continue     

            ScalingFactor =  input("Enter the scaling factor (>=0)):")
            if(int(ScalingFactor) <0):
                print('Wrong input !')
                continue
                 
                
            Target,ModifiedEntry,NeedleRadius,NeedleHolderPoint1,NeedleHolderPoint2,Holder_Radius,Old_Entry,New_Entry=ScaledEntryTarget(EntryTarget[EntryString],EntryTarget[TargetString],ScalingFactor)        
                
            
            EntryTargetRadius =[]
            EntryTargetRadius.append([1.25/2])

            dm  =[]
            dm.append(Target)
            dm.append(ModifiedEntry)
            dm.append(NeedleHolderPoint1)
            dm.append(NeedleHolderPoint2)

            

            ETCList.append(dm)


            Target=Target.tolist()
            ModifiedEntry = ModifiedEntry.tolist()
            NeedleHolderPoint1 = NeedleHolderPoint1.tolist()
            NeedleHolderPoint2 = NeedleHolderPoint2.tolist()

            CurrentTargetList.append(Target)
            CurrentEntryList.append(ModifiedEntry)
            CurrentRadii.append(NeedleRadius)                
            CurrentTargetList.append(NeedleHolderPoint1)
            CurrentEntryList.append(NeedleHolderPoint2)
            CurrentRadii.append(Holder_Radius)

            if(i==1):
                
                EntryList.append([9856,2345,6789])
                TargetList.append([2345,3456,4567])
                Needle_radius.append([23])

            elif(i > 1):
                          

                print("....")
                print("Entry,Target,EntryTargetRadius :")
                EntryTargetRadius =[]
                EntryTargetRadius.append([1.25])
                print(New_Entry,EntryTarget[TargetString],EntryTargetRadius)
                print("EntryList, TargetList, EntryTargetList_radius : ")
                print(EntryList,TargetList,np.array(Needle_radius))
                
            
            # print(EntryTargetStringList[Counter-1])
            # status1,Plot_Figure = Cylinder_Cylinder_Collision_NN(np.array(CurrentEntryList),np.array(CurrentTargetList),np.array(CurrentRadii),np.array(EntryList),np.array(TargetList),np.array(Needle_radius),55,55,EntryTargetStringList[Counter-1],EntryTargetStringList,ETCList,statustype = "Plate")
            
            # if status1 == 1:                 
            #     print("Needle - Needle collision ")
            #     sta_inp =input("Do you want to overwrite and collide? press 1 to continue :")

            Plot_Figure = RobotBaseStructure()
            Plot_Figure =PlotFunction_Cylinders(np.array(CurrentEntryList),np.array(CurrentTargetList),np.array(CurrentRadii),np.array(EntryList),np.array(TargetList),np.array(Needle_radius),Plot_Figure,'viridis')
            
            sta_inp =input("Do you want to overwrite and collide? press 1 to continue :")
            movement = input("Do you want to execute the movement ? press 1 to continue :")
            status2,statuspullback ,FInalJ,PullBackJ,Final_Orientation,Plot_Figure,pose_flag = FunctMain(New_Entry,np.array(EntryTarget[TargetString]),np.array(EntryTargetRadius),np.array(EntryList),np.array(TargetList),np.array(Needle_radius),ETCList,EntryTargetStringList,Old_Entry,Plot_Figure,statustype = "Plate")
            if(pose_flag == 0):
                print("Found pose optimization solution!")
                if(status2 ==0) or (sta_inp == 1): #if there is no R-N collision or you want to overwrite collision

                    if(statuspullback == 0):
                        print("No colliison during pullback !")
                        
                        if(movement == 1):
                            print("Making the movement!")
                            # Trajectory 1
                            
                            # status = ActualMovement(Final_Jvalues,0.15,0.15,0.02,robo)
                            # time.sleep(0.5)
                            # TrajectoryProfiling(FInalJ[0:17])
                            # time.sleep(0.5)
                            
                            # count_1 = 1
                            # while(count_1 == 1):
                            #     if robo.is_program_running() == False:
                            #         count_1 = 0
                            #         time.sleep(0.2)
                            # # Trajectory 2 
                                    
                            # v= np.array(np.array(Old_Entry)-np.array(New_Entry))
                            # Position =[(v[0])/1000.0,v[1]/1000.0,v[2]/1000.0,0,0,0]  
                            # time.sleep(1.5)                      
                            # robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                            
                            # # stat = robo_correction(robo,End_point)
                            # # time.sleep(1.5)

                            # count_2 = 1
                            # while(count_2 == 1):
                            #     if robo.is_program_running() == False:
                            #         count_2 = 0
                            #         time.sleep(0.2)
                            # print("Robot Positioned") 
                            
                            # intpullback =input("Do you want to Pullback? press 1 to continue :")
                            # if(intpullback == 1):
                                
                            #     SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                            #                         -1.970654150048727,
                            #                         1.1324918905841272,
                            #                         4.770189034729757,
                            #                         -1.6497204939471644,
                            #                         0.0055446624755859375] 
                            #     vector2 = -np.array(Final_Orientation[:,2]) - np.array(Old_Entry)
                            #     vector2 = vector2 /Norm(vector2)
                            #     vector2 = vector2*45  
                            #     #MOve 45 mm in z axis wrt tool        
                            #     Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]                        
                            #     robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                            #     time.sleep(1.5)
                                
                            #     count_2 = 1
                            #     while(count_2 == 1):
                            #         if robo.is_program_running() == False:
                            #             count_2 = 0
                            #             time.sleep(0.2)

                            #     # Move 100 mm upwards wrt robot base
                            #     robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
                            #     time.sleep(1.5)

                            #     count_1 = 1
                            #     while(count_1 == 1):
                            #         if robo.is_program_running() == False:
                            #             count_1 = 0
                            #             time.sleep(0.2)                                
                            #     time.sleep(1.0)
                            #     #Move to Home Position
                            #     robo.movej((SafeHomePosition),0.2,0.2,wait=False)
                            #     time.sleep(1.0)

                            #     Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                            #     print("Showing Final graph")         
                            #     ShowCurrentPlot(Plot_Figure)
                                
                            if(i == 1):

                                EntryList = []
                                TargetList = []
                                Needle_radius = []
                            # TargetList.append(EntryTarget[TargetString])
                            # EntryList.append(ModifiedEntry)
                            # Needle_radius.append(NeedleRadius)
                            TargetList.append(Target)
                            EntryList.append(ModifiedEntry)
                            Needle_radius.append(NeedleRadius)                
                            TargetList.append(NeedleHolderPoint1)
                            EntryList.append(NeedleHolderPoint2)
                            Needle_radius.append(Holder_Radius)
                    
                    else :
                        print("Collision During PullBack!. ")
                        ovrpullback = int(input("Do you want to overwrite pullback and collide ? Press 1 to continue :"))
                        if(ovrpullback == 1):
                            if(movement == 1):
                                print("Making the movement!")
                                # Trajectory 1
                                
                                # status = ActualMovement(Final_Jvalues,0.15,0.15,0.02,robo)
                                # time.sleep(0.5)
                                # TrajectoryProfiling(FInalJ[0:17])
                                # time.sleep(0.5)
                                
                                # count_1 = 1
                                # while(count_1 == 1):
                                #     if robo.is_program_running() == False:
                                #         count_1 = 0
                                #         time.sleep(0.2)
                                # # Trajectory 2 
                                        
                                # v= np.array(np.array(Old_Entry)-np.array(New_Entry))
                                # Position =[(v[0])/1000.0,v[1]/1000.0,v[2]/1000.0,0,0,0]  
                                # time.sleep(1.5)                      
                                # robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                                
                                # stat = robo_correction(robo,End_point)
                                # time.sleep(1.5)

                                # count_2 = 1
                                # while(count_2 == 1):
                                #     if robo.is_program_running() == False:
                                #         count_2 = 0
                                #         time.sleep(0.2)
                                # print("Robot Positioned") 
                                # intpullback =input("Do you want to Pullback? press 1 to continue :")
                                # if(intpullback == 1):
                                    
                                #     SafeHomePosition=  [0.45477795600891113, #LEFT SIDE DOCKING
                                #                         -1.970654150048727,
                                #                         1.1324918905841272,
                                #                         4.770189034729757,
                                #                         -1.6497204939471644,
                                #                         0.0055446624755859375] 


                                #     vector2 = -np.array(Final_Orientation[:,2]) - np.array(Old_Entry)
                                #     vector2 = vector2 /Norm(vector2)
                                #     vector2 = vector2*45          

                                #     # Move 45 mm in z axis wrt tool
                                #     Position =[(vector2[0]/1000.0),(vector2[1]/1000.0),(vector2[2]/1000.0),0,0,0]                        
                                #     robo.movel((Position), 0.04, 0.05, wait=False, relative=True, threshold=None)
                                #     time.sleep(1.5)
                                    
                                #     count_2 = 1
                                #     while(count_2 == 1):
                                #         if robo.is_program_running() == False:
                                #             count_2 = 0
                                #             time.sleep(0.2)

                                #     # Move 100 mm upwards wrt robot base
                                #     robo.movel([0,0,0.1,0,0,0], 0.04, 0.05, wait=False, relative=True, threshold=None)
                                #     time.sleep(1.5)

                                #     count_1 = 1
                                #     while(count_1 == 1):
                                #         if robo.is_program_running() == False:
                                #             count_1 = 0
                                #             time.sleep(0.2)                                
                                #     time.sleep(1.0)
                                #     # Move to Home Position
                                #     robo.movej((SafeHomePosition),0.2,0.2,wait=False)
                                #     time.sleep(1.0)

                                #     Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                                #     print("Showing Final graph")         
                                #     ShowCurrentPlot(Plot_Figure)
                                    
                                if(i == 1):

                                    EntryList = []
                                    TargetList = []
                                    Needle_radius = []
                            
                                TargetList.append(Target)
                                EntryList.append(ModifiedEntry)
                                Needle_radius.append(NeedleRadius)                
                                TargetList.append(NeedleHolderPoint1)
                                EntryList.append(NeedleHolderPoint2)
                                Needle_radius.append(Holder_Radius)
                            

                else : 
                    
                    print("Robot - needle collision!. Try planning away from needle")
                    Plot_Figure = plot_frames(robo,Plot_Figure,FInalJ)                        
                    print("Showing Final graph")         
                    ShowCurrentPlot(Plot_Figure)
                    if(i == 1):
                        EntryList = []
                        TargetList = []
                        Needle_radius = []  
                Counter = Counter +1
                i = i + 1
            else:
                print("Pose Optimization Failed !")   
    except KeyboardInterrupt:        
        print('interrupted!')



if __name__ == '__main__':
    # robo = urx.Robot("172.16.101.224")
    EntryTargetComputation()










