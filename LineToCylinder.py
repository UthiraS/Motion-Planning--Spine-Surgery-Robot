import math
import numpy as np

def EuclideanDistance(vec1,vec2):
    E2Distance = np.sqrt( np.square(vec1[0]- vec2[0]) + np.square(vec1[1]- vec2[1]) + np.square(vec1[2]- vec2[2]))
    return E2Distance

def OrientMatixRotation(entry,target,theta):
    
    #vector y:
    v_z = np.asarray(target) - np.asarray(entry)
    #unit vector
    denom = np.sqrt( np.square(v_z[0]) + np.square(v_z[1]) + np.square(v_z[2]))
    vector_z = -( v_z / denom)
        
    #vector z:
    dumyvector = [vector_z[0]+10,vector_z[1]+10,vector_z[2]+10]
    vector1 = np.cross(vector_z,dumyvector)
    v_x = np.cross(vector_z,vector1)
    denom1 = np.sqrt( np.square(v_x[0]) + np.square(v_x[1]) + np.square(v_x[2]))
    vector_x = v_x /denom1
    
    #Rotation shift:Rodrigues form
    comp1 = vector_x * np.cos(theta)
    comp2 = np.cross(vector_z,vector_x) * np.sin(theta)
    comp3 = (vector_z * (np.dot(vector_z,vector_x)))* (1 - np.cos(theta))
    Rodgriues = comp1 + comp2 + comp3
    vector_x = Rodgriues
    
    #vector x:
    vector_y = np.asarray ( np.cross(vector_z,vector_x))

    return vector_x,vector_y,vector_z


def LineToCylinder(Entry,Target,radius,nol):
    var = 0

    Point1List = []
    Point2List = []
        
    for inc in np.arange(0,2*math.pi,(2*math.pi)/nol ):
        x,y,z = OrientMatixRotation(Entry,Target,inc) 
        shiftedP1 = Target + (x * radius)
        shiftedP2 =  Entry + (x * radius)

        Point1List.append(shiftedP1)
        Point2List.append(shiftedP2)

    return Point1List,Point2List 