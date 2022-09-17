#imports 
import numpy as np
from modules import *
import plotly.graph_objects as go
from numpy import *
# from CollisionFunction import ComputeTransformation
from CylinderCollision import *
from RobotSelfCollisionModified import *
#Functions : 

def ShowCurrentPlot(Plot_Figure):
    Plot_Figure.show()
    print("Updated graph is shown")

#Visualizing functions

def Visualizing_Cylinder(Cylinder_Point1,Cylinder_Point2,r,colorscale,opacity):
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
                          showscale=False,opacity=opacity)
    cyl_bottom = go.Surface(x=X2, y=Y2, z=Z2,
                          colorscale=colorscale,
                          showscale=False,opacity=opacity)
    cyl_top = go.Surface(x=X3, y=Y3, z=Z3,
                          colorscale=colorscale,
                          showscale=False,opacity=opacity)
    return(cyl_tube,cyl_bottom,cyl_top)


def TraceCollisionPoints(CollisionPoints):

    Px =[]
    Py =[]
    Pz =[]
    
    for j in range(len(CollisionPoints)):           
                    
        Px.append(CollisionPoints[j][0])
        Py.append(CollisionPoints[j][1])
        Pz.append(CollisionPoints[j][2])
    Collision_Points = go.Scatter3d(x = Px,
                                y = Py,
                                z = Pz,  
                                    mode= 'markers',                
                                    marker=dict(
                                    size=5,
                                    color='blue',               
                                    colorscale='Viridis',  
                                    line=dict(width=1,color='DarkSlateGrey'),
                               opacity=1))
    # fig.add_trace(Collision_Points)
    return(Collision_Points)


def SepPoints(data):
    xpoints = []
    ypoints = []
    zpoints = []

    for i in range(len(data)):
        xpoints.append((data[i][0]))
        ypoints.append((data[i][1]))
        zpoints.append((data[i][2]))
        
    return xpoints,ypoints,zpoints

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
    'color': 'yellow',
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

# def RobotBaseStructure():
    
#     thetas = [0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
#     temp_joint_locations = thetas
#     sl = JointLocations(temp_joint_locations)
#     x_p,y_p,z_p = SepPoints(sl)

#     x_p.insert(1,0)
#     x_p.insert(2,-2.602374e-17)

#     y_p.insert(1,-0.1333)
#     y_p.insert(2,-0.1333)

#     z_p.insert(1,0.1625)
#     z_p.insert(2,0.5875)    


#     x_p=list(np.asarray(x_p) * 1000)
#     y_p=list(np.asarray(y_p) * 1000)
#     z_p=list(np.asarray(z_p) * 1000)

#     data=go.Scatter3d(x=x_p,
#                         y=y_p,
#                         z=z_p,marker=dict(
#                 size=[58,58,48,48,37.5,37.5,37.5,37.5],
#                 opacity = 0,
#                 color = ('rgb(22, 96, 167)'),              
#                 colorscale='Viridis',  

#             ),
#                 line = dict(
#                 colorscale="Viridis",
#                 width = 50,
#                 ),
#             )
#     fig = go.Figure(data = data)
#     fig.update_layout(scene=dict(zaxis=dict(range=[-925,925],autorange=False),       
#                         yaxis=dict(range=[-925,925],autorange=False),
#                     xaxis=dict(range=[-925,925],autorange=False)))               
#     fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

#     fig.add_trace(data)
#     Data = go.Mesh3d(
        
#         x=[400, 400, 0, 0, 400, 400, 0, 0],
#         y=[400, -450, -450, 400, 400, -450, -450, 400],
#         z=[0, 0, 0, 0, -700, -700, -700, -700],
#         colorbar_title='z',
       
#         i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
#         j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
#         k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
#         name='y',
#         showscale=True
#     )
#     fig.add_trace(Data)
    
#     #Base cylinder
#     r1 = 45
#     a1 = 0
#     h1 = 150
#     x1, y1, z1 = RobotBaseCylinder(r1, h1, a=a1)
#     colorscale = [[0, 'red'],
#                     [1, 'red']]
#     cyl1 = go.Surface(x=x1, y=y1, z=z1,
#                     colorscale=colorscale,
#                     showscale=False,)
#     #############auto -925 925

#     fig.add_trace(cyl1)

#     #Plane data:

#     height = -450
#     Pp1 = [-800,-750,height]
#     Pp2 = [-250,-750,height]
#     Pp3 = [-800,750,height]
#     Pp4 = [-250,750,height]

#     fig = plane(fig,Pp1,Pp2,Pp3,Pp4)

#     return fig


def TracePath(fig,xp,yp,zp):

    Pathdata = go.Scatter3d(x=xp, y=yp, z=zp,
                             marker=dict(
            size=3,
    #         color=z,                
            color = ('rgb(22, 96, 167)'),   
            opacity=0.9
        ),line = dict(width=4))
    
    fig.add_trace(Pathdata)
    return fig,Pathdata

def TraceEntTarpoints(fig,Ep,Tp):

    EpTpPoints = go.Scatter3d(x=[Ep[0],Tp[0]], y=[Ep[1],Tp[1]], z=[Ep[2],Tp[2]],
                            line = dict(width=4),
                            marker=dict(
            size = 5,
    #         color=z,                
            color = ('rgb(78, 12, 11)'),   
            opacity=0.9
        ))
    
    fig.add_trace(EpTpPoints)
    return fig,EpTpPoints

def plot_frames(robo,fig,FinalJ):
    
    fig.update_layout(width=1800,
                scene_camera_eye_z=0.75)
                # title="Start Title")
            # updatemenus=[dict(
            # type="buttons",
            # buttons=[dict(label="Play",
            #             method="animate",
            #             args=[None])])])
    
    # print(len(FinalJ))
    cjv = FinalJ[19]
    dumy,values = JointLocationsEF_URCT(cjv) 
    CylinderBottom,CylinderTop,Radii = GenerateCylinders_URCT(values)
    fig = PlotFunction_Cy(CylinderBottom,CylinderTop,Radii,fig,1.0)
       

    # b=list(fig.frames)
    # fig.frames = tuple(b)
    # fig.add_trace(data)

    return fig



def DisplayEllipsoid(fig,c,TMat,r1,r2):
    
    theta = linspace(0,2*np.pi,100)
    phi = linspace(0,np.pi,100)
    x =  (r1) * outer(cos(theta),sin(phi))
    y =  (r2) * outer(sin(theta),sin(phi))
    z =  (r2) * outer(ones(100),cos(phi))
    x1,y1,z1 = ComputeTransformation(x.flatten(),y.flatten(),z.flatten(),TMat,1)

    x1 = c[0] + x1
    y1 = c[1] + y1
    z1 = c[2] + z1

    EllipsoidData=go.Surface(
    x=x1,
    y=y1,
    z=z1,
    opacity=0.8)

    fig.add_trace(EllipsoidData)

    return fig


def DeleteGraphPoints(Plot_Figure,Pathdata,EpTpPoints):
    Current_data = list(Plot_Figure.data)
    if Pathdata != 0:
        Current_data.remove(Pathdata)
    Current_data.remove(EpTpPoints)
    Plot_Figure.frames = ()
    Plot_Figure.data = tuple(Current_data)

    return Plot_Figure


    
def FrameVisual(fig,CollidingFrame):
    ## radius and number of lines can be added into the FrameVisual argumennts too
    Link_radius =[58,58,48,37.5,37.5]
    NoOfDivivsions = [20,20,20,20,20]
    x = np.asarray(CollidingFrame[0])*1000
    y = np.asarray(CollidingFrame[1])*1000
    z = np.asarray(CollidingFrame[2])*1000
    for i in range(len(x)-1):
        pt1,pt2 = LineToCylinder([x[i],y[i],z[i]],[x[i+1],y[i+1],z[i+1]],Link_radius[i],NoOfDivivsions[i])
        for j in range(len(pt1)-1):
            fig,pts = TraceEntTarpoints(fig,pt1[j],pt2[j+1])
    return fig
    

def RobotBaseStructure():
   
    # thetas = [0,-np.pi/2,np.pi/2,0,np.pi/2,np.pi]
    # temp_joint_locations = thetas
    # sl = JointLocations(temp_joint_locations)
    # x_p,y_p,z_p = SepPoints(sl)

    # x_p.insert(1,0)
    # x_p.insert(2,-2.602374e-17)

    # y_p.insert(1,-0.1333)
    # y_p.insert(2,-0.1333)

    # z_p.insert(1,0.1625)
    # z_p.insert(2,0.5875)
   
    # p1 = [x_p[-2],y_p[-2],z_p[-2]]
    # p2 = [x_p[-1],y_p[-1],z_p[-1]]
   
    # P = p2 + 1.12*np.subtract(p2,p1)
    # x1 = P[0]
    # y1 = P[1]
    # z1 = P[2]
    # x_p.append(x1)
    # y_p.append(y1)
    # z_p.append(z1)    


    # x_p=list(np.asarray(x_p) * 1000)
    # y_p=list(np.asarray(y_p) * 1000)
    # z_p=list(np.asarray(z_p) * 1000)


    # data=go.Scatter3d(x=x_p,
    #                     y=y_p,
    #                     z=z_p,marker=dict(
    #             size=[58,58,48,48,37.5,37.5,37.5,37.5,37.5],
    #             opacity = 0,
    #             color = ('rgb(22, 96, 167)'),              
    #             colorscale='Viridis',  

    #         ),
    #             line = dict(
    #             colorscale="Viridis",
    #             width = 50,
    #             ),
    #         )
    # fig = go.Figure(data = data)
    # fig.update_layout(scene=dict(zaxis=dict(range=[-925,925],autorange=False),      
    #                     yaxis=dict(range=[-925,925],autorange=False),
    #                 xaxis=dict(range=[-925,925],autorange=False)))              
    # fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))

    # fig.add_trace(data)
    # Data = go.Mesh3d(
       
    #     x=[400, 400, 0, 0, 400, 400, 0, 0],
    #     y=[400, -450, -450, 400, 400, -450, -450, 400],
    #     z=[0, 0, 0, 0, -700, -700, -700, -700],
    #     colorbar_title='z',
       
    #     i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
    #     j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
    #     k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
    #     name='y',
    #     showscale=True
    # )
    # fig.add_trace(Data)
   
    # #Base cylinder
    # r1 = 45
    # a1 = 0
    # h1 = 150
    # x1, y1, z1 = RobotBaseCylinder(r1, h1, a=a1)
    # colorscale = [[0, 'red'],
    #                 [1, 'red']]
    # cyl1 = go.Surface(x=x1, y=y1, z=z1,
    #                 colorscale=colorscale,
    #                 showscale=False,)
    # #############auto -925 925

    # fig.add_trace(cyl1)

    # #Plane data:

    # height = 0
    # Pp1 = [-800,-750,height]
    # Pp2 = [-250,-750,height]
    # Pp3 = [-800,750,height]
    # Pp4 = [-250,750,height]

    # fig = plane(fig,Pp1,Pp2,Pp3,Pp4)

    # return fig


    fig = go.Figure()
    fig.update_layout(scene=dict(zaxis=dict(range=[-925,925],autorange=False),      
                        yaxis=dict(range=[-925,925],autorange=False),
                    xaxis=dict(range=[-925,925],autorange=False)))              
    fig.update_layout(scene_aspectratio=dict(x=1.4,y=1,z=1.4))
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
    # #Base cylinder
    # r1 = 45
    # a1 = 0
    # h1 = 150
    # x1, y1, z1 = RobotBaseCylinder(r1, h1, a=a1)
    # colorscale = [[0, 'red'],
    #                 [1, 'red']]
    # cyl1 = go.Surface(x=x1, y=y1, z=z1,
    #                 colorscale=colorscale,
    #                 showscale=False,)
    # #############auto -925 925

    # fig.add_trace(cyl1)

    #Plane data:

    height = -100
    Pp1 = [-800,-750,height]
    Pp2 = [-250,-750,height]
    Pp3 = [-800,750,height]
    Pp4 = [-250,750,height]

    fig = plane(fig,Pp1,Pp2,Pp3,Pp4)
    cjv =  [0.45477795600891113, #LEFT SIDE DOCKING
            -1.970654150048727,
            1.1324918905841272,
            4.770189034729757,
            -1.6497204939471644,
            0.0055446624755859375]
    dumy,values = JointLocationsEF_URCT(cjv)
    # print(values)
    
    CylinderBottom,CylinderTop,Radii = GenerateCylinders_URCT(values)
    fig = PlotFunction_Cy(CylinderBottom,CylinderTop,Radii,fig,0.3)
   
    return(fig)

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

                

def Cylinder_Cylinder_Collision_NN(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder2_Point_Bottom,Cylinder2_Point_Top,Radii_2,NumLines,NumPoints,EntryTargetString,EntryTargetStringList,ETCList,statustype):
    intersected =0
    Plot_Figure = RobotBaseStructure()
    Plot_Figure =PlotFunction_Cylinders(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder2_Point_Bottom,Cylinder2_Point_Top,Radii_2,Plot_Figure,'viridis')
    # print("Inside Collision NN")
    for i in range(len(Cylinder1_Point_Bottom)):        
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]         
        
        for j in range(len(Cylinder2_Point_Bottom)):         
            
            Cylinder2_Point1 =Cylinder2_Point_Bottom[j]        
            Cylinder2_Point2 =Cylinder2_Point_Top[j]            
            Radius_2 = Radii_2[j]

            

            if(Cylinder_Cylinder(Cylinder1_Point1 ,Cylinder1_Point2,Radius_1,Cylinder2_Point1,Cylinder2_Point2,Radius_2,NumLines,NumPoints) ==1 ):
               
                if(statustype == "Plate"):
                    counter = WhichNeedle(Cylinder2_Point1,ETCList,4) 
                    print("Collision between Needles placed in "+EntryTargetString+" and "+EntryTargetStringList[counter])
                elif(statustype == "restAPI"):
                    counter = WhichNeedle(Cylinder2_Point1,ETCList,2) 
                    print("Collision between Needles "+EntryTargetString+" and "+EntryTargetStringList[counter])
                Plot_Figure =PlotFunction_Cylinders(np.array([Cylinder1_Point1]) ,np.array([Cylinder1_Point2]),np.array([Radius_1]),np.array([Cylinder2_Point1]),np.array([Cylinder2_Point2]),np.array([Radius_2]),Plot_Figure,'reds')
                
                intersected =1    
                break
    
    if(intersected == 1):
        ShowCurrentPlot(Plot_Figure)        
        return 1,Plot_Figure
        
    elif(intersected == 0):
         
        return 0,Plot_Figure

def PlotFunction_Cylinders(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Cylinder2_Point_Bottom,Cylinder2_Point_Top,Radii_2,Plot_Figure,colorscale):
    
    for i in range(len(Cylinder1_Point_Bottom)):
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]
        cyl_tube_1,cyl_bottom_1,cyl_top_1= Visualizing_Cylinder(Cylinder1_Point1, Cylinder1_Point2, Radius_1,colorscale,1.0)   
            
        Plot_Figure.add_trace(cyl_tube_1)
        Plot_Figure.add_trace(cyl_bottom_1)
        Plot_Figure.add_trace(cyl_top_1)  

    
        for j in range(len(Cylinder2_Point_Bottom)):
            Cylinder2_Point1 = Cylinder2_Point_Bottom[j]
            Cylinder2_Point2 = Cylinder2_Point_Top[j]
            r = Radii_2[j]
            cyl_tube_2,cyl_bottom_2,cyl_top_2= Visualizing_Cylinder(Cylinder2_Point1,Cylinder2_Point2,r,colorscale,1.0)
            # LinesPoint=LinesCylinderPlot(Cylinder_Point1,Cylinder_Point2,r,NumLines,NumPoints)
            # Plot_Figure.add_trace(LinesPoint)
            Plot_Figure.add_trace(cyl_tube_2)
            Plot_Figure.add_trace(cyl_bottom_2)
            Plot_Figure.add_trace(cyl_top_2)      
        
    return Plot_Figure    

def PlotFunction_CylinderPoints(Cylinder1_Point_Bottom,Cylinder1_Point_Top,Radii_1,Plot_Figure):
    
    
    #Cylinder-Pathpoints
    colorscale= 'viridis'
    for i in range(len(Cylinder1_Point_Bottom)):   
        Cylinder1_Point1 =Cylinder1_Point_Bottom[i]
        Cylinder1_Point2 =Cylinder1_Point_Top[i]
        Radius_1 = Radii_1[i]    
        cyl_tube_1,cyl_bottom_1,cyl_top_1= Visualizing_Cylinder(Cylinder1_Point1, Cylinder1_Point2, Radius_1,colorscale,1.0)   
            
        Plot_Figure.add_trace(cyl_tube_1)
        Plot_Figure.add_trace(cyl_bottom_1)
        Plot_Figure.add_trace(cyl_top_1)  
    # CollisionPoints =TraceCollisionPoints(CollisionPoints)/
    # Plot_Figure.add_trace(CollisionPoints)
    return Plot_Figure

def PlotFunction_Cy(Cylinder_Point_Bottom,Cylinder_Point_Top,Radii_,Plot_Figure,Opacity):
    
    
    
    colorscale= 'purples'
    for i in range(len(Cylinder_Point_Bottom)):   
        Cylinder_Point1 =Cylinder_Point_Bottom[i]
        Cylinder_Point2 =Cylinder_Point_Top[i]
        Radius = Radii_[i]    
        cyl_tube_1,cyl_bottom_1,cyl_top_1= Visualizing_Cylinder(Cylinder_Point1, Cylinder_Point2, Radius,colorscale,Opacity)   
            
        Plot_Figure.add_trace(cyl_tube_1)
        Plot_Figure.add_trace(cyl_bottom_1)
        Plot_Figure.add_trace(cyl_top_1)
        

    
    return Plot_Figure 