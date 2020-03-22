import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import sys
import cv2

class Map:
    def __init__(self,start,goal,clearence,radius,step_size):
        """
        Constructs a new instance.
    
        :param      start:      The start
        :type       start:      Start Node coordinates
        :param      goal:       The goal
        :type       goal:       Goal Node coordinates
        :param      clearence:  The clearence
        :type       clearence:  Int
        :param      radius:     The radius
        :type       radius:     Int
        :param      step_size:  The step size
        :type       step_size:  Int
        """
        self.visited_map = np.zeros((401,601,12),np.uint8)
        self.start = start
        self.goal = goal
        self.step_size = step_size
        self.queue=[]
        self.visited = []
        self.shortest_path = [] 
        self.radius= radius
        self.clearence= clearence
        r = self.radius
        c = self.clearence
        self.anim = np.zeros((200,300,3),np.uint8)
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output.avi',self.fourcc,20.0,(300,200))

        #!---------IMPORTANT----------------!
        # Discretizing the map for animation purpose only
        for j in range(0,300):
            for i in range(0,200):
                if ((j>=0) and (j<=(r+c))) or (j>=(300-(r+c)) and (j<=300)) or ((i>=0) and (i<=(r+c))) or ((i>=200-(r+c)) and (i<=200)):
                    self.anim[i,j]=[0,255,0]

                # Circle
                if ((j-225)**2 + (i-50)**2 <= ((25+r+c)**2)):
                    self.anim[i,j]=255

                # Ellipse
                if (((j-150)/(40+r+c))**2 + ((i-100)/(20+r+c))**2 <= 1   ):
                    self.anim[i,j]=255

                # Diamond
                if ((0.6*j+i-(295-(r+c)))>=0) and ((0.6*j+i-(325+(r+c)))<=0) and ((-0.6*j+i-(25-(r+c)))>=0) and ((-0.6*j+i-(55+(r+c)))<=0):
                    self.anim[i,j]=255

                # Line
                if ((i-0.58*j-(115.15+(r+c)))<=0)  and ((i-0.58*j-(103.6-(r+c)))>=0) and ((i+1.73*j-(184.55-(r+c)))>=0) and ((i+1.73*j-(334.55+(r+c)))<=0):
                    self.anim[i,j]=255

                # Right Poly
                if ((i-(15-(r+c))>=0)) and ((i-1.4*j+(90+(r+c)))>=0) and ((i+1.2*j-(170+(r+c)))<=0) and ((i-1.2*j+(10-(r+c)))<=0) and ((i+13*j-(340-(r+c)))>=0):
                    self.anim[i,j]=255

                if ((i+j-(100+(r+c)))<=0) and ((i+13*j-(340-(r+c)))>=0) and ((i-1.4*j+(90+(r+c)))>=0) and (i-(15-(r+c))>=0):
                    self.anim[i,j]=255  
