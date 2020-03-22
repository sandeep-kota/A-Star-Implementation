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


    def isObstacle(self,j,i):
        """
        Determines if obstacle using half-plane equations.
    
        :param      j:    x-coordinate
        :type       j:    flloat
        :param      i:    y-coordinate
        :type       i:    float
    
        :returns:   True if obstacle, False otherwise.
        :rtype:     boolean
        """
        r=self.radius
        c=self.clearence
   
        obstacle = False
        
        if ((j>=0) and (j<=(r+c))) or (j>=(300-(r+c)) and (j<=300)) or ((i>=0) and (i<=(r+c))) or ((i>=200-(r+c)) and (i<=200)):
            obstacle = True 
        
        # Circle
        if ((j-225)**2 + (i-50)**2 <= ((25+r+c)**2)):
            # b[i,j]=255
            obstacle = True

        # Diamond
        if ((0.6*j+i-(295-(r+c)))>=0) and ((0.6*j+i-(325+(r+c)))<=0) and ((-0.6*j+i-(25-(r+c)))>=0) and ((-0.6*j+i-(55+(r+c)))<=0):
            obstacle = True


        # Eclipse
        if (((j-150)/(40+r+c))**2 + ((i-100)/(20+r+c))**2 <= 1   ):
            obstacle = True

        # Line
        if ((i-0.58*j-(115.15+(r+c)))<=0)  and ((i-0.58*j-(103.6-(r+c)))>=0) and ((i+1.73*j-(184.55-(r+c)))>=0) and ((i+1.73*j-(334.55+(r+c)))<=0):
            obstacle = True

        # Right Poly
        if ((i-(15-(r+c))>=0)) and ((i-1.4*j+(90+(r+c)))>=0) and ((i+1.2*j-(170+(r+c)))<=0) and ((i-1.2*j+(10-(r+c)))<=0) and ((i+13*j-(340-(r+c)))>0):
            obstacle = True

        if ((i+j-(100+(r+c)))<=0) and ((i+13*j-(340-(r+c)))>=0) and ((i-1.4*j+(90+(r+c)))>=0) and (i-(15-(r+c))>=0):
            obstacle = True

        return obstacle
        
    def cost(self,node,step_cost):
        """
        Returns 
    
        :param      node:       The node at which the cost is to be obtained 
        :type       node:       List
        :param      step_cost:  The step cost to reach that node from start node
        :type       step_cost:  int
    
        :returns:   Heuristic cost for A*
        :rtype:     float
        """
        return(step_cost + np.linalg.norm(np.array(node[0:2])-np.array(self.goal[0:2])))


    def actionsAvailable(self,x,y,theta,step_cost):
        """
        Returns 5 actions available at the given nodes 
    
        :param      x:          The current node x
        :type       x:          float
        :param      y:          The current node y
        :type       y:          float
        :param      theta:      The theta
        :type       theta:      Int in range (0,12)
        :param      step_cost:  The step cost
        :type       step_cost:  Int
        """
        for i in range(0,5):
            i = 4-i

            # Check the states at -60, -30, 0, 30, 60
            xn = (x+(self.step_size*np.sin((theta+i-2)*np.pi/6)))
            yn = (y+(self.step_size*np.cos((theta+i-2)*np.pi/6)))
            
            # Edit the states -1, -2, etc to 11, 10, etc so on
            alpha = theta+i-2
            if alpha <0:
                alpha = alpha +12
            # Edit the states 13, 14, etc to 1, 2, etc so on
            if alpha >11:
                alpha = alpha-12

            # Check obstacle condition for the explored nodes 
            if self.isObstacle(xn,yn) == False:

                # Check already visited condition for explored nodes 
                if (np.sum(self.visited_map[int(round(yn*2)),int(round(xn*2)),:]))==0:
                    self.visited_map[int(round(yn*2)),int(round(xn*2)),alpha]=1
                    self.anim[int(yn),int(xn)]=[255,0,0]
                    self.queue.append([self.cost((xn,yn),step_cost),xn,yn,alpha,step_cost+2,x,y ])
                    heapq.heapify(self.queue)

                elif (np.sum(self.visited_map[int(round(yn*2)),int(round(xn*2)),:]))>0:
                    pass

        # print("-------")

    def backtrack(self):
        n= len(self.visited)
        parent = []
        j = 0
        print("Backtracking")
        # print(np.array(self.visited))
        self.shortest_path.append([self.goal[0],self.goal[1]])
        while(True):
            popped = self.visited[n-1-j]
            current_node = [popped[1],popped[2]]
            parent_node = [popped[-2],popped[-1]]
            # print("Current Node :",current_node)
            # print("Parent Node :",parent_node)
            parent.append(parent_node)
            self.anim[int(parent_node[1]),int(parent_node[0])]=[0,0,255]
            self.shortest_path.append([parent_node[0],200-parent_node[1]])
            if list(current_node) == [self.start[0],self.start[1]]:
                break
            # cv2.imshow("Anim",self.anim)
            self.out.write(self.anim)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # Extract the explored nodes columns of the queue
            cp = np.array(self.visited)[:,1:3]
            # print("CP: ", np.array(cp)[9])

            # Return the index of the parent node in the explored node columns of the queue
            for i in range(0,cp.shape[0]):
                if (cp[i][0]==parent_node[0]) and (cp[i][1]==parent_node[1]):
                    # print("Found at ",i)
                    j = n-1-i
        self.out.release()

    def astar(self):
        """
        A Star Alorithm
        """
        heapq.heapify(self.queue)
        heapq.heappush(self.queue,[self.cost(self.start[:2],0),self.start[0],self.start[1],self.start[2],0,self.start[0],self.start[1]])
        while True:

            # Pop the element with least cost
            current = heapq.heappop(self.queue)
            self.visited.append(current)
            # print("current_node", np.array(current))

            # Check Goal Condition
            if np.linalg.norm(np.array(current[1:3])- np.array(self.goal[0:2])) <= 3 :
                print("Goal Reached! ")
                # print("Visited" , np.array(self.visited))
                # cv2.imshow("Animation :",self.anim)
                # cv2.waitKey()
                 
                # Perform backtracking
                self.backtrack()
                break 

            # Search for the available actions in the popped element of the queue
            self.actionsAvailable(current[1],current[2],current[3],current[4])
            print("|________________|")  

            # Represent that node in the animation map
            self.anim[int(round(current[2])),int(round(current[1]))]= [0,255,0]
            # cv2.imshow("Animation :",self.anim)
            self.out.write(self.anim)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # cv2.waitKey()
        self.out.release()
        