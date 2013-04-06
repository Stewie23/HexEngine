'''
Created on 18.07.2012

@author: benjamin
'''

from heapq import heappush, heappop
from operator import itemgetter
import math
import HexMath
class Map:
    
    def __init__(self):
        self.x = 0 # dimension x
        self.y = 0 # dimension y
        self.radius = 0 #radius of the hextiles
        self.height = 0
        self.width = 0
        self.side  = 0
        self.tiles = [] #two dimensional array storring the tiles
                        #acess is[y][x] 
        self.cache = None               
    def LoadMap(self):
        #simple load the debug.map and parse it to draw map    
        map = open('map/debug.map','r')
        count = 0
        
        for line in map:
            if count == 0: #first line get dimensions and radius: x,y,radius
                firstLine = line.split(',')
                self.x = int(firstLine[0])
                self.y = int(firstLine[1])
                self.radius = int(firstLine[2])
                self.calcDimensions()#calculate values derived from radius
                #build 2d array
                row = [0] * (self.y+1) 
                for i in range(self.x):
                    self.tiles.append(list(row))
                
            else: #create tiles
                n = 0 
                element = line.split(',')
                while n <= self.x:#dummy row
                    if n == 0:
                        self.tiles[count-1][n] = Tile(n,count-1,99,self)#dummy
                    else:
                        self.tiles[count-1][n] = Tile(n,count-1,int(element[n-1]),self)
                        
                    n+=1
            count+= 1
            
                
    def calcDimensions(self):
        self.height = int(self.radius * math.sqrt(3))
        self.side = int(self.radius * 3/2.0)
        self.width = int(self.radius * 2)
        
    def getTile(self,Pos):
        x = Pos[0]
        y = Pos[1]
        
        return self.tiles[y][x]
    
    def getTilebyHex(self,Pos):#get a Tile using hex coords
        arrayPos  = HexMath.convertHexToArray(Pos[0], Pos[1])
        return self.getTile((arrayPos[0],arrayPos[1]))
    
    def getTilesInRange(self,Pos,Range): #returns a list of reachable tiles
        #put nodes into list until g > Range, return list
        dirs = 6
        dxOdd = [0, 1, 1, 1, 0, -1]
        dyOdd = [-1, -1, 0, 1, 1,0]
    
        dxEven = [-1, 0, 1, 0, -1,-1]
        dyEven = [-1, -1, 0, 1, 1,0]
        closedNodes = []
        openNodes = []
        row = [0] * self.y
        for i in range(self.x): # create 2d arrays
            closedNodes.append(list(row))
            openNodes.append(list(row))
            #dir_map.append(list(row))
        pq = [[], []] # priority queues of open (not-yet-tried) nodes
        pqi = 0 # priority queue index
        # create the start node and push into list of open nodes
        n0 = node(Pos[0],Pos[1], 0, 0,0) #no movmentcosts for startnode
        n0.updatePriority(0,0,H=False)
        heappush(pq[pqi], n0)
        openNodes[Pos[1]][Pos[0]] = n0.priority # mark it on the open nodes map
        
        # A* search
        while len(pq[pqi]) > 0:
            # get the current node w/ the highest priority
            # from the list of open nodes
            n1 = pq[pqi][0] # top node
            n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority,self.getTile((n1.xPos,n1.yPos)).typ)
            x = n0.xPos
            y = n0.yPos
            heappop(pq[pqi]) # remove the node from the open list
            openNodes[y][x] = 0
            closedNodes[y][x] = 1 # mark it on the closed nodes map
            # generate moves (child nodes) in all possible dirs"
            # hex field madness:)
            if y %2:
                dx = dxOdd
                dy = dyOdd
            else:
                dx = dxEven
                dy = dyEven
            for i in range(dirs):
                xdx = x + dx[i]
                ydy = y + dy[i]
                if not (xdx < 0 or xdx > self.x-1 or ydy < 0 or ydy > self.y-1 or closedNodes[ydy][xdx] == 1 or Range < n0.distance + self.getTile((xdx,ydy)).typ):
                    # generate a child node
                    m0 = node(xdx, ydy, n0.distance, n0.priority,self.getTile((xdx,ydy)).typ)
                    m0.nextMove()
                    m0.updatePriority(0,0,H=False)
                    if openNodes[ydy][xdx] == 0:
                        openNodes[ydy][xdx] = m0.priority
                        heappush(pq[pqi], m0)
                        # mark its parent node direction
                    elif openNodes[ydy][xdx] > m0.priority:
                        # update the priority
                        openNodes[ydy][xdx] = m0.priority
                        # update the parent direction
                        # replace the node
                        # by emptying one pq to the other one
                        # except the node to be replaced will be ignored
                        # and the new node will be pushed in instead
                        while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                            heappush(pq[1 - pqi], pq[pqi][0])
                            heappop(pq[pqi])
                        heappop(pq[pqi]) # remove the target node
                        # empty the larger size priority queue to the smaller one
                        if len(pq[pqi]) > len(pq[1 - pqi]):
                            pqi = 1 - pqi
                        while len(pq[pqi]) > 0:
                            heappush(pq[1-pqi], pq[pqi][0])
                            heappop(pq[pqi])       
                        pqi = 1 - pqi
                        heappush(pq[pqi], m0) # add the better node instead
        
        iY= 0
        returnList=[]
        while iY != self.y:
            iX = 0
            while iX != self.x:
                if closedNodes[iY][iX]== 1:
                    returnList.append((iX,iY))
                iX+=1
            iY+=1
        return returnList
    
    def getTilesbyDistance(self,Pos,maxDistance,minDistance = 0): # returns a list of tiles
        rtrList =[]
        
        for row in self.tiles:
            for tile in row:
                if HexMath.getDistance(Pos[0],Pos[1],tile.x,tile.y) <= maxDistance and HexMath.getDistance(Pos[0],Pos[1],tile.x,tile.y) >= minDistance:
                    rtrList.append((tile.x,tile.y))
        return rtrList
          
    def getPath(self,StartPos,GoalPos):#A* Pathfinding 
        StartPos = (StartPos[0]+1,StartPos[1])#+1 for dummy col
        dirs = 6
        dxOdd = [0, 1, 1, 1, 0, -1]
        dyOdd = [-1, -1, 0, 1, 1,0]
    
        dxEven = [-1, 0, 1, 0, -1,-1]
        dyEven = [-1, -1, 0, 1, 1,0]
        #way faster then old,but still to slow:)
        #10x10 - 0.01
        #20x20 - 0.04
        #40x40 - 0.38
        #------------
        #80x80 - 3.00
        closedNodes = []
        openNodes = []
        dir_map = [] # map of dirs
        row = [0] * self.y
        for i in range(self.x): # create 2d arrays
            closedNodes.append(list(row))
            openNodes.append(list(row))
            dir_map.append(list(row))
        pq = [[], []] # priority queues of open (not-yet-tried) nodes
        pqi = 0 # priority queue index
        # create the start node and push into list of open nodes
        n0 = node(StartPos[0],StartPos[1], 0, 0,0) #no movmentcosts for startnode
        n0.updatePriority(GoalPos[0],GoalPos[1])
        heappush(pq[pqi], n0)
        openNodes[StartPos[1]][StartPos[0]] = n0.priority # mark it on the open nodes map
        
        # A* search
        while len(pq[pqi]) > 0:
            # get the current node w/ the highest priority
            # from the list of open nodes
            n1 = pq[pqi][0] # top node
            n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority,self.getTile((n1.xPos,n1.yPos)).typ)
            x = n0.xPos
            y = n0.yPos
            heappop(pq[pqi]) # remove the node from the open list
            openNodes[y][x] = 0
            closedNodes[y][x] = 1 # mark it on the closed nodes map
            
            if x == GoalPos[0] and y == GoalPos[1]:
                # generate the path from finish to start
                # by following the dirs
                path = []
                while not (x == StartPos[0] and y == StartPos[1]):
                    j = dir_map[y][x]
                    #path.append((x,y))
                    if y %2:
                        x += dxOdd[j]
                        y += dyOdd[j]
                        path.append((x,y))
                    else:
                        x += dxEven[j]
                        y += dyEven[j]
                        path.append((x,y))
                        
                return path
            # generate moves (child nodes) in all possible dirs"
            # hex field madness:)
            if y %2:
                dx = dxOdd
                dy = dyOdd
            else:
                dx = dxEven
                dy = dyEven
            for i in range(dirs):
                xdx = x + dx[i]
                ydy = y + dy[i]
                #xdx 1 for dummy col
                if not (xdx < 1 or xdx > self.x-1 or ydy < 0 or ydy > self.y-1 or closedNodes[ydy][xdx] == 1):
                    # generate a child node
                    m0 = node(xdx, ydy, n0.distance, n0.priority,self.getTile((xdx,ydy)).typ)
                    m0.nextMove()
                    m0.updatePriority(GoalPos[0],GoalPos[1])
                    if openNodes[ydy][xdx] == 0:
                        openNodes[ydy][xdx] = m0.priority
                        heappush(pq[pqi], m0)
                        # mark its parent node direction
                        dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    elif openNodes[ydy][xdx] > m0.priority:
                        # update the priority
                        openNodes[ydy][xdx] = m0.priority
                        # update the parent direction
                        dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                        # replace the node
                        # by emptying one pq to the other one
                        # except the node to be replaced will be ignored
                        # and the new node will be pushed in instead
                        while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                            heappush(pq[1 - pqi], pq[pqi][0])
                            heappop(pq[pqi])
                        heappop(pq[pqi]) # remove the target node
                        # empty the larger size priority queue to the smaller one
                        if len(pq[pqi]) > len(pq[1 - pqi]):
                            pqi = 1 - pqi
                        while len(pq[pqi]) > 0:
                            heappush(pq[1-pqi], pq[pqi][0])
                            heappop(pq[pqi])       
                        pqi = 1 - pqi
                        heappush(pq[pqi], m0) # add the better node instead
        return '' # if no route found
      
    def getFov(self,Pos,Range):
        #rather slow lot of overhead,needs to work with 120 tiles
        List = self.getTilesbyDistance(Pos,Range,minDistance=1)
        
        for element in List:
            rtrOpacity = 0
            intersected = self.intersectingline(self.getTile(Pos),self.getTile(element))
            for tile in intersected:
                if tile[0] == 1: #line did just intercept one tile so add seeing difficulty
                    rtrOpacity += self.getTile(tile[1]).opacity
                else: #two hexes intercepted, calculate mean difficulty weighted by distance
                    start = self.getTile(Pos).center
                    end = self.getTile(element).center
                    point = self.getTile(tile[1]).center
                    d1 = HexMath.distancefromLine(start[0],start[1],end[0],end[1], point[0], point[1])
                    point = self.getTile(tile[2]).center
                    d2 = HexMath.distancefromLine(start[0],start[1],end[0],end[1], point[0], point[1])
                    d1 = round((d1/float((d1+d2))),2) #percentage distance for d1
                    d2 = round(1.0 - d1,2) # precentage distance for d2
                    #weight view difficulty
                    opacity = round((self.getTile(tile[1]).opacity * d1 + self.getTile(tile[2]).opacity * d2),2)
                    rtrOpacity += opacity
            #print (element,rtrOpacity)   
             
    def getFovB(self,Pos,Range):
        #hopfully faster:)
        TotalTiles = self.getTilesbyDistance(Pos,Range)
        EndTiles = self.getTilesbyDistance(Pos,Range,minDistance=Range)
        ReturnList = []                           
        for element in EndTiles: #start from the end tiles  
            rtrOpacity = 0
            intersected = self.intersectingline(self.getTile(Pos),self.getTile(element))
            switch = True 
            for tile in intersected: 
                #as long as one tile has been used los to current tile has been calculated
                #if two tiles have been used variation is possible
                if tile[0] == 1: #line did just intercept one tile so add seeing difficulty
                    if rtrOpacity < 1:
                        rtrOpacity += self.getTile(tile[1]).opacity
                    if switch == True:
                        try:
                            TotalTiles.remove(tile[1])
                            ReturnList.append((tile[1],rtrOpacity))
                        except:
                            pass #tile already removed, do nothing 
                else:
                    switch = False
                    if rtrOpacity < 1:
                        rtrOpacity += self.InterpolateOpacity(Pos,element,tile[1],tile[2])
                    else:
                        try:
                            TotalTiles.remove(tile[1])
                            ReturnList.append((tile[1],rtrOpacity))
                        except:
                            pass #tile already removed, do nothing 
            try:
                TotalTiles.remove(tile[1])
                ReturnList.append((tile[1],rtrOpacity))
            except:
                pass #tile already removed, do nothing 
        
        for element in TotalTiles:#calculate rest
            rtrOpacity = 0
            intersected = self.intersectingline(self.getTile(Pos),self.getTile(element))
            for tile in intersected:
                if tile[0] == 1: #line did just intercept one tile so add seeing difficulty
                    if rtrOpacity < 1:
                        rtrOpacity += self.getTile(tile[1]).opacity
                else: #two hexes intercepted, calculate mean difficulty weighted by distance
                    if rtrOpacity < 1:
                        rtrOpacity += self.InterpolateOpacity(Pos,element,tile[1],tile[2])
            ReturnList.append((tile[1],rtrOpacity))
        print ReturnList   
                          
    def InterpolateOpacity(self,Start,End,Tile1,Tile2):
        start = self.getTile(Start).center
        end = self.getTile(End).center
        point = self.getTile(Tile1).center
        d1 = HexMath.distancefromLine(start[0],start[1],end[0],end[1], point[0], point[1])
        point = self.getTile(Tile2).center
        d2 = HexMath.distancefromLine(start[0],start[1],end[0],end[1], point[0], point[1])
        d1 = round((d1/float((d1+d2))),2) #percentage distance for d1
        d2 = round(1.0 - d1,2) # precentage distance for d2
        #weight view difficulty
        return round((self.getTile(Tile1).opacity * d1 + self.getTile(Tile2).opacity * d2),2)
                                                 
    def intersectingline(self,startTile,goalTile):   #returns a list of hext tiles intersected by a line 
        #still bug probably rounding problems 
        
        cur1 = startTile
        cur2 = None
        last1 = None
        last2 = None
        next1 = None
        next2 = None
        
        
        rtrList = []

        def nextHexes(next1,next2,cur,cur2):
            h = HexMath.getNeighbours(cur.x,cur.y,self.x,self.y)
            for pos in h:
                #check if hex is intersected by line,still some bugs
                tile = self.getTile(pos)
                n = HexMath.hexintersectsline(tile,startTile,goalTile)
                if n == 1 and tile != last1 and tile != last2 and tile != cur and tile!= cur2 and tile != next1 and tile!=next2:
                    if next1 == None:
                        next1 = tile
                    elif next2 == None:
                        next2 = tile
            return(next1,next2)
                    
        
                    
       
        
        while cur1 != goalTile: 
            next1 = None
            next2 = None   
            next1,next2 = nextHexes(next1,next2,cur1,cur2)
            
            if cur2 != None:
                next1,next2 = nextHexes(next1,next2,cur2,cur1)
                rtrList.append((2,(cur1.x,cur1.y),(cur2.x,cur2.y)))
            else:
                rtrList.append((1,(cur1.x,cur1.y)))
            last1 = cur1
            last2 = cur2
            cur1 = next1
            cur2 = next2
            
    
            
        rtrList.append((1,(cur1.x,cur1.y)))
        return rtrList

class Tile:
    def __init__(self,x,y,typ,map):
        self.map = map
        #array position
        self.x = x
        self.y = y
        #hex position
        hPos = HexMath.convertArrayToHex(self.x,self.y)
        self.hX = hPos[0]
        self.hY = hPos[1]
        #typ,for terrain
        self.typ = typ
        self.opacity = self.getOpacity()
        #calculate dimensions
        #calculate points
        self.center = self.setCenter()
        self.pointlist = self.setPoints()
        self.getCenterInt= (int(self.center[0]),int(self.center[1]))


    
  
    def getOpacity(self):
        if self.typ == 1:
            return 0.0
        elif self.typ == 2:
            return 0.25
        elif self.typ == 3:
            return 0.75
        else:
            return 0.0 
    def setCenter(self):#calculate the center pixel of a hex
        if self.y % 2: #ungrade
            x = int((self.x +1) * self.map.height)
        else: #grade
            x = int( self.x * self.map.height + self.map.height/2.0)
            
        y = int(self.y * self.map.side + self.map.radius)
        
        return(x,y)
    
    def setPoints(self):#calculate corner points of the hex
        #                0        |      2
        #             5 / \ 1     |    3/ \1
        #              | c |      |    | c |
        #             4 \ / 2     |    4\ /0
        #                3  (used)|      5    (clark verbrugges notation)
        
        cornerX = [self.map.height/2.0,self.map.height,self.map.height,self.map.height/2.0,0,0]
        cornerY = [0,self.map.radius/2.0,self.map.side,self.map.width,self.map.side,self.map.radius/2.0]
    
        #offset fuer ungerade 
        if self.y % 2: #ungrade
            PixelX = (self.x+1) * self.map.height - self.map.height/2.0
        else: #grade
            PixelX = self.x  * self.map.height
        
        PixelY = self.y *  self.map.side
        

        i = 0
        rtrList = []
        while i <6:
            rtrList.append((int(cornerX[i] + PixelX),cornerY[i] + PixelY))
            i += 1
        return rtrList
        
class node: #class for pathfinding
    xPos = 0
    yPos = 0
    distance = 0
    priority = 0
    def __init__(self, xPos, yPos, distance, priority,cost):
        self.xPos = xPos
        self.yPos = yPos
        self.cost = cost
        self.distance = distance
        self.priority = priority
    def __lt__(self,other):
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest,H=True):
        if H == True:
            self.priority = self.distance + self.estimate(xDest, yDest)  #*n, n >= 1 higher h -> faster search, supoptimal path
        else:
            self.priority = self.distance
            
        #possible to multiply estimate factor, for faster searching
        #but route is not optimal
                                        
    def nextMove(self):
        self.distance += self.cost
    def estimate(self, xDest, yDest):
        #convert to Hex Coordinate for easier calculation
        a = HexMath.convertArrayToHex(self.xPos,self.yPos)
        b = HexMath.convertArrayToHex(xDest,yDest)
        #calculate differenc beteween x and y values, and between does two
        dX =  b[0]-a[0]
        dY =  b[1]-a[1]
        dZ =  dY-dX
        return max((dX,dY,dZ))



