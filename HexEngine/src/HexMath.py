'''
Created on 16.07.2012
Fundamental functions for hex stuff 
(for now just for flat side from left to right 
and second line offset to the right)

@author: benjamin
'''
import math
import time



def convertArrayToHex(x,y):
    returnX = int(x - math.floor(y/2.0))
    returnY = int(x + math.ceil(y/2.0))
    return(returnX,returnY)

def convertHexToArray(x,y):
    returnX = int(math.floor((x+y)/2.0))
    returnY = int(y-x)
    return(returnX,returnY)

def getNeighboursB(pos,map):
    aX = pos[0]
    aY = pos[1]
    gridX = map.x
    gridY = map.y
    

    return getNeighbours(aX,aY,gridX,gridY)

def getNeighbours(aX,aY,gridX,gridY):
    #get neigbouring hexes
    if aY % 2:
        list= ((aX,aY-1),(aX+1,aY-1),(aX+1,aY),(aX+1,aY+1),(aX,aY+1),(aX-1,aY))
    else:
        list= ((aX-1,aY-1),(aX,aY-1),(aX+1,aY),(aX,aY+1),(aX-1,aY+1),(aX-1,aY))
    return removeOutsideofGrid(list,gridX,gridY)

def getDistance(aX,aY,bX,bY):
    #convert to Hex Coordinate for easier calculation
    a = convertArrayToHex(aX,aY)
    b = convertArrayToHex(bX,bY)
    #calculate differenc beteween x and y values, and between does two
    dX =  b[0]-a[0]
    dY =  b[1]-a[1]
    dZ =  dY-dX
    return max(abs(dX),abs(dY),abs(dZ))

def ScreenToHex(x,y,radius): 
    #convert screen coordinates to hex coordinates, used for mouse picking
    height = radius * math.sqrt(3)
    side = radius * 3/2
    #ruff position
    cY = math.floor(y/side)     
    #draw debug rectancle with ofset for ven and od
    #offset for even an odd
    if cY % 2:
        cX = math.floor((x+height-height/2)/height)
        #test if cursor is over the target hex x = y|0.5 - x/heigt|
        if y- cY*side < radius * math.fabs(0.5-(x - (cX*height-height/2))/height):
            cY-=1
            if (x + height-height/2) - cX*height > height/2:
                cX+=1
        cX-=1
    else:
        cX = math.floor(x/height)    
        if y- cY*side < radius * math.fabs(0.5-(x - cX*height)/height):
            cY-=1
            if x-cX*height < height/2:
                cX-=1    
    return(cX,cY)

def getTopLeft(aX,aY,radius):
    #gets the top left pixel of a hex
    height = radius * math.sqrt(3)
    side = radius * 3/2
    if aY % 2: #ungrade
        x = (aX +1) * height -height/2
    else: #grade
        x = aX * height
    y = aY * side
    return(int(x),int(y))
       
def removeOutsideofGrid(HexList,gridX,gridY):
    #removes hexs not inside the grid from a given list
    outlist = []
    for element in HexList:
        #1 for dummy hex
        if element[0] >= 0 and element[0] < gridX and element[1] >= 0 and element[1]<gridY:
            outlist.append((element[0],element[1]))
    return outlist

def checkInGrid(aX,aY,gridX,gridY):#check if tile is in grid
    #1 for dummy hex
    if aX >= 1 and aX < gridX and aY >= 0 and aY<gridY:
        return True
    else:
        return False 

def sign(x):
    #returns signium - 1 if x < 0, 1 if x > 0 , and 0 if 0
    return (x > 0) - (x < 0)

def HexSpaceLineOfSight(startPos,goalPos):# a line of hex from Start to End, for now dX>dy
    def process(x,y):
        global factor
        global xone
        global yone
        global RtrList
        RtrList.append((x,y))
        if x != goalPos[0] or y != goalPos[1]:
            factor += dy
            if factor >= dx:
                factor -= dx
                if sig:
                    x += xone
                    y += yone
                else:
                    x += xone
                    process(x,y)
                    y += yone
            else:
                x += xone
            process(x,y)
        return (factor,x,y)    

            
    dx = goalPos[0] - startPos[0]
    dy = goalPos[1] - startPos[1]
    sig = (sign(dx) == sign(dy))
    global xone
    global yone
    global RtrList 
    RtrList =[]
    if dx< 0:
        xone = -1
    else:
        xone = 1
    
    if dy < 0:
        yone = -1
    else:
        yone = 1
        
    if dx %2:
        dx *=2
        dy *=2
        
    dx = math.fabs(dx)
    dy = math.fabs(dy)
    
    global factor 
    factor = dx/2
    x = startPos[0]
    y = startPos[1]


            
    process(x,y)
    
    return RtrList

def distancefromLine(ax,ay,bx,by,px,py):#gets distance of a point from a line
    normalLenght = math.sqrt((bx-ax)*(bx-ax)+(by-ay)*(by-ay))
    return int(abs((px-ax)*(by-ay)-(py-ay)*(bx-ax))/ normalLenght)
    
def turns(x0,y0,x1,y1,x2,y2):#check if a point is left or right of a line
    cross = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0)
    if cross > 0:
        return 1#left
    elif cross == 0:
        return 0#straight
    else:
        return -1#right

def hexintersectsInfinitline(tile,startTile,endTile):
    #hex is intersected if it lies on the line
    #or one of its points lies on the opposite side as side1   
    side1 = turns(startTile.center[0],startTile.center[1],
                  endTile.center[0],endTile.center[1],
                  tile.center[0],tile.center[1])
    print "------"
    print startTile.center
    print endTile.center
    print tile.center[0],tile.center[1]
    print "--------"
    print ("side1:",side1)
    if side1 == 0:
        return 1 # hex is straight on the line
    for side in tile.pointlist:
        j = turns(startTile.center[0],startTile.center[1],
                  endTile.center[0],endTile.center[1],
                  side[0],side[1]) 
        print side[0],side[1],j
        if j == 0 or j!= side1:
            return 1
    return 0
    
def hexintersectsline(tile,startTile,endTile):
    #hex is intersected if it lies on the line
    #or one of its points lies on the opposite side as side1 
    #ignore points outside startTile.center -> goalTile.center
    #construct bounding rectangel
    x1 = 0
    x2 = 0
    y1 = 0
    y2 = 0
    if startTile.center[0] < endTile.center[0]:
        x1 = startTile.center[0]
        x2 = endTile.center[0]
    elif startTile.center > endTile.center[0]:
        x1 = endTile.center[0]
        x2 = startTile.center[0]
        
    if startTile.center[1] < endTile.center[1]:
        y1 = startTile.center[1]
        y2 = endTile.center[1]
    elif startTile.center > endTile.center[1]:
        y1 = endTile.center[1]
        y2 = startTile.center[1]
             
    side1 = turns(startTile.center[0],startTile.center[1],endTile.center[0],endTile.center[1],tile.center[0],tile.center[1])

    
    if tile.center[0] >= x1 and tile.center[0] <= x2 and tile.center[1] >= y1 and tile.center[1] <= y2: 
        if side1 == 0:
            return 1 # hex is straight on the line
        
    for side in tile.pointlist:
        if side[0] >= x1 and side[0] <= x2 and side[1] >= y1 and side[1] <= y2:
            j = turns(startTile.center[0],startTile.center[1],
                  endTile.center[0],endTile.center[1],
                  side[0],side[1])  
            if j == 0 or j!= side1:
                return 1
    return 0
    

