#/usr/bin/env python

#test

import os, pygame,math,time
import HexMath,HexMap
from pygame.locals import *
import pygame.gfxdraw #we want to be able to use filled polys...
import cProfile

class HexagonExample:
         

    def drawHex(self,Tile):
        """
        Draw the tiles.
        """
    
        color = pygame.Color(250,250, 250, 250) # for lines
        
        bgcolor = None
        if Tile.typ == 1:
            bgcolor = pygame.Color(0,255,0,250)#green
        if Tile.typ == 2:
            bgcolor = pygame.Color(139,90,43,250)#brown
        if Tile.typ == 3:
            bgcolor = pygame.Color(184,184,184,250)#gray
            
        pygame.gfxdraw.filled_polygon(self.mapimg,Tile.pointlist,bgcolor)
        pygame.draw.aalines(self.mapimg, color,False, Tile.pointlist,1)
          
    def drawMap(self):
        self.mapimg = pygame.Surface((640,480),1)
        self.mapimg= self.mapimg.convert()
        self.mapimg.fill((104,104,104))
        iX = 1
        iY = 0      
        while iX < self.mMap.x:
            for iY in range(self.mMap.y):
                self.drawHex(self.mMap.getTile((iX,iY)))
            iX+=1
       
    def HighlightHex(self,ArrayCord): 
        #rebuild this, hex cords are now stored in tile
        #highlight the selected hex
        #clear old highlight
        self.highlight.fill((0,0,0,0))
        color = pygame.Color(0,0,250, 250)
        #check if cord is in grid
        if HexMath.checkInGrid(ArrayCord[0],ArrayCord[1],self.mMap.x,self.mMap.y):
            radius = self.mMap.radius
            height = self.mMap.height
            side = self.mMap.side
            width = self.mMap.width
        
            cornerX = [height/2,height,height,height/2,0,0]
            cornerY = [0,radius/2,side,width,side,radius/2]
        
            if ArrayCord[1] % 2: #ungrade
                PixelX = (ArrayCord[0]+1)* height -height/2
            else: #grade
                PixelX = ArrayCord[0] * height
            
            PixelY = ArrayCord[1] * side
    
            pointlist = []
            i = 0
            while i <6:
                pointlist.append((cornerX[i] + PixelX + self.mMap.offsetX,cornerY[i] + PixelY + self.mMap.offsetY))
                i += 1
            pygame.draw.aalines(self.highlight, color,False, pointlist,1)
    
    def HighlightMutipleHex(self,HexList):
        self.pathfind.fill((0,0,0,0))
        color = pygame.Color(250,250,0, 250)
        for hex in HexList:
            if HexMath.checkInGrid(hex[0],hex[1],self.mMap.x,self.mMap.y):
                radius = self.mMap.radius
                height = self.mMap.height
                side = self.mMap.side
                width = self.mMap.width
            
                cornerX = [height/2,height,height,height/2,0,0]
                cornerY = [0,radius/2,side,width,side,radius/2]
            
                if hex[1] % 2: #ungrade
                    PixelX = (hex[0]+1)* height -height/2
                else: #grade
                    PixelX = hex[0] * height
                
                PixelY = hex[1] * side
        
                pointlist = []
                i = 0
                while i <6:
                    pointlist.append((cornerX[i] + PixelX + self.mMap.offsetX,cornerY[i] + PixelY + self.mMap.offsetY))
                    i += 1
                pygame.draw.aalines(self.highlight, color,False, pointlist,1)
        
    def init(self):
        """
        Setup the screen etc.
        """
        self.screen = pygame.display.set_mode((640, 480),1)
        pygame.display.set_caption('HexEngine')
        self.mMap = HexMap.Map()
        self.mMap.LoadMap()
        self.drawMap()        
        #create transparent surface for higlight
        self.highlight = pygame.Surface((640,480),SRCALPHA)
        self.highlight.fill((0,0,0,0))
        #another surface for pathfinding visualisation
        self.pathfind = pygame.Surface((640,480),SRCALPHA)
        self.highlight.fill((0,0,0,0))
        
        self.centerScreenOnHex(5,4)
                      
    def setCursor(self,x,y):
        #addjust for offset of the hex grid
        x -= self.mMap.offsetX
        y -= self.mMap.offsetY
    
        self.HighlightHex(HexMath.ScreenToHex(x, y,self.mMap.radius))
        #set screen caption for debug purpose
        pygame.display.set_caption(str(HexMath.ScreenToHex(x,y,self.mMap.radius)))
        
        path = self.mMap.getPath((0,0), HexMath.ScreenToHex(x, y,self.mMap.radius))
        self.HighlightMutipleHex(path)
    
    def zoom(self,button,pos):
        #Calculate HexTile under MouseCursor
        posX = pos[0] - self.mMap.offsetX
        posY = pos[1] - self.mMap.offsetY
        hPos = HexMath.ScreenToHex(posX,posY,self.mMap.radius)
        
        #handles zoom button 4 is mousewheel up, button 5 is mousewheel down
        if button == 4:
            self.mMap.changeRadius(self.mMap.getRadius() + 5)
            self.drawMap()
        if button == 5:
            if self.mMap.getRadius() -5 > 0: #check so radius is bigger then 0
                self.mMap.changeRadius(self.mMap.getRadius() - 5)
                self.drawMap()
     
        #Center View on HexTile
        self.centerScreenOnHex(hPos[0], hPos[1])
            
    def scroll(self,mousePos):
        #check if mouse position is on the edge of the screen
        x = 0
        y = 0
        
        if mousePos[0] /640.0 < 0.05:
            x = 5
        if mousePos[0] /640.0 > 0.95:
            x = -5
        if mousePos[1] /480.0 < 0.05:
            y = 5
        if mousePos[1] /480.0 > 0.95:
            y = -5
        
        if x != 0 or y != 0:
            self.mMap.changeOffset((x,y))
            self.drawMap()
        
    def centerScreenOnHex(self,x,y):
        try:
            tileCenter = self.mMap.getTile((int(x),int(y))).center
        except:
            return False
        
        ScreenCenter = (320,240)
        
        xOffset = (ScreenCenter[0] - tileCenter[0]) 
        yOffset = (ScreenCenter[1] - tileCenter[1]) 
  
        self.mMap.setOffset((xOffset,yOffset))
        self.drawMap()
                     
    def mainLoop(self):    
        pygame.init()    

        self.init()

        clock = pygame.time.Clock()             
        while 1:
            clock.tick(30)
                                                                         
            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        return                
                    elif event.key == K_SPACE:
                        pass 
                elif event.type == MOUSEMOTION:
                    self.setCursor(event.pos[0],event.pos[1])
                    self.scroll(event.pos)
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 4 or event.button == 5:
                        self.zoom(event.button,event.pos)

                        
    
            # DRAWING             
            self.screen.blit(self.mapimg, (0,0)) 
            self.screen.blit(self.pathfind,(0,0))
            self.screen.blit(self.highlight,(0,0))
            pygame.display.flip()
            
def main():
    g = HexagonExample()
    g.mainLoop()

 
#this calls the 'main' function when this script is executed
#if __name__ == '__main__': main()
cProfile.run('main()')