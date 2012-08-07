#/usr/bin/env python

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
        
        #show location in the middel probeblay somehting for debug
        #pygame.draw.circle(self.mapimg,color,Tile.center,2,0)
   
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
            height = radius * math.sqrt(3)
            side = radius * 3/2
            width = radius * 2
        
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
                pointlist.append((cornerX[i] + PixelX,cornerY[i] + PixelY))
                i += 1
            pygame.draw.aalines(self.highlight, color,False, pointlist,1)
    
    def HighlightMutipleHex(self,HexList):
        self.pathfind.fill((0,0,0,0))
        color = pygame.Color(250,250,0, 250)
        for hex in HexList:
            if HexMath.checkInGrid(hex[0],hex[1],self.mMap.x,self.mMap.y):
                radius = self.mMap.radius
                height = radius * math.sqrt(3)
                side = radius * 3/2
                width = radius * 2
            
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
                    pointlist.append((cornerX[i] + PixelX,cornerY[i] + PixelY))
                    i += 1
                pygame.draw.aalines(self.highlight, color,False, pointlist,1)
        
    def init(self):
        """
        Setup the screen etc.
        """
        self.screen = pygame.display.set_mode((640, 480),1)
        pygame.display.set_caption('Press SPACE to toggle the gridRect display')
        self.mMap = HexMap.Map()
        self.mMap.LoadMap()
        self.drawMap()        
        #create transparent surface for higlight
        self.highlight = pygame.Surface((640,480),SRCALPHA)
        self.highlight.fill((0,0,0,0))
        #another surface for pathfinding visualisation
        self.pathfind = pygame.Surface((640,480),SRCALPHA)
        self.highlight.fill((0,0,0,0))
               
    def setCursor(self,x,y):
        self.HighlightHex(HexMath.ScreenToHex(x, y,self.mMap.radius))
        path = self.mMap.getPath((0,0), HexMath.ScreenToHex(x, y,self.mMap.radius))
        self.HighlightMutipleHex(path)
            
    def mainLoop(self):    
        pygame.init()    

        self.init()

        clock = pygame.time.Clock()
        x = 0             
        while 1:
            clock.tick(30)
                                                                         
            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        return                
                    elif event.key == K_SPACE:

                        #tStart = time.clock()
                        self.mMap.cache.getPath((4,0),(5,1))
                        #print time.clock() - tStart
                        
                elif event.type == MOUSEMOTION:
                    self.setCursor(event.pos[0],event.pos[1])
    
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