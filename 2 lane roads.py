# -*- coding: utf-8 -*-
"""
Created on Mon Sep 18 18:09:58 2017

@author: zacth
"""

import pygame

import random
simulation = False
population = 0.0
h=30
class Car:
    def __init__(self,start,end,grid,road):
        self.carx = -1
        self.cary = end
        self.grid = grid
        self.Road = road
        self.new = None
        self.old = None
        self.now = None
        hue = random.randint(50,254)
        self.color =  [254-hue,hue,130-int(0.5*(hue))]
        self.traffic = False

    def update(self,i):
        self.grid= grid
        self.grid = grid
        if self.grid[self.carx][self.cary] != 0:
            self.grid[self.carx][self.cary] = 2
        if self.new == None:
            n = 1+i*3
            while self.grid[self.Road.vertices[n].x][self.Road.vertices[n].y] == 3:
                n = random.randint(1,len(self.Road.vertices))
            if n != len(self.Road.vertices):
                p = 0
                if p != len(self.Road.vertices[n].edges):
                    self.carx = self.Road.vertices[n].x
                    self.cary = self.Road.vertices[n].y
                    self.old = self.Road.vertices[n]
                    self.new = self.Road.vertices[n].edges[p]
                    self.grid[self.carx][self.cary] = 3
        else: 
            self.now = self.new
            self.carx = self.now.x
            self.cary = self.now.y
            if len(self.now.edges)>1:
                self.now.edges.remove(self.old)
                p = random.randint(0,len(self.now.edges)-1)
                if self.grid[self.now.edges[p].x][self.now.edges[p].y] == 3:
                    self.now.edges.append(self.old)
                    self.new =self.now
                    self.grid[self.carx][self.cary] = 3
                elif self.now.traffic is not True and (((self.now.edges[p].timer < 100 or self.now.edges[p].timer >=185)and self.now.edges[p].timer != -1 and self.now.x == self.now.edges[p].x) or (self.now.edges[p].timer >= 85 and self.now.edges[p].timer != -1 and self.now.y == self.now.edges[p].y)):
                    self.now.edges.append(self.old)
                    self.new = self.now
                    self.grid[self.carx][self.cary] = 3
                else:
                    self.new = self.now.edges[p]
                    self.now.edges.append(self.old)
                    self.old = self.now
                    self.grid[self.carx][self.cary] = 3
            else:
                self.new = self.now
        if self.grid[self.carx][self.cary] == 2:
            self.grid[self.carx][self.cary] = 3
        return self.grid
        
        
class vertex:
    def __init__(self, edges: list, weights: list, x, y):
        self.edges = edges
        self.weights = weights
        self.x = x
        self.y = y
        self.distance = 2 ** 20
        self.previous = None
        self.traffic = False
        self.timer = -1

    def addvertex(self, newvertex: "vertex", weight):
        if newvertex not in newvertex.edges:
            newvertex.edges.append(self)
            newvertex.weights.append(weight)
            return


class edge:
    def __init__(self, start: vertex, end: vertex, weight):
        self.start = start
        self.end = end
        self.weight = weight


class Graph:
    vertices = []
    edges = []

    def __init__(self, vertex: vertex):
        self.vertices.append(vertex)
    def check(self, vertex: vertex):
        for node in self.vertices:
            if node.x == vertex.x and node.y == vertex.y:
                return False
            else:
                return True
    def addVertices(self, node: vertex):
        if node not in self.vertices:
            self.vertices.append(node)
            for i in range(0, len(node.edges)):
                node.addvertex(node.edges[i], node.weights[i])
            return
    def lights(self):
        for node in self.vertices:
            if  len(node.edges) > 2 and not node.traffic:
                node.traffic = True
    def change(self):
        for node in self.vertices:
            if node.traffic:
                node.timer += 1
                node.timer = node.timer%200
    def update(self):
        self.lights()
        self.change()
    def edgelist(self, repeats):
        self.edges = []
        for i in range(0, len(self.vertices)):
            for j in range(0, len(self.vertices[i].edges)):
                arc = edge(self.vertices[i], self.vertices[i].edges[j], self.vertices[i].weights[j])
                self.edges.append(arc)
        if repeats is False:
            for k in range(0, int((len(self.edges)) / 2)):
                side = self.edges.pop(k)
                found = False
                for p in self.edges:
                    if p.start == side.end and p.end == side.start:
                        found = True
                if found:
                    continue
                else:
                    self.edges.insert(k, side)

    def dijkstra(self, source: vertex, target=None):
        self.edgelist(False)
        for node in self.vertices:
            node.distance = 2 ** 32
            node.previous = None
        self.vertices[self.vertices.index(source)].distance = 0
        q = []
        for i in range(0, len(self.vertices)):
            q.append(self.vertices[i])
        while len(q) > 0:
            node = q[0]
            for point in q:
                if point.distance < node.distance:
                    node = point
            node = q.pop(q.index(node))
            if node == target:
                pattern = []
                u = target
                while u.previous is not None:
                    pattern.append(u)
                    u = u.previous
                pattern.append(source)
                return pattern
            for k in range(0, len(node.edges)):
                if node.edges[k] in q:
                    maximum = 0
                    for l in range(0, len(self.edges)):
                        if self.edges[l].start == node or (self.edges[l].end == node):
                            if self.edges[l].start == node.edges[k] or self.edges[l].end == node.edges[k]:
                                maximum = self.edges[l].weight
                    alt = node.distance + maximum
                    if alt < node.edges[k].distance:
                        node.edges[k].distance = alt
                        node.edges[k].previous = node




BLACK = (0, 0, 0)
WHITE = (200, 200, 200)
GREEN = (0, 255, 0)
GREY = (100,100,100)
BLUE = (255,0,0)
WIDTH = 16
HEIGHT = 10
MARGIN = 1
grid = []
for row in range(100):
    grid.append([])
    for column in range(100):
        grid[row].append(0)
pygame.init()
WINDOW_SIZE = [1600, 1000]
screen = pygame.display.set_mode(WINDOW_SIZE)


pygame.display.set_caption("Traffic Simulator")


done = False


clock = pygame.time.Clock()
carx = 0
cary = 0
road = []
end = [-1,-2]
current = [-1,-1]
started = False
new = None
old = None
global occupied
occupied = []
now= None
n = 0
carq = True
for i in range(100):
    occupied.append([])
    for k in range(100):
        occupied[i].append(False)
while not done:
    screen.fill([60,80,110])
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True  
        
        
        elif event.type == pygame.MOUSEBUTTONDOWN and current != end:
            started =True
            pos = pygame.mouse.get_pos()
            current = [pos[1]// (HEIGHT + MARGIN),pos[0]// (WIDTH + MARGIN)]
            end = current
            column = pos[0] // (WIDTH + MARGIN)
            row = pos[1] // (HEIGHT + MARGIN)
            grid[row][column] = 2 
        
        
        elif event.type == pygame.MOUSEBUTTONDOWN and current == end:
            flag = True
            pos = pygame.mouse.get_pos()
            end = [pos[1]// (HEIGHT + MARGIN),pos[0]// (WIDTH + MARGIN)]
            column = pos[0] // (WIDTH + MARGIN)
            row = pos[1] // (HEIGHT + MARGIN)
            if grid[row][column] != 0:
                differencer = current[0] - end[0]
                differencec = current[1] - end[1]
                if differencer == 0 and differencec**2 > 0:
                    if differencec < 0:
                        column = -1
                        row = 0
                    else:
                        column =1
                        row = 0
                elif differencec == 0 and differencer**2 > 0:            
                    if differencer < 0:
                        row = -1
                        column = 0
                    else:
                        row = 1
                        column = 0
                else:
                    current = end
            
            for i in range((differencer)*row+1):
                for j in range((differencec)*column+1):
                            posr = current[0]+(i)*-row
                            posc = current[1]+(j)*-column
                            grid[posr][current[1]+(j)*-column]=2
                            if [posr,posc] not in road:
                                road.append([posr,posc])
                                if len(road) == 1:
                                    start = vertex( [] , [] , road[0][0], road[0][1])
                                    Road= Graph(start)
                                
            for k in range(100):
                for l in range(100):
                    if grid[k][l] == 1:
                        grid[k][l] = 0
            current = end
            pos = pygame.mouse.get_pos()
            current = [pos[1]// (HEIGHT + MARGIN),pos[0]// (WIDTH + MARGIN)]
            end = current
            column = pos[0] // (WIDTH + MARGIN)
            row = pos[1] // (HEIGHT + MARGIN)
            grid[row][column] = 2
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            pos = pygame.mouse.get_pos()
            end = [pos[1]// (HEIGHT + MARGIN),pos[0]// (WIDTH + MARGIN)]
            column = pos[0] // (WIDTH + MARGIN)
            row = pos[1] // (HEIGHT + MARGIN)
            if grid[row][column] != 0:
                differencer = current[0] - end[0]
                differencec = current[1] - end[1]
                if differencer == 0 and differencec**2 > 0:
                    if differencec < 0:
                        column = -1
                        row = 0
                    else:
                        column =1
                        row = 0
                elif differencec == 0 and differencer**2 > 0:            
                    if differencer < 0:
                        row = -1
                        column = 0
                    else:
                        row = 1
                        column = 0
                else:
                    current = end
                                
            for k in range(100):
                for l in range(100):
                    if grid[k][l] == 1:
                        grid[k][l] = 0
            current = [-1,-1]
    
    
    pressed = pygame.key.get_pressed()
    if pressed[pygame.K_SPACE]:
        population += 0.25
        simulation =True
    elif pressed[pygame.K_UP]:
        h += 0.5
    elif pressed[pygame.K_DOWN]:
        h -= 0.5
            
    if len(road) > 1:
        occupied[road[0][0]][road[0][1]] = True    
        for section in road:
            if occupied[section[0]][section[1]] is not True:
                links = []
                weights= []
                try:
                    if occupied[section[0]-1][section[1]]:
                        for point in Road.vertices:
                            if point.x == section[0]-1 and point.y == section[1]:
                                links.append(point)
                except:
                    print()
                try:
                    if occupied[section[0]][section[1]-1]:
                        for point in Road.vertices:
                            if point.x == section[0] and point.y == section[1]-1:
                                links.append(point)
                except:
                    print()
                try:
                    if occupied[section[0]+1][section[1]]:
                        for point in Road.vertices:
                            if point.x == section[0]+1 and point.y == section[1]:
                                links.append(point)
                except:
                    print()
                try:
                    if occupied[section[0]][section[1]+1]:
                        for point in Road.vertices:
                            if point.x == section[0] and point.y == section[1]+1:
                                links.append(point)
                except:
                    print()
                occupied[section[0]][section[1]] = True
                for n in range(len(links)):
                    weights.append(0)
                final = vertex(links,weights,section[0],section[1])
                Road.addVertices(final)        
                       

    if len(road) > 1 and simulation:
        if carq:
            cars = []
            for i in range(1):
                car = Car(i,Road.vertices[0].y,grid,Road)
                cars.append(car)

            carq = False
        else: 
            for i in range(int(population)-len(cars)):
                car = Car(i,Road.vertices[0].y,grid,Road)
                cars.append(car)
            for i in range(int(population)):
                grid = cars[i].update(i)
            Road.update()


    if current != [-1,-1]:
            for i in range(100):
                    if grid[current[0]][i] != 3:
                        grid[row][i]=1
                        pygame.draw.rect(screen,
                                        GREEN,
                                        [(WIDTH + MARGIN) * i + MARGIN,
                                        (HEIGHT + MARGIN) * current[0] + MARGIN,
                                        WIDTH,
                                        HEIGHT])
            for i in range(100):
                if grid[i][current[1]] != 3:
                    grid[i][column]=1
                    pygame.draw.rect(screen,
                                    GREEN,
                                    [(WIDTH + MARGIN) * current[1] + MARGIN,
                                    (HEIGHT + MARGIN) * i + MARGIN,
                                    WIDTH,
                                    HEIGHT])

    for track in Graph.vertices:
        pygame.draw.rect(screen,
                         [0,0,0],
                         [(WIDTH + MARGIN) * track.y + MARGIN,
                          (HEIGHT + MARGIN) * track.x + MARGIN,
                          WIDTH,
                          HEIGHT])
        if track.traffic:
            if track.timer <85:
                pygame.draw.rect(screen,
                                 [255, 0, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN+WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN+HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 0, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [0, 255, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN + WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [0, 255, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN + HEIGHT,
                                  WIDTH,
                                  HEIGHT])
            elif track.timer <100 or track.timer>=185:
                pygame.draw.rect(screen,
                                 [255, 100, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN+WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN+HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 100, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 100, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN + WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 100, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN + HEIGHT,
                                  WIDTH,
                                  HEIGHT])
            else:
                pygame.draw.rect(screen,
                                 [0, 255, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN + WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN + HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [0, 255, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 0, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN + WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN - HEIGHT,
                                  WIDTH,
                                  HEIGHT])
                pygame.draw.rect(screen,
                                 [255, 0, 0],
                                 [(WIDTH + MARGIN) * track.y + MARGIN - WIDTH,
                                  (HEIGHT + MARGIN) * track.x + MARGIN + HEIGHT,
                                  WIDTH,
                                  HEIGHT])

    if not carq:
        for vehicle in cars:
            if vehicle.cary != -1:
                pygame.draw.rect(screen,
                             vehicle.color,
                             [(WIDTH + MARGIN) * vehicle.cary + MARGIN,
                              (HEIGHT + MARGIN) * vehicle.carx + MARGIN,
                              WIDTH,
                              HEIGHT])
        
    clock.tick(int(h))

    pygame.display.flip()


pygame.quit()
