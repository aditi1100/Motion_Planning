#!/usr/bin/env python

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from shapely.geometry import Polygon, Point
import time

#constants
width = 1280
height = 720
WINSIZE = [width, height]
#small value taken to show smoother motion
threshold =10.0
#No of nodes                                       
k = 10000 
 

tree = {}
pathList=[]
#initialize and prepare screen
pygame.init()
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('ARENA_1 ')
white = 255, 255, 255
black = 0, 0, 0
red=255,0,0
blue= 0,0,255
screen.fill(white)
#use of functions in pygame to draw obstacles
#in pygame.draw.rect ,a vertex and width and height are passed
obstacleRect = pygame.draw.rect(screen, black, [780, 250, 160, 100])
obstacleTr = pygame.draw.polygon(screen, black, [[200,30],[200,200],[250,200]])


listOfObstacle= []
listOfObstacle.append(Polygon([(780, 250), (780, 350), (940, 350), (940, 250)]))
listOfObstacle.append(Polygon([(200,30),(200,200),(250,200)]))

pygame.display.update()

rectangle = [(780,250),(940,250),(940,350),(780,350)]
triangle = [(200,30),(200,200),(250,200)]

#use of a generator
def pairs(lst):
    i = iter(lst)
    first = prev = i.next()
    for item in i:
        yield prev, item
        prev = item
    yield item, first

#the intersection of a line with an edge of a polygon is computed
#function modified from https://gist.github.com/scw/2838603
def polyLineInt(from_point, to_polygon):
    nearest_point = None
    min_dist = float(sys.maxint)
#use of Point from shapely
    for seg_start, seg_end in pairs(list(to_polygon.exterior.coords)[:-1]):
        line_start = Point(seg_start)
        line_end = Point(seg_end)
        intersection_point = intersect_point_to_line(Point(from_point), line_start, line_end)
        cur_dist =  distance1(Point(from_point), intersection_point)
        if cur_dist < min_dist:
            min_dist = cur_dist
            nearest_point = intersection_point
    return nearest_point
#distance between two points using object
def distance1(point1, point2):
    vect_x = point2.x - point1.x
    vect_y = point2.y - point1.y
    return math.sqrt(vect_x**2 + vect_y**2)

#intersection of a point on a line
#checks if line may be parallel

def intersect_point_to_line(point, line_start, line_end):
    line_magnitude =  distance1(line_end, line_start)
    slope = ((point.x - line_start.x) * (line_end.x - line_start.x) + \
         (point.y - line_start.y) * (line_end.y - line_start.y)) \
         / (line_magnitude ** 2)
    intersect_point = None
    #corner cases that intersection pt may be vertex
    if slope < 0.00001 or slope > 1:
        ix = distance1(point, line_start)
        iy = distance1(point, line_end)
        if ix > iy:
            intersect_point = line_end
        else:
            intersect_point = line_start
    else:
        ix = line_start.x + slope * (line_end.x - line_start.x)
        iy = line_start.y + slope * (line_end.y - line_start.y)
        intersect_point = Point([ix, iy])
    return intersect_point
#calculating distance between a point and a polygon
def dist_calc(from_point, to_polygon):
    to_point = polyLineInt(from_point, to_polygon)
    to_point = Point(to_point)
    return dist(from_point, (to_point.x,to_point.y))

#distance when vertices are specified in a list
def dist(point1,point2):
    return sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))

#finding the random node
def step_from_to(point1,point2):
    if dist(point1,point2) < threshold:
        return point2
    else:
        theta = atan2(point2[1]-point1[1],point2[0]-point1[0])
        return point1[0] + threshold*cos(theta), point1[1] + threshold*sin(theta)

def main():
    
    nodes = []
    #append start position in list of nodes
    nodes.append(pathList[0])

    tree[pathList[0]]={None : 0}
    for i in range(k):
        rand = random.random()*1280.0, random.random()*720.0
	#initially tree is empty so nearest node is start itself
        nn = nodes[0]
        #find the nearest node in among current nodes
        for p in nodes:
            if dist(p,rand) < dist(nn,rand):
                nn = p
        newnode = step_from_to(nn,rand)

        #caluate distance from newnode to obstacle
        #if distance is less than 20, skip the node since now we may exceed clearance for the obstacle
        distt = dist_calc(newnode, Polygon([(780, 250), (780, 350), (940, 350), (940, 250)]))
	#check clearance as well as if the newnode is generated inside the obstacle
        if distt < 20 or obstacleRect.collidepoint(newnode):
            continue

        Tdistt = dist_calc(newnode, Polygon([(200,30),(200,200),(250,200)]))
        if Tdistt < 20 or obstacleTr.collidepoint(newnode):
            continue

	nodes.append(newnode)	
	tree[newnode]={nn:dist(nn,newnode)}
	pygame.draw.line(screen,black,nn,newnode)
        pygame.display.update()
	
	nn=nodes[1]
	for p in nodes:
	    	if dist(p,pathList[1]) < dist(nn,pathList[1]):
			if dist(p,pathList[1]) < 20:
				nodes.append(pathList[1])
				newnode=pathList[1]
				tree[newnode]={p:dist(p,newnode)}
				pygame.draw.line(screen,red,p,newnode)
		        	pygame.display.update()
				break
	  		else:
				nn=p

    parentNodes = [pathList[1]]
    #draw the robot and give its dimensions
    robot = pygame.draw.rect(screen,red,(pathList[0][0],pathList[0][1], 10, 10))
    while parentNodes[-1] != pathList[0]:
        pre=tree[parentNodes[-1]]
        tup=pre.keys()
        parentNodes.append(tup[0])

    parentNodes.reverse()
    for path in parentNodes:
        robot.x = path[0]
        robot.y = path[1]
        pygame.draw.rect(screen, red, robot)
        #pygame.draw.line(screen, blue, parentNodes[-1], path[0])
        time.sleep(0.25)
        pygame.display.update()


if __name__ == '__main__':
    initialPos=input('Enter intial position:')
    pathList.append(initialPos)
    goalPos=input('Enter goal position:')
    pathList.append(goalPos)
    running = True		
    main()
    while(running):
		for event in pygame.event.get():
			if (event.type == pygame.QUIT):
				running = False
    pygame.quit()


input('press ENTER to exit')
