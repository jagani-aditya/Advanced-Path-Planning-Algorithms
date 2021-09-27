# Bi-Directional RRT* 

import sys
import pygame
from pygame import *
from pygame.locals import *
import copy
from math import *
import random

# Insert start and goal node in global
# make a wall map
# color start and goal nodes blue and red
x_map = 640
y_map = 480
map_size = [x_map, y_map]
e = 2.5
max_nodes = 20000
tolerance = 5
obstacle_locs = [(500,150,100,50),(300,80,100,50),(150,220,100,50), (70,20,20,100)]

class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord

def environment_setup(pygame,screen):
    blue=(0,0,255)
    green=(0,255,0)
    red=(255,0,0)
    
    for o in obstacle_locs: 
      pygame.draw.rect(screen,blue,o)
      pygame.draw.circle(screen, green, [0,0], 15, 50)
      pygame.draw.circle(screen, red, [640, 480], 15, 50)
      pygame.display.update()

def compute_euclid_dist(p1,p2):
    p = (p2[0] - p1[0])
    q = (p2[1] - p1[1])

    dist = sqrt(p**2 + q**2)

    return dist

def next_step_node(p1,p2):
    if compute_euclid_dist(p1,p2) < e:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + e*cos(theta), p1[1] + e*sin(theta)

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])


def find_q_near(all_nodes, target_node):
    q_near = all_nodes[0]
    ccost = 100000
    neighbouring_nodes = []

    for current_node in all_nodes:
        p1 = ( current_node.x, current_node.y)
        p2 = ( target_node.x, target_node.y )
        
        if compute_euclid_dist(p1,p2) < tolerance:
            neighbouring_nodes.append(current_node)
    
    for current_node in neighbouring_nodes:
        if current_node.cost < ccost:
            q_near = copy.deepcopy(current_node)
            ccost = current_node.cost

    return q_near

def backtrack_path(parent, path_nodes):
    backtrack_nodes = []
    current_node = path_nodes[0]
    current_node_parent = parent

    while current_node != None:
        next_node = Node(current_node.x, current_node.y)
        next_node.parent = current_node_parent
        current_node = current_node.parent
        current_node_parent = next_node
        backtrack_nodes.append(next_node)

    return backtrack_nodes

def find_final_path(source_node, goal_node, all_nodes, pygame,screen):
    yellow = 0,255,255
    new_node = all_nodes[0]

    for current_node in all_nodes:
        d1 = compute_euclid_dist( [current_node.x, current_node.y ], [goal_node.x, goal_node.y ] ) 
        d2 = compute_euclid_dist( [new_node.x, new_node.y ] , [goal_node.x, goal_node.y ] )
        
        if d1 < d2:
            new_node = current_node
    
    while (new_node.x, new_node.y)!= (source_node.x, source_node.y):
        pygame.draw.line(screen,yellow,[new_node.x,new_node.y],[new_node.parent.x,new_node.parent.y],5)  
        new_node = new_node.parent

def extend(all_nodes,screen,black):
    rand = Node(random.random()*x_map, random.random()*y_map)
    nn = all_nodes[0]
    for p in all_nodes:
        if compute_euclid_dist([p.x,p.y],[rand.x,rand.y]) < compute_euclid_dist([nn.x,nn.y],[rand.x,rand.y]):
            nn = p

    interpolatedNode= next_step_node([nn.x,nn.y],[rand.x,rand.y])

    newnode = Node(interpolatedNode[0],interpolatedNode[1])

    if is_in_collision_w_object(nn,newnode,obstacle_locs):
        [newnode,nn] = find_parent(nn,newnode,all_nodes)
        all_nodes.append(newnode)

    pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
    pygame.display.update()

    for e in pygame.event.get():
      if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
          sys.exit("Exiting.")


    return all_nodes

def find_parent(nn,newnode,all_nodes):
    for p in all_nodes:
        if is_in_collision_w_object(p,newnode,obstacle_locs) and compute_euclid_dist([p.x,p.y],[newnode.x,newnode.y]) < tolerance and p.cost + compute_euclid_dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+compute_euclid_dist([nn.x,nn.y],[newnode.x,newnode.y]):
            nn = p
    newnode.cost=nn.cost+compute_euclid_dist([nn.x,nn.y],[newnode.x,newnode.y])
    newnode.parent=nn
    
    return newnode,nn

def get_path(start,goal,all_nodes):
    ret_nodes = []
    nn = all_nodes[0]
    for p in all_nodes:
        if compute_euclid_dist([p.x,p.y],[goal.x,goal.y]) < compute_euclid_dist([nn.x,nn.y],[goal.x,goal.y]):
            nn = p
    while nn!=start:
        ret_nodes.append(nn)
        nn=nn.parent

    return ret_nodes

def is_in_collision_w_object(nodeA,nodeB,OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])

      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])

      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])

      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])

      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1) 
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        continue      
      else:
         return False
    return True


def main():
    pygame.init()
    screen = pygame.display.set_mode(map_size)
    pygame.display.set_caption('Bi-Directional RRT*')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    environment_setup(pygame,screen)

    nodes_from_root = []
    nodes_from_goal = []
    
    all_nodes = []
    nodes_from_root.append(Node(0.0,0.0)) # Start in the corner
    nodes_from_goal.append(Node(630.0,470.0)) # Start in the corner

    start_root=nodes_from_root[0]
    start_goal=nodes_from_goal[0]

    goal_root=Node(630.0,470.0)
    goal_goal=Node(0.0,0.0)

    q_nearest = None
    q_target = nodes_from_goal[0]

    for i in range(max_nodes):
        if(i%2):
            nodes_from_root = extend(nodes_from_root,screen,black)
        else:
            nodes_from_goal = extend(nodes_from_goal,screen,black)
            q_target = nodes_from_goal[-1]

        q_nearest = find_q_near(nodes_from_root,q_target)
        if(compute_euclid_dist([q_target.x,q_target.y],[q_nearest.x,q_nearest.y])<tolerance):
            if is_in_collision_w_object(q_nearest,q_target,obstacle_locs):
                newnode = Node(q_target.x,q_target.y)
                newnode.parent = q_nearest
                nodes_from_root.append(newnode)
                pygame.draw.line(screen,black,[q_nearest.x,q_nearest.y],[newnode.x,newnode.y])
            break

    pppath = get_path(start_root,q_nearest,nodes_from_root)
    ppath = get_path(start_goal,q_target,nodes_from_goal)
    ppath = backtrack_path(q_nearest,ppath)
    ppath.reverse()
    ppath.extend(pppath)
    
    find_final_path(start_root,goal_root,ppath,pygame,screen)

    pygame.display.update()



if __name__ == '__main__':
    main()
    running = True
    while running:
       for event in pygame.event.get():
           if event.type == pygame.QUIT:
                 running = False



