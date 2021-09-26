#!/usr/bin/env python
'''

Implementation of the Bi-Directional RRT* Algorithm

'''


# Import libraries
import sys
import pygame
from pygame import *
from pygame.locals import *
import copy
from math import *
import random
import time

# Setup environment variables
x_map = 640
y_map = 480
map_size = [x_map, y_map]

start_node_x, start_node_y = 10,300
goal_node_x, goal_node_y = 600,200

obstacle_locs = [(100,150,200,50), (100,200,20,300), (500,00,20,300)] 

# Setup RRT properties
e = 5.5 # step distance 
max_nodes = 2000 # Max number of nodes before declaring 'no-solution'
tolerance = 10 # radius of circle to search parent nodes for q_new


class Node:
    x = 0
    y = 0
    cost=0  
    parent=None

    def __init__(self,x, y):
         self.x = x
         self.y = y

def environment_setup(pygame,screen):
    '''
    Setups up environment using Pygame,
    Can visualize start node, goal node, obstacles
    '''

    # Draw map
    blue=(0,0,255)
    green = (0,255,0)
    red = (255,0,0)
    pygame.draw.circle(screen,green, (start_node_x,start_node_y), 5, 15)
    pygame.draw.circle(screen,blue, (goal_node_x, goal_node_y), 5, 15)

    for i in obstacle_locs: 
      pygame.draw.rect(screen,red,i)

def compute_euclid_dist(p1,p2):
    '''
    Given two points, this function calculates the Euclidean distance 
    between two points and returns the distance.
    '''
    
    p = (p2[0] - p1[0])
    q = (p2[1] - p1[1])

    dist = sqrt(p**2 + q**2)

    return dist

def next_step_node(p1,p2):
    '''
    Returns point p2 if distance from p1 is less than pre-defined distance e
    else returns a point in direction of p2 at a distance of e from p1
    '''

    if compute_euclid_dist(p1,p2) < e:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0]) 
        next_node_pose = p1[0] + e*cos(theta), p1[1] + e*sin(theta) 
       
        return next_node_pose

def ccw(A,B,C):
    '''
    Returns a Boolean value: True if 2 points are collision free from obstacles
                            else False
    '''
    p = (C[1]-A[1]) * (B[0]-A[0])
    q = (B[1]-A[1]) * (C[0]-A[0])
    return  p > q

def find_q_near(all_nodes, target_node):
    '''
    Finds out all nodes inside circle of radius 'tolerance' centered at
    'target_node', calculates cost of each possible parent nodes from q_init
    and calculates q_near with the lowest cost

    Returns node closest to q_rand
    '''

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
    '''
    Returns path from goal node to start node by 
    finding out parents of current node until it is the start node
    '''

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
    '''
    Calculates the final path given start, end point, and a list of all expanded nodes
    and highlights it on the pygame GUI
    '''

    yellow = (0,50,250)
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
    '''
    Extends nodes using uniform distribution and checks for collison with objects.
    If the random node is too far, it takes a step in the direction of the q_rand to get q_new

    Returns all nodes after expanding in the configuration space

    '''

    rand = Node( random.random()*x_map, random.random()*y_map)

    nn = all_nodes[0]

    for node in all_nodes:
        if compute_euclid_dist([node.x,node.y],[rand.x,rand.y]) < compute_euclid_dist([nn.x,nn.y],[rand.x,rand.y]):
            nn = node

    q_rand= next_step_node([nn.x,nn.y],[rand.x,rand.y])

    newnode = Node(q_rand[0],q_rand[1])

    if is_in_collision_w_object( nn, newnode, obstacle_locs):
        [newnode, nn] = find_parent( nn, newnode, all_nodes)
        all_nodes.append(newnode)

    pygame.draw.line(screen, black, [nn.x,nn.y], [newnode.x,newnode.y])
    pygame.display.update()

    for e in pygame.event.get():
      if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
          sys.exit("Quitting Program")


    return all_nodes

def find_parent(nn, newnode, all_nodes):
    '''
    Returns q_new and q_parent

    Finds parent node of q_new by first checking for all nodes in neighbouhood of radius 'tolerance'
    then checks for any collision of link segments 
    also checks for the cost of each node in the radius of q_new from q_init
    and chooses as the parent node with the node having lowest cost 
    '''

    for current_node in all_nodes:

        is_collision_free = is_in_collision_w_object(current_node,newnode,obstacle_locs)
        distance_compute = compute_euclid_dist([current_node.x,current_node.y],[newnode.x,newnode.y]) 

        total_cost_current_node_new_node = current_node.cost + compute_euclid_dist([current_node.x,current_node.y],[newnode.x,newnode.y])
        total_cost_nn_new_node = nn.cost + compute_euclid_dist([nn.x,nn.y],[newnode.x,newnode.y])

        if is_collision_free and distance_compute < tolerance and  total_cost_current_node_new_node < total_cost_nn_new_node:
            nn = current_node

    newnode.cost = nn.cost + compute_euclid_dist([nn.x,nn.y],[newnode.x,newnode.y])
    newnode.parent = nn
    
    return newnode,nn

def get_path(start_node, goal_node, all_nodes):
    '''
    Returns path from goal node to start node
    '''

    ret_nodes = []
    nn = all_nodes[0]
    
    for node in all_nodes:
        if compute_euclid_dist([node.x,node.y],[goal_node.x,goal_node.y]) < compute_euclid_dist([nn.x,nn.y],[goal_node.x,goal_node.y]):
            nn = node
    
    while nn != start_node:
        ret_nodes.append(nn)
        nn = nn.parent

    return ret_nodes

def is_in_collision_w_object(node_1, node_2, obstacle_locs):
    '''
    Checks if both nodes are inside the object or not
    Also checks if the line segment between node 1 and node 2
    is intersecting with the obstacle

    Returns Booolen -> True if collision free
                    --> False if collision

    '''

    A = (node_1.x , node_1.y)
    B = (node_2.x , node_2.y)

    for i in obstacle_locs:
        # Obtain the coordinates of the vertices
        obs= (i[0],i[1],i[0]+i[2],i[1]+i[3])

        C1 = (obs[0],obs[1])
        D1 = (obs[0],obs[3])

        C2 = (obs[0],obs[1])
        D2 = (obs[2],obs[1])

        C3 = (obs[2],obs[3])
        D3 = (obs[2],obs[1])

        C4 = (obs[2],obs[3])
        D4 = (obs[0],obs[3])

        # Checking for collision with objects
        check_condition_1 = ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1) 
        check_condition_2 = ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
        check_condition_3 = ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
        check_condition_4 = ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
     
        if check_condition_1 == False and check_condition_2 == False and check_condition_3 == False and check_condition_4 == False:
            continue      
        else:
            return False
    return True

def run_planner():

    # Setup up initial environment
    pygame.init()
    screen = pygame.display.set_mode(map_size)
    pygame.display.set_caption('Bi-Directional RRT*')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    environment_setup(pygame,screen)

    # Empty list that will contain nodes from q_init and q_goal
    nodes_from_root = []
    nodes_from_goal = []
    
    all_nodes = []

    # Initialized them with starting points
    nodes_from_root.append(Node(start_node_x, start_node_y)) 
    nodes_from_goal.append(Node(goal_node_x, goal_node_y)) 

    start_root=nodes_from_root[0]
    start_goal=nodes_from_goal[0]

    goal_root=Node(goal_node_x,goal_node_y)
    goal_goal=Node(start_node_x,start_node_y)

    # Initialized q_nearest
    q_nearest = None
    q_target = nodes_from_goal[0]
    
    # Initialized time at t = t0
    t0 = time.time()
    
    # Until max number of nodes is not reached, continue
    for i in range(max_nodes):
        
        #alternate between expanding from source and goal
        
        if i % 2 == 0:
            nodes_from_root = extend(nodes_from_root,screen,black)
        else:
            nodes_from_goal = extend(nodes_from_goal,screen,black)
            q_target = nodes_from_goal[-1]

        # Calculate q_near 
        q_nearest = find_q_near(nodes_from_root,q_target)
 
        # Check for all nodes within radius
        if compute_euclid_dist([q_target.x,q_target.y],[q_nearest.x,q_nearest.y]) < tolerance:

            # Checks for collision free nodes, nodes with least cost, then declares as parent node of q_new
            if is_in_collision_w_object(q_nearest,q_target,obstacle_locs):

                newnode = Node(q_target.x,q_target.y)
                newnode.parent = q_nearest
                nodes_from_root.append(newnode)
                pygame.draw.line(screen,black,[q_nearest.x,q_nearest.y],[newnode.x,newnode.y])
 
            break

    # Record time at end of loop and measure time elapsed
    t1 = time.time()
    time_elapsed = t1 - t0

    print('Max Number of nodes utilized: ',i)

    if i != max_nodes: print('Solution obtained in: ', time_elapsed,' seconds')
    else: print('Failed to obtain path')


    # Path from source and path from destination
    path_from_source = get_path(start_root, q_nearest, nodes_from_root)
    path_from_destination = get_path(start_goal, q_target, nodes_from_goal)

    # Backtrack path from destination and add it to path from source
    path_from_destination = backtrack_path(q_nearest, path_from_destination)
    path_from_destination.extend(path_from_source)
    
    # Obtain Final path and highlight it
    find_final_path(start_root, goal_root, path_from_destination, pygame, screen)

    # Update display 
    pygame.display.update()


if __name__ == '__main__':
    run_planner()
    flag = True
    while flag:
       for event in pygame.event.get():
           if event.type == pygame.QUIT:
                 flag = False



