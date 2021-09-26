# import required libraries 
import sys
import time
import pygame
import random
from pygame.locals import KEYDOWN, K_q


'''
In order to change grid sizes, please adjust the HEIGHT WIDTH WINDOW_SIZE and MARGIN parameters

'''


# Define some colors
BLACK = (0, 0, 0) # Obstacles
WHITE = (255, 255, 255) # Unexplored nodes
GREEN = (0, 255, 0) # Explored nodes
RED = (255, 0, 0) # Goal point
BLUE = (0,0,255) # Start point
ORANGE = (255,165,0) # optimal path

# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 20
HEIGHT = 20
 
# This sets the margin between each cell
MARGIN = 5


# Set start position here
start_r = 0
start_c = 0


# Set goal position here
# Some test cases are (goal_x, goal_y): (9,9), (7,5), (8,8), (3,4), (2,9), (6,5)
goal_x = 7
goal_y = 7

reached_goal = False
visited_nodes = []

# Set obstacle locations here
obstacle_locs = [ [4,3], [4,4], [4,5], [4,6], [4,7], [3,7], [2,7], [5,2], [2,2], [2,3], [2,4], [2,5] ] 
# In case you're adding obstacle locations to this list, make sure you update the function set_obstacles with those grid
# locations so that obstacle cells are filled with that colour and is easy to comprehend visually.


# Initialize pygame
pygame.init()
 
# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [255, 255]
screen = pygame.display.set_mode(WINDOW_SIZE)

total_grid_size = 10

grid = []
for row in range(total_grid_size):
    # Add an empty array that will hold each cell in this row
    grid.append([])
    for column in range(total_grid_size):
        grid[row].append(0)  # Append a cell

# Checks for 'quit' event
def checkEvents():
    # will continously monitor if 'Q' was pressed for Quitting the program
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
            sys.exit()
        elif event.type == KEYDOWN and event.key == K_q:
            pygame.quit()
            sys.exit()

# Sets the environment - lattice graph       
def setup_grid():
    screen.fill(BLACK)
    set_goal_point(goal_x,goal_y)  
    set_start_point(start_r,start_c)              
    set_obstacles()

    # Draw the grid
    for row in range(total_grid_size):
        for column in range(total_grid_size):
            color = WHITE
            if grid[row][column] == 1:
                color = GREEN

            elif grid[row][column] == 2:
                color = BLUE

            elif grid[row][column] == 3:
                color = RED
            
            elif grid[row][column] == 4:
                color = BLACK

            elif grid[row][column] == 5:
                color = ORANGE
                
            pygame.draw.rect(screen, color, [(MARGIN + WIDTH) * column + MARGIN, (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])

# Computes shortest distance
def compute_shortest_distance_path(visited_nodes):
    path_nodes = [[start_r,start_c]]

    while euclidean_distance(path_nodes[-1][0], path_nodes[-1][1], goal_x, goal_y) > 1:
        temp_list = []
        # To obtain all the neighbouring nodes
        for nodes in visited_nodes:
            if euclidean_distance(path_nodes[-1][0], path_nodes[-1][1], nodes[0],nodes[1] ) == 1: 
                if nodes not in path_nodes:
                    temp_list.append(nodes)
                
        # Find the node which is closest to the goal and then add it to path nodes
        nodes_dist_list = []
        for neighbour_node in temp_list:
            dist_to_goal = euclidean_distance(neighbour_node[0], neighbour_node[1], goal_x, goal_y)
            nodes_dist_list.append(dist_to_goal)
        
        
        closest_node_to_goal_index = nodes_dist_list.index(min(nodes_dist_list))
        closest_node_to_goal = temp_list[closest_node_to_goal_index] 
        
        path_nodes.append(closest_node_to_goal)
    
    
    return path_nodes

# Euclidean distance
def euclidean_distance(x1,y1,x2,y2):
    dist = ( (x2 - x1)**2 +  (y2 - y1)**2  )**0.5
    return abs(dist)


def main():

    # Initializes GUI for the grid
    pygame.init()
    pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("BFS Path Planning")

    dr = [-1,1,0,0]
    dc = [0,0,1,-1]

    reached_goal = False

    visited_nodes.append([start_r,start_c])
    
    iteration = 0

    while True:
        checkEvents()                
        setup_grid()

        '''
        Until the goal is not reached, the start node will explore its valid neighbours,
        which inturn will explore their valid neighbours until one of the neighbour is the goal node
        '''

        if reached_goal!= True:
            for [r,c] in visited_nodes:
                iteration +=  1    
                
                if r == goal_x and c == goal_y:
                    reached_goal = True
                    break

                for i in range(4):
                    rr = r + dr[i]
                    cc = c + dc[i]
                    
                    if rr < 0 or cc < 0: continue
                    if rr > total_grid_size-1 or cc > total_grid_size-1: continue

                    if [rr,cc] in visited_nodes: continue
                    if [rr,cc] in obstacle_locs: continue

                
                    visited_nodes.append([rr,cc])

                    # Fills colour for better visual comprehension
                    fill_cell(rr,cc)
        else:
            # Computes shortest/optimal path from all the nodes explored
            shortest_path = compute_shortest_distance_path(visited_nodes)            
            for node in shortest_path:
                fill_cell_path(node[0],node[1])


            print('-----------------------------')
            print("GOAL STATUS: SUCCESSFULLY REACHED")
            print('Iterations: ',iteration) 
            print('Visited Nodes: ',visited_nodes)         
            print('Shortest Path: ', shortest_path)
            print('-----------------------------')
            print('Please press Q to quit the program')


        # Continoulsy updates the GUI grid
        pygame.display.update()
        pygame.display.flip()     
                

# Fills a particular row,column pair - GREEN
def fill_cell(r,c):
    grid[r][c] = 1            

# Fills a particular row,column pair - BLUE
def set_start_point(r,c):
    grid[r][c] = 2            

# Fills a particular row,column pair - RED
def set_goal_point(r,c):
    grid[r][c] = 3            

# Fills a particular row,column pair - BLACK
def set_obstacles():
    grid[4][3] = 4            
    grid[4][4] = 4            
    grid[4][5] = 4 
    grid[4][6] = 4
    grid[4][7] = 4
    
    grid[3][7] = 4
    
    grid[2][7] = 4
    
    grid[5][2] = 4
    grid[2][2] = 4
    grid[2][3] = 4
    grid[2][4] = 4
    grid[2][5] = 4



[4,3], [4,4], [4,5], [4,6], [4,7], [3,7], [2,7], [5,2], [2,2], [2,3], [2,4], [2,5]

# Fills a particular row,column pair - ORANGE
def fill_cell_path(r,c):
    grid[r][c] = 5            


if __name__ == '__main__':
    main()

