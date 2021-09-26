#!/usr/bin/env python
import sys
from Queue import *
from PIL import Image
import copy
from math import sqrt
import time

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file

'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing nodes to expand to in the future

# Class for queue data structure
class FooPriorityQueue(PriorityQueue):
    def put(self,node):
        node = node[0],node[1]
        PriorityQueue.put(self,node)

    def get(self):
        node = PriorityQueue.get(self)
        node = node[0],node[1]
        return node        

# Class for a* 
class a_star_search(object):
	def __init__(self, map_file, start_node, goal_node):
		self.map = map_file
		self.map = self.map.convert('1')
		self.map_size = self.map.size
		self.map = self.map.load()
		self.start_node = start_node
		self.goal_node = goal_node

		self.G = 1000000

		self.open_nodes = FooPriorityQueue(0)

		self.prev_node = {}
		self.expanded = {}
		self.frontier = {}
		self.path = []
        
		self.q = PriorityQueue()

		self.performance_data_gs = []
		self.performance_data_runtime = []
		self.performance_data_iter = []

	def heuristic_cost(self, x, y):
		(px_1, py_1) = x
		(px_2, py_2) = y

		# Euclidean distance
		dist = sqrt( (px_2 - px_1)**2 + (py_2 - py_1)**2  )
		
		return dist

	def expand_nodes(self, node):
		x = node[0]
		y = node[1]
		results = []

		x_max = self.map_size[0]
		y_max = self.map_size[1]

		if (x+1 < x_max) and (self.map[x+1,y] != (255-self.map[self.start_node[0],self.start_node[1]])): #1
			results.append((x+1,y)) 

		if (y+1 < y_max) and (self.map[x,y+1] != (255-self.map[self.start_node[0],self.start_node[1]])): #1
			results.append((x,y+1))

		if (y>=1) and self.map[x,y-1] != (255-self.map[self.start_node[0],self.start_node[1]]): #1
			results.append((x,y-1))

		if (x>=1) and self.map[x-1,y] != (255-self.map[self.start_node[0],self.start_node[1]]): #1
			results.append((x-1,y))

		return results

	def improve_solution(self):

		while not self.open_nodes.empty():
			g_s, current_node = self.open_nodes.get()
			
			if current_node == self.goal_node:
				self.G = g_s
				print('Goal Reached')
				break

			for i in self.expand_nodes(current_node):
				cost = self.expanded[current_node] + 1
				if i not in self.expanded or cost < self.expanded[i]:
					self.expanded[i] = cost
					h_cost = self.heuristic_cost(i, self.goal_node)
					total_cost = cost + h_cost
					self.open_nodes.put((cost,i))

					self.prev_node[i] = current_node


	def log_data(self, run_time, iteration):
		self.performance_data_gs.append(self.G)
		self.performance_data_runtime.append(run_time)
		self.performance_data_iter.append(iteration)

		print(" ")
		print(" ================= A* Performance Data ================= ")
		print('Run-Time: ',self.performance_data_runtime[-1] * 1000 , 'milliseconds')
		print('Iterations: ',self.performance_data_iter[-1])
		print('G cost: ',self.performance_data_gs[-1])


	def run(self):
		h_cost = self.heuristic_cost(self.start_node, self.goal_node)
		current_node = self.start_node
		self.open_nodes.put( (0 ,current_node ) )

		self.prev_node[self.start_node] = None
		self.expanded[self.start_node] = 0

		iteration = 0

		while not self.open_nodes.empty():
			iteration += 1

			t_0 = time.time()
			self.improve_solution()
			t_1 = time.time()
			run_time = t_1 - t_0
			self.log_data(run_time,iteration)

		
		current_node = self.goal_node
		while current_node != self.start_node:
			self.path.append(current_node)
			current_node = self.prev_node[current_node]
		self.path.append(self.start_node)
		self.path.reverse()

		for i in self.open_nodes.queue:
			self.frontier[i[2]] = i[1]


def search(map):
	"""
	This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
	provided map.
	:param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
	# """

	# O is unoccupied (white); 1 is occupied (black)
	print("pixel value at start point ", map[start[0], start[1]])
	print("pixel value at end point ", map[end[0], end[1]])

	# put your final path into this array (so visualize_search can draw it in purple)
	path.extend([(8,2), (8,3), (8,4), (8,5), (8,6), (8,7)])

	# put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
	expanded.update({(7,2):True, (7,3):True, (7,4):True, (7,5):True, (7,6):True, (7,7):True})

	# put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
	frontier.update({(6,2):True, (6,3):True, (6,4):True, (6,5):True, (6,6):True, (6,7):True})

	'''
	YOUR WORK HERE.
	
	I believe in you
		-Gunnar (the TA)-
	'''

    # Declare as object  
	a_star_search_algorithm = a_star_search(im, start, end)
	
    # Run algorithm
    a_star_search_algorithm.run()

	path.extend(a_star_search_algorithm.path)
	expanded.update(a_star_search_algorithm.expanded)
	frontier.update(a_star_search_algorithm.frontier)

    # Visualize path, expanded and frontier nodes
	visualize_search("out_astar.png") # see what your search has wrought (and maybe save your results)

def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()


    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # draw frontier pixels
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # display and (maybe) save results
    im.show()
    if(save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()

if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
    assert len(sys.argv) == 2, "Incorrect Number of arguments"      # require difficulty input

    print("============  A Star Implementation ============")

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print "running " + function_name + " with " + difficulty + " difficulty."

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    search(im.load())
