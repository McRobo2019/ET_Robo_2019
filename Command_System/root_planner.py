import networkx as nx
import pickle
import math

map_block_dict = {
	0: {'pos': ( 350, 1400), 'connections': [1, 4, 7]}, 
	1: {'pos': ( 700, 1400), 'connections': [0, 2, 4, 5, 8]}, 
	2: {'pos': (1050, 1400), 'connections': [1, 3, 5, 6, 9]}, 
	3: {'pos': (1400, 1400), 'connections': [2, 6, 10]}, 

	4: {'pos': ( 525, 1225), 'connections': [0, 1, 7, 8]}, 
	5: {'pos': ( 875, 1225), 'connections': [1, 2, 8, 9]}, 
	6: {'pos': (1225, 1225), 'connections': [2, 3, 9, 10]}, 

	7:  {'pos': ( 350, 1050), 'connections': [0, 4, 8, 11, 13]}, 
	8:  {'pos': ( 700, 1050), 'connections': [1, 4, 5,  7,  9, 11, 14]}, 
	9:  {'pos': (1050, 1050), 'connections': [2, 5, 6,  8, 10, 12, 15]}, 
	10: {'pos': (1400, 1050), 'connections': [3, 6, 9, 12, 16]}, 

	11: {'pos': ( 525, 875), 'connections': [7,  8, 13, 14]}, 
	12: {'pos': (1225, 875), 'connections': [9, 10, 15, 16]}, 

	13: {'pos': ( 350, 700), 'connections': [ 7, 11, 14, 17, 20]}, 
	14: {'pos': ( 700, 700), 'connections': [ 8, 11, 13, 15, 17, 18, 21]}, 
	15: {'pos': (1050, 700), 'connections': [ 9, 12, 14, 16, 18, 19, 22]}, 
	16: {'pos': (1400, 700), 'connections': [10, 12, 15, 19, 23]}, 

	17: {'pos': ( 525, 525), 'connections': [13, 14, 20, 21]}, 
	18: {'pos': ( 875, 525), 'connections': [14, 15, 21, 22]}, 
	19: {'pos': (1225, 525), 'connections': [15, 16, 22, 23]}, 

	20: {'pos': ( 350, 350), 'connections': [13, 17, 21]}, 
	21: {'pos': ( 700, 350), 'connections': [14, 17, 18, 20, 22]}, 
	22: {'pos': (1050, 350), 'connections': [15, 18, 19, 21, 23]}, 
	23: {'pos': (1400, 350), 'connections': [16, 19, 22]}, 

}

class Map:
	def __init__(self, G):
		self._graph = G
		self.intersections = nx.get_node_attributes(G, "pos")
		self.roads = [list(G[node]) for node in G.nodes()]

	def save(self, filename):
		with open(filename, 'wb') as f:
			pickle.dump(self._graph, f)

def load_map_graph(map_dict):
	G = nx.Graph()
	for node in map_dict.keys():
		G.add_node(node, pos=map_dict[node]['pos'])
	for node in map_dict.keys():
		for con_node in map_dict[node]['connections']:
			G.add_edge(node, con_node)
	return G

def load_map_block():
	G = load_map_graph(map_block_dict)
	return Map(G)


#Functions for Root Planner
def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated
    return set()

def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    open_set = set()
    if self.start != None:
#01291029#        open_set.add(self.start)
        open_set = {self.start}

#        for node in self.map.roads[self.start]:
#            open_set.add(node)

    return open_set

    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    came_from_dic = {}
#    for open_node in self.openSet:
#        came_from_dic[open_node] = self.start
    
    return came_from_dic


def create_gScore(self):
    """Creates and returns a data structure that holds the cost of getting from the start node to that node, 
    for each node. The cost of going from start to start is zero."""
    gscore_dic = {}
#    for open_node in self.openSet:
#        gscore_dic[open_node] = self.distance(self.start, open_node)

#for node in range(len(self.map.intersections)):
#        gscore_dic[node] = float('inf')

#01292019
    gscore_dic = dict.fromkeys(self.map.intersections.keys(),float('inf'))
    gscore_dic[self.start] = 0
    return gscore_dic

    
def create_fScore(self):
    """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    fscore_dic = {}
#    for open_node in self.openSet:
#        h = self.distance(open_node, self.goal)
#        fscore_dic[open_node] = h + self.gScore[open_node]
    
#    for node in range(len(self.map.intersections)):
#        fscore_dic[node] = float('inf')

#01292019        
    fscore_dic = dict.fromkeys(self.map.intersections.keys(),float('inf'))
    fscore_dic[self.start] = self.distance(self.start, self.goal)
    return fscore_dic

def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    self.map = M

def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    self.start = start

def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    self.goal = goal

def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    if len(self.openSet) == 0:
        return True
    else:
        return False

def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    return min(self.openSet, key=self.fScore.get)

def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    neighbors = []
    for nei_node in self.map.roads[node]:
        neighbors.append(nei_node)
        if not nei_node in self.cameFrom:
            self.cameFrom[nei_node] = node
    return neighbors

def get_gScore(self, node):
    """Returns the g Score of a node"""
    # TODO: Return the g Score of a node
    return self.gScore[node]

def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    # TODO: Compute and return the Euclidean L2 Distance
    x1 = self.map.intersections[node_1][0]
    y1 = self.map.intersections[node_1][1]
    x2 = self.map.intersections[node_2][0]
    y2 = self.map.intersections[node_2][1]
    l2_dis = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    return l2_dis

def get_tentative_gScore(self, current, neighbor):
    """Returns the tentative g Score of a node"""
    return self.gScore[current] + self.distance(current, neighbor)

def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    return self.distance(node, self.goal)

def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    # REMEMBER F = G + H
    return self.gScore[node] + self.heuristic_cost_estimate(node)

def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    # TODO: Record the best path to a node, by updating cameFrom, gScore, and fScore
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
    self.fScore[neighbor] = self.calculate_fscore(neighbor)

# Do not change this cell
# When you write your methods correctly this cell will execute
# without problems
class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
#        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        if current == self.start:
            return total_path

        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
            if current == self.start:
                return total_path
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False


PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to
