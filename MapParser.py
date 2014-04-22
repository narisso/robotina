import astar
from astar import Graph
from astar import astar
from astar import shortest_path

FILE = 'map1.map'
MOVE_COST = 1
TURN_COST = .75

walls = []
start = []
goals = set()

print 'Reading File'
f = open(FILE, 'r')


# Reading walls: [row, col, up, left, down, right]
print '\t> Parsing walls'
[MAX_ROW, MAX_COL] = f.readline().split(' ')
MAX_ROW = int(MAX_ROW)
MAX_COL = int(MAX_COL)

for i in range(0, MAX_ROW*MAX_COL):
	data = f.readline().split(' ')
	data = map(int, data)
	walls.append(data)


# Reafing starts
print '\t> Parsing ', f.readline()
MAX_START = int(f.readline())

for i in range(0, MAX_START):
	data = f.readline().split(' ')
	start.append((int(data[0]),int(data[1]),data[2][0]))


# Reading Goals
print '\t> Parsing ', f.readline()
MAX_GOALS = int(f.readline())

for i in range(0, MAX_GOALS):
	data = f.readline().split(' ')
	goals.add((int(data[0]),int(data[1]),data[2][0]))

MAX_DEPH = int(f.readline())
f.close()

print 'Generating graph'
# Generating graph
graph = Graph()

for w in walls:
	# Adding nodes to graph and edges between diferent nodes
	row = w[0]
	col = w[1]
	if w[2] == 0: # Up wall
		graph.add_node((row, col, 'u'))
		graph.add_node((row+1, col, 'u'))
		graph.add_edge((row,col,'u'),(row+1,col,'u'), MOVE_COST)
	if w[3] == 0: # Left wall
		graph.add_node((row, col, 'l'))
		graph.add_node((row, col-1, 'l'))
		graph.add_edge((row,col,'l'),(row,col-1,'l'), MOVE_COST)
	if w[4] == 0: # Down wall
		graph.add_node((row, col, 'd'))
		graph.add_node((row-1, col, 'd'))
		graph.add_edge((row,col,'d'),(row-1,col,'d'), MOVE_COST)
	if w[5] == 0: # right wall
		graph.add_node((row, col, 'r'))
		graph.add_node((row, col+1, 'r'))
		graph.add_edge((row,col,'r'),(row,col+1,'r'), MOVE_COST)

	# Adding turn edges for each node
	if (row, col, 'u') in graph.nodes:
		if (row, col, 'l') in graph.nodes:
			graph.add_edge((row,col,'u'),(row,col,'l'), TURN_COST)
		if (row, col, 'd') in graph.nodes:
			graph.add_edge((row,col,'u'),(row,col,'d'), 2*TURN_COST)
		if (row, col, 'r') in graph.nodes:
			graph.add_edge((row,col,'u'),(row,col,'r'), TURN_COST)
	if (row, col, 'l') in graph.nodes:
		if (row, col, 'd') in graph.nodes:
			graph.add_edge((row,col,'l'),(row,col,'d'), TURN_COST)
		if (row, col, 'r') in graph.nodes:
			graph.add_edge((row,col,'l'),(row,col,'r'), 2*TURN_COST)
	if (row, col, 'd') in graph.nodes:
		if (row, col, 'r') in graph.nodes:
			graph.add_edge((row,col,'d'),(row,col,'r'), TURN_COST)

# for node, edge in graph.edges.items():
# 	print '\nNode: ', node
# 	print '\tEdges: ', edge

print '==> Solving shortest path with A* Algorithm'
print '\t Start: ', start[0]
print '\t Goal:  ', goals.pop()
import math
dist = lambda c1, c2: math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)
print '\t Path:  ', shortest_path(graph, start[0], goals.pop(), dist)
