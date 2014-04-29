import math
from astar import Graph
from astar import shortest_path

def optimal_path(file_name):
	MOVE_COST = 1
	TURN_COST = .75

	walls = []
	start = []
	goals = set()

	print 'Reading File'
	f = open(file_name, 'r')


	# Reading walls: [row, col, up, left, down, right]
	print '\t> Parsing walls'
	[MAX_ROW, MAX_COL] = f.readline().split(' ')
	MAX_ROW = int(MAX_ROW)
	MAX_COL = int(MAX_COL)

	for i in range(0, MAX_ROW*MAX_COL):		
		data = f.readline().split(' ')
		print 'data: ', data
		data = map(int, data)
		walls.append(data)


	# Reafing starts
	print '\t> Parsing ', f.readline()
	MAX_START = int(f.readline())

	for i in range(0, MAX_START):
		data = f.readline().split(' ')
		if data[2][0] == 'u':
			orientation = 0
		elif data[2][0] == 'l':
			orientation = 1
		elif data[2][0] == 'd':
			orientation = 2
		else:
			orientation = 3
		start.append((int(data[0]),int(data[1]),orientation))


	# Reading Goals
	graph = Graph()
	print '\t> Parsing ', f.readline()
	MAX_GOALS = int(f.readline())

	for i in range(0, MAX_GOALS):
		data = f.readline().split(' ')
		row = int(data[0])
		col = int(data[1])
		graph.add_node((row,col,0))
		graph.add_node((row,col,1))
		graph.add_node((row,col,2))
		graph.add_node((row,col,3))

		goals.add((row,col,0))
		goals.add((row,col,1))
		goals.add((row,col,2))
		goals.add((row,col,3))

	MAX_DEPH = int(f.readline())
	f.close()

	print 'Generating graph'
	# Generating graph

	for w in walls:
		# Adding nodes to graph and edges between diferent nodes
		row = w[0]
		col = w[1]
		if w[2] == 0: # Up wall
			graph.add_node((row, col, 0))
			graph.add_node((row+1, col, 0))
			graph.add_edge((row,col,0),(row+1,col,0), MOVE_COST)
		if w[3] == 0: # Left wall
			graph.add_node((row, col, 1))
			graph.add_node((row, col-1, 1))
			graph.add_edge((row,col,1),(row,col-1,1), MOVE_COST)
		if w[4] == 0: # Down wall
			graph.add_node((row, col, 2))
			graph.add_node((row-1, col, 2))
			graph.add_edge((row,col,2),(row-1,col,2), MOVE_COST)
		if w[5] == 0: # right wall
			graph.add_node((row, col, 3))
			graph.add_node((row, col+1, 3))
			graph.add_edge((row,col,3),(row,col+1,3), MOVE_COST)

		# Adding turn edges for each node
		if (row, col, 0) in graph.nodes:
			if (row, col, 1) in graph.nodes:
				graph.add_edge((row,col,0),(row,col,1), TURN_COST)
			if (row, col, 2) in graph.nodes:
				graph.add_edge((row,col,0),(row,col,2), 2*TURN_COST)
			if (row, col, 3) in graph.nodes:
				graph.add_edge((row,col,0),(row,col,3), TURN_COST)
		if (row, col, 1) in graph.nodes:
			if (row, col, 2) in graph.nodes:
				graph.add_edge((row,col,1),(row,col,2), TURN_COST)
			if (row, col, 3) in graph.nodes:
				graph.add_edge((row,col,1),(row,col,3), 2*TURN_COST)
		if (row, col, 2) in graph.nodes:
			if (row, col, 3) in graph.nodes:
				graph.add_edge((row,col,2),(row,col,3), TURN_COST)

	print '==> Solving shortest path with A* Algorithm'
	
	goal = goals.pop()

	print '\t Start: ', start[0]
	print '\t Goal:  ', goal

	for key in graph.edges.iterkeys():
		print 'Node: ', key

	dist = lambda c1, c2: math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)
	path = shortest_path(graph, start[0], goal, dist)
	print '\t Path:  ', path
	return path
