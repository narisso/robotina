import math
import MapParser
from turtle_bot import get_robot


def step(current, last, robot, has_wall, distance=1.2, angle=math.pi/2):
	turn = current[2] - last[2]
	print 'Turn: ', turn
	if turn == 0:
		if has_wall:
			robot.move_maze_wall(0.3)
		else:
			robot.move_maze_distance(0.8,0.3)
	elif turn >-3 and turn < 3:
		robot.turn_angle(turn * angle,0.4)
	elif turn == 3:
		robot.turn_angle(-angle,0.4)
	elif turn == -3:
		robot.turn_angle(angle,0.4)

robot = get_robot()
filename = "map2.map"

path, walls = MapParser.optimal_path(filename)
last_state = path[0]

#robot.turn_angle(math.pi/2,0.2) # Error overhead
#robot.wait(0.5)

for i in range(1,len(path)):
	print '\n Last state: ', last_state
	print ' Current state: ', path[i]
	
	current_walls = walls[(path[i][0],path[i][1])]
	has_wall = current_walls[path[i][2]] == 1

	print 'has_wall ' , has_wall 

	step(path[i],last_state,robot,has_wall)
	
	last_state = path[i]
	robot.wait(0.5)
	robot.play_sound(3)

robot.play_sound(5)
