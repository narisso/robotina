import math
import MapParser
from turtle_bot_original import get_robot


def step(current, last, robot, distance=1.2, angle=math.pi/2):
	turn = current[2] - last[2]
	print 'Turn: ', turn
	if turn == 0:
		robot.move_distance(distance)
	elif turn >0 and turn < 3:
		robot.turn_angle(turn * angle)
	elif turn == 3:
		robot.turn_angle(-angle)
	elif turn == -3:
		robot.turn_angle(angle)


robot = get_robot()
filename = "../maze2.txt"

path = MapParser.optimal_path(filename)
last_state = path[0]

#robot.turn_angle(math.pi/2) # Error overhead
#robot.wait(0.5)
for i in range(1,len(path)):
	print '\n Last state: ', last_state
	print ' Current state: ', path[i]
	step(path[i],last_state,robot)
	last_state = path[i]
	robot.wait(0.5)
