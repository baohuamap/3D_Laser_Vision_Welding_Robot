class Position:
	def __init__(self, speed, coordinate, x, y, z, rotx, roty, rotz, tool):
		self.speed = speed
		self.coordinate = coordinate
		self.x = x
		self.y = y
		self.z = z
		self.rotx = rotx
		self.roty = roty
		self.rotz = rotz
		self.tool = tool

# class Trajectory:


def read_position():
	x = input("Enter x: ") + "\n"
	y = input("Enter y: ") + "\n"
	z = input("Enter z: ") + "\n"
	rotx = input("Enter rotx: ") + "\n"
	roty = input("Enter roty: ") + "\n"
	rotz = input("Enter rotz: ") + "\n"
	pos = Position(50, 3, x, y, z, rotx, roty, rotz, 3)
	return pos

def read_positions():
	positions = []
	total_pos =  int(input("Enter how many positions: "))
	for i in range(total_pos):
		print("--- Enter position ", i+1, " ---")
		pos = read_position()
		positions.append(pos)
	return positions

def write_postion_txt(pos, f):
	command = "0" + 
	pass

def write_positions_txt(positions):
	total = len(positions)
	for i in range(int(total)):
		
	# with open("data.txt", "w") as f:
		# f.write()
		
		# write_videos_txt(playlist.videos, f)
	pass