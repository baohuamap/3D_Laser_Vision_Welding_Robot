import socket
import time
# from cv2 import line

from setuptools import Command
# import threading
# import codecs

# Params for NX100 controller
nx100Address = "169.254.3.45"
nx100tcpPort = 80
CR = "\r"
CRLF = "\r\n"

def utf8len(inputString):
    return len(inputString.encode('utf-8'))

def command_data_length(command):
    if len(command) == 0:
        return 0
    else:
        return utf8len(command + CR)

def read_pos_from_txt(trajectory_path, pos):
	trajectory = open(trajectory_path, "r")
	for i, line in enumerate(trajectory):
		if i == pos - 1:
			command = line.rstrip()
	return command

def robot_move_to_pos(command):
	#Comm setup
	client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	client.settimeout(5)
	
	#Connect to the client/NX100 controller
	client.connect((nx100Address, nx100tcpPort))

	#START request
	startRequest = "CONNECT Robot_access" + CRLF
	client.send(startRequest.encode())
	time.sleep(0.01)
	response = client.recv(4096)      #4096: buffer size
	startResponse = repr(response)
	print(startResponse)
	
	if 'OK: NX Information Server' not in startResponse:
		client.close()
		print('[E] Command start request response to NX100 is not successful!')
		return
	
	#COMMAND request
	commandLength = command_data_length(command)
	commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVL" + " " + str(commandLength) + CRLF
	client.send(commandRequest.encode())
	time.sleep(0.01)
	response = client.recv(4096)      #4096: buffer size
	commandResponse = repr(response)
	print(commandResponse)
	
	if ('OK: ' + "MOVL" not in commandResponse):
		client.close()
		print('[E] Command request response to NX100 is not successful!')
		return
	else:
		#COMMAND DATA request
		commandDataRequest = command + (CR if len(command) > 0 else '')
		client.send(commandDataRequest.encode())
		time.sleep(0.01)
		response = client.recv(4096)
		commandDataResponse = repr(response)
		# print(commandDataResponse)
		if commandDataResponse:
			#Close socket
			client.close()
	time.sleep(0.01)
	# b = time.clock()
	# print(b-a)

def read_pos_from_robot():
	client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	client.settimeout(5)
	
	#Connect to the client/NX100 controller
	client.connect((nx100Address, nx100tcpPort))

	#START request
	startRequest = "CONNECT Robot_access" + CRLF
	client.send(startRequest.encode())
	time.sleep(0.01)
	response = client.recv(4096)      #4096: buffer size
	startResponse = repr(response)
	print(startResponse)
	
	if 'OK: NX Information Server' not in startResponse:
		client.close()
		print('[E] Command start request response to NX100 is not successful!')
		return

	#COMMAND request
	command = "1,0"
	commandLength = command_data_length(command)
	commandRequest = "HOSTCTRL_REQUEST" + " " + "RPOSC" + " " + str(commandLength) + CRLF
	client.send(commandRequest.encode())
	time.sleep(0.01)
	response = client.recv(4096)      #4096: buffer size
	commandResponse = repr(response)
	print(commandResponse)
	
	# if ('OK: ' + "MOVL" not in commandResponse):
	# 	client.close()
	# 	print('[E] Command request response to NX100 is not successful!')
	# 	return
	# else:
		#COMMAND DATA request
	commandDataRequest = command + (CR if len(command) > 0 else '')
	client.send(commandDataRequest.encode())
	time.sleep(0.01)
	response = client.recv(4096)
	commandDataResponse = repr(response)
	# print(commandDataResponse)
	if commandDataResponse:
		client.close()
	time.sleep(0.01)	

def main():
	a = read_pos_from_txt("D:/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/RobotControl/trajectory.txt", 1)
	# robot_move_to_pos(a)
	# a = read_pos_from_txt("trajectory.txt", 2)
	# robot_move_to_pos(a)
	# a = read_pos_from_txt("trajectory.txt", 3)
	# robot_move_to_pos(a)
	# print(a, type(a))
	# b = read_pos_from_txt("trajectory.txt", 2)
	# robot_move_to_pos(b)

main()