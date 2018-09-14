'''
CF control lib

Library I created with simple functions to move the Crazyflie Drone. The function names are pretty self-explanatory.

'''


import os
import logging
import time
import _thread
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander



liftHeight = .5
lateralMisalign = 0


cflib.crtp.init_drivers(enable_debug_driver=False)
crazyflie = Crazyflie()

#Connects to CrzzyFlie Drone
def connectCF():
	

	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces(0xE7E7E7E7E9)
	print('Crazyflies found:')
	for i in available:
		print(i[0])

	

	crazyflie.open_link("radio://0/80/250K/E7E7E7E7E9")

def main():
	liftoff()
	moveHorizontal(2,4)
	land()
	

	


def liftoff():
	global liftHeight
	x = 0
	while x <= 20:
		thrust  = x * (liftHeight / 20);
		crazyflie.commander.send_hover_setpoint(0, 0, 0, thrust)
		x += 1
		time.sleep(.05)
	

def land():
	global liftHeight
	x = 0
	while x <= 15:
		thrust  = liftHeight - (x * (liftHeight / 20));
		crazyflie.commander.send_hover_setpoint(0, 0, 0, thrust)
		x += 1
		time.sleep(.05)


def moveHorizontal(horizontalDist, horizontalTime):
	global liftHeight
	global lateralMisalign
	timeIncrement = 0
	while timeIncrement < (horizontalTime * 20):
		if(lateralMisalign >= .05):
			print("readjusting")
			print(lateralMisalign)
			moveLateral(lateralMisalign, .25)
		elif(lateralMisalign <= -.05):
			print("readjusting to right")
			print(lateralMisalign)
			moveLateral(lateralMisalign, .25)
		horizontalVelocity = horizontalDist / horizontalTime
		crazyflie.commander.send_hover_setpoint(horizontalVelocity, 0, 0, liftHeight)
		timeIncrement += 1
		time.sleep(.05)
	

def setMisalign(misalign):
	global lateralMisalign
	lateralMisalign = misalign
		
def moveLateral(lateralDist, lateralTime):
	global liftHeight
	timeIncrement = 0
	while timeIncrement < (lateralTime * 20):
		lateralVelocity = lateralDist / lateralTime
		crazyflie.commander.send_hover_setpoint(lateralVelocity, 0, 0, liftHeight)
		timeIncrement += 1
		time.sleep(.05)
		




