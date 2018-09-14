'''
Abdullah Akbar and Ben Clayman   5/18/2018
"Drone Tracking"
Information Engineering                        St. Mark's School of Texas
Code Abstract: Demonstrates a drone tracking program using OpenCV. When 'capture' is typed into the console, capture the origin point as the center of the drone.
Then, as the drone flies, draws a rectangle and line realtime between the drone's origin point and its current center point.


Source(s): Code adapted from Andrew Smith's 'blob tracker'
https://gist.github.com/Andrew-William-Smith/837d2de7c5903dc3f59d6b6ad08890ce




This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License v3 as published by the Free Software Foundation


'''

import cv2
import numpy
import _thread
import os
import time
from cfControlLib import connectCF, liftoff, moveHorizontal, moveLateral, land,setMisalign


# Colours to be detected
lowerBound = numpy.array([120,  60,  30], dtype=numpy.uint8)
upperBound = numpy.array([255, 170, 100], dtype=numpy.uint8)
envelope   = 40
gFrame	 = None
#Video Input Device ID
cameraID = 0
theInput = ""

#Boolean to determine whether program loops should keep going or not
keepProgramAlive = True

#Meters per pixel displayed
conversionFactor = .00265
connectCF()

#misalign 
global misalignX
misalignX = 0

#Variables for origin point of tracking line and rectangle
originX,originY
OriginIsSet = False
ShouldSetOrigin = False

#Array of numbers that hold the drone's current center point and its origin point
global currentXYarray, originXYarray

#command to determine if drone should be flown (if launchCMD should run)
shouldLaunch = False


#Lifts off the drone, fly the drone 1 meter in 4 seconds, then lands the drone
def launchCMD():
	global shouldLaunch
	while keepProgramAlive:
		if shouldLaunch:
			liftoff()
			moveHorizontal(1,4)
			
			land()
			shouldLaunch = False
	
	
#Gets input from console and sets the variable theInput with that value
def inputCmd():
	global theInput
	global keepProgramAlive
	while keepProgramAlive:
		os.system("cls")
		theInput = input("Enter Command: ").lower()
		time.sleep(.5)
	

#Starts the inputCMD and launchCMD threads
_thread.start_new_thread( inputCmd,())
_thread.start_new_thread( launchCMD,())



def uintAddAbsolute(val, addend):
	"""Add a value to a numpy.uint8, without wrapping on over/underflow."""
	return (val.astype(numpy.int16) + addend).clip(0, 255).astype(numpy.uint8)


#When mouse is clicked, selects the contour of the same color surrounding the point clicked
def recalibrateColour(evt, x, y, flags, param):
	global lowerBound, upperBound,ShouldSetOrigin,lineArray

	
	if evt == cv2.EVENT_LBUTTONUP:
		hVal, sVal, vVal = gFrame[y, x]
		lowerBound = numpy.array([uintAddAbsolute(hVal, -envelope),
								  uintAddAbsolute(sVal, -envelope),
								  uintAddAbsolute(vVal, -envelope)],
								 dtype=numpy.uint8)
		upperBound = numpy.array([uintAddAbsolute(hVal,  envelope),
								  uintAddAbsolute(sVal,  envelope),
								  uintAddAbsolute(vVal,  envelope)],
								 dtype=numpy.uint8)
		print(lowerBound, upperBound)
	elif evt == cv2.EVENT_MBUTTONUP:
		lowerBound = gFrame[y, x]
		print(lowerBound, upperBound)
	elif evt == cv2.EVENT_RBUTTONUP:
		upperBound = gFrame[y, x]
		print(lowerBound, upperBound)
	return


	#main thread
if __name__ == '__main__':
	global originXYarray, currentXYarray
	camera	  = cv2.VideoCapture(cameraID)
	kernelOpen  = numpy.ones((5, 5))
	kernelClose = numpy.ones((20, 20))

	cv2.namedWindow('camera')
	cv2.setMouseCallback('camera', recalibrateColour)

	while keepProgramAlive:
		
		#Reads the frames incoming from the camera, resizes them, and then creates contours of each polygon of the selected color
		frameRead, frame = camera.read()
		frame	 = cv2.resize(frame, (1366, 768))
		frame	 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		gFrame	= frame.copy()
		mask	  = cv2.inRange(frame, lowerBound, upperBound)
		maskOpen  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
		maskClose = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelClose)
		_, contours, _ = cv2.findContours(maskClose.copy(), cv2.RETR_EXTERNAL,
										  cv2.CHAIN_APPROX_NONE)

		maxArea = 0
		largestContour = None
		#Iterates through all the contours and selects the contour with the largest area to set as largestContour
		for contour in contours:
			area = cv2.contourArea(contour)
			if area > maxArea:
				maxArea = area
				largestContour = contour

		if largestContour is not None and len(largestContour) > 0:
			cv2.drawContours(frame, [largestContour], -1, (125, 250, 130), 3)
			
			#Gets the image moments of the largest contour, given that the largest contour has an area greater than 0
			M = cv2.moments(largestContour)
			
			#Calculates the centroid (center point) of the contour
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			
			#Gets x,y coordinates of the bounding rect around the contour (top left corner)
			x, y, w, h = cv2.boundingRect(largestContour)
			
			#xLength and yLength for 
			xLength = 2 * (cx - x)
			yLength = 2 * (cy - y)
			
			#If loop to determine whether the origin point should be set; if it should, set a center point as the centroid of the largest contour. If not, get the center point of the largestContour as the current point
			if (OriginIsSet == False) and (ShouldSetOrigin == True):
				print("set Array")
				originXYarray = numpy.array([cx,cy])
				currentXYarray = numpy.array([cx,cy])
				OriginIsSet = True
				ShouldSetOrigin = False
			elif (OriginIsSet == False) and (ShouldSetOrigin == False):
				1 + 1
			else:
				currentXYarray = numpy.array([cx,cy])
				#Display the rectange and line between the current point and the origin point
				cv2.rectangle(frame, (originXYarray[0], originXYarray[1]), (currentXYarray[0], currentXYarray[1]),(0, 220, 13), 2)
				cv2.line(frame, (originXYarray[0], originXYarray[1]), (currentXYarray[0], currentXYarray[1]),(0, 220, 13), 2)
				misalign = currentXYarray[0] - originXYarray[0]
				#if the misalign between the origin and the current point is greater than 5 cm, set misalign to that
				if(misalign >= (.05 / conversionFactor)):
					misalignX = misalign * conversionFactor
					setMisalign(misalignX)
				elif(misalign <= (-1 * .05 / conversionFactor)):
					misalignX = misalign * conversionFactor
					setMisalign(misalignX)
				else:
					misalignX = 0
  
			
		frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)

		cv2.imshow('camera', frame)
		cv2.waitKey(10)
		
		#When 'launch' is typed into the console, set the shouldLaunch to true, trigger the launchCMD function
		if theInput == "launch":
		
			shouldLaunch = True
			
			theInput = ""
		#When 'kill' is typed into the console, end the program by setting keepProgramAlive to false
		elif theInput == "kill":
			print("kill cmd triggered")
			keepProgramAlive = False
			theInput = ""
			quit()
		#When 'capture' is typed into the console, set the booleans to capture a new origin point
		elif theInput == "capture":
			OriginIsSet = False
			ShouldSetOrigin = True
			theInput = ""
		
		
	quit()
	cv2.destroyAllWindows()