#matplotlib update image

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

import serial
import time

import Queue
import threading

queue = Queue.Queue(10000)

try:
	serA = serial.Serial('COM5', 9600,timeout=1)
	serA.write("%d"%(1))
	time.sleep(5)	
except:
	print "BT problem"

# create the figure

fig = plt.figure()
ax = fig.add_subplot(111)
im = plt.imshow(np.zeros((5,7)),animated=True)
plt.style.use('classic')
#plt.show(block=False)

def serial_read(s):
    while 1:
		try:
			global line
			line = s.readline()
			queue.put(line)
			print line
		except:
			pass

def operations():
	while 1:
		try:
			
			electrodes = line
			
			#electrodes = str(input("Enter electrode status: "))
			electrodes = ['0' if x == '2' else x for x in electrodes]
			#print (electrodes)
			lines = electrodes[0:5]
			columns = electrodes[5:12]
			
			lines = ['0' if x == '2' else x for x in lines]
			columns = ['0' if x == '2' else x for x in columns]
			
			#print lines
			#print columns
			
					
			# #print press1
			# #print press2
					
			touch = np.empty((5,7))
			
			lines = map(int,lines)
			columns = map(int,columns)
			
			for i in range(len(lines)):
				for j in range(len(columns)):
					#touch[i,j] = lines[i]*columns[j]
					if lines[i] and columns[j]:
						touch[i,j] = 1
					else:
						touch[i,j] = -1
			
			press1 = line[12:16]
			press2 = line[16:20]
			
			press1 = int(str(press1))
			press2 = int(str(press2))
			#press1 = 512
			#press2 = 1023
			
			press1 = translate(press1,0,700,0,1.2)
			press2 = translate(press2,0,700,0,1.2)
			
			#print press1
			#print press2

			#touch = [-1 if x == 0 else 0 for x in lines]
			#print touch
			#print pressMask
			#pressTouch = np.multiply(touch,pressMask)
			
			pressMatrix1 = press1*pressMask[:,0:4]
			#print pressMatrix1
			
			
			pressMatrix2 = press2*pressMask[:,4:7]
			#print pressMatrix2
			
			pressMatrix = np.column_stack((pressMatrix1,pressMatrix2))
			#print pressMatrix
			
			global pressTouch
			pressTouch = touch + pressMatrix
			#print pressTouch
					
			#print touch
			
			
		except:
			print "error in operations"

#pressMask = np.matrix('1 1 1 1 1 1 1; 1.1 1.25 1.1 1 1.1 1.25 1.1; 1.25 2 1.25 1 1.25 2 1.25; 1.1 1.25 1.1 1 1.1 1.25 1.1; 1 1 1 1 1 1 1')
pressMask = np.matrix('0 0 0 0 0 0 0; 0.1 0.25 0.1 0 0.1 0.25 0.1; 0.25 1 0.25 0 0.25 1 0.25; 0.1 0.25 0.1 0 0.1 0.25 0.1; 0 0 0 0 0 0 0')

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)       
        
threadA = threading.Thread(target=serial_read, args=(serA,),).start()
threadB = threading.Thread(target=operations, args=(),).start()

def updateFig(*args):
		
		#print line
		# buf = serA.readline()
		# print buf

				

		
		#ax.clear()
		#ax.plot(touch)	
		


		im = plt.imshow(pressTouch,cmap='RdBu_r',vmin=-1, vmax=2,interpolation='Gaussian', aspect='auto')
		#im = plt.imshow(touch)
		#plt.pause(0.001)
		#fig.canvas.flush_events()
		return im,
		
		
		
		fig.canvas.flush_events()
		#time.sleep(0.1)
		

ani = animation.FuncAnimation(fig, updateFig, interval=0.0001,blit=True)
# figManager = plt.get_current_fig_manager() 
# figManager.full_screen_toggle() 
plt.show()
	
	
	
