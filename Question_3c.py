#!/usr/bin/env python

# Midterm 1 - ECE 590

# Jose Lucas Gomes Olavo 

import hubo_ach as ha
import ach
import sys
import time
import math
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# PART 1

# Change the center on mass of the robot to be on top of the right leg
mov1 = 0
while mov1 <= 0.16:
  mov1 += 0.016
	ref.ref[ha.LHR] = mov1
	ref.ref[ha.RHR] = mov1
	ref.ref[ha.LAR]= -mov1
	ref.ref[ha.RAR] = -mov1
	r.put(ref)
	#print mov1
	time.sleep(2)

time.sleep(5)

# Take the left leg out of the ground
mov2 = 0
while mov2 <= 0.8:
  mov2 += 0.1
  ref.ref[ha.LKN] = 2.0*mov2
  ref.ref[ha.LHP] = -1.0*mov2
  ref.ref[ha.LAP]= -1.0*mov2
	r.put(ref)
	#print mov2
	time.sleep(2)

time.sleep(5)

# Movement up and down 0.2m two times at 0.2Hz
amplitude = 0.2
freq = 0.2
period = 1/freq
loop_freq = 20
count = 0
ang1 = math.asin(amplitude/0.3);

while count < 40:
 	t1 = time.time()
  #print t1

  mov4 = (ang1/2)*(math.sin(math.pi/2 + count*0.1*math.pi) - 1)
  ref.ref[ha.RKN] = -1.2*mov4
  ref.ref[ha.RHP] = +0.6*mov4
 	ref.ref[ha.RAP] = +0.6*mov4
 	r.put(ref)
	 
 	t2 = time.time()
  #print t2
 	temp = period/loop_freq - (t2 - t1)
  if temp < 0:
 		temp = 0
 	time.sleep(temp)
 	count += 1

time.sleep(5)

# Put the left leg back to the ground
while mov2 >= 0.0:
	mov2 -= 0.1
  ref.ref[ha.LKN] = 2.0*mov2
  ref.ref[ha.LHP] = -1.0*mov2
  ref.ref[ha.LAP]= -1.0*mov2
	r.put(ref)
	#print mov2
	time.sleep(2)

time.sleep(5)

# Make sure some joints are zero 
ref.ref[ha.LKN] = 0.0
ref.ref[ha.LHP] = 0.0
ref.ref[ha.LAP]= 0.0
ref.ref[ha.RKN] = 0.0
ref.ref[ha.RHP] = 0.0
ref.ref[ha.RAP]= 0.0
r.put(ref)

# Change the center of mass to the center of the robot
while mov1 >= -0.0:
	mov1 -= 0.016
	ref.ref[ha.LHR] = mov1
	ref.ref[ha.RHR] = mov1
	ref.ref[ha.LAR]= -mov1
	ref.ref[ha.RAR] = -mov1
	r.put(ref)
	#print mov1
	time.sleep(2)

# Make sure some joints are zero 
ref.ref[ha.LHR] = 0.0
ref.ref[ha.RHR] = 0.0
ref.ref[ha.LAR]= 0.0
ref.ref[ha.RAR] = 0.0
ref.ref[ha.LKN] = 0.0
ref.ref[ha.LHP] = 0.0
ref.ref[ha.LAP]= 0.0
ref.ref[ha.RKN] = 0.0
ref.ref[ha.RHP] = 0.0
ref.ref[ha.RAP]= 0.0
r.put(ref)

time.sleep(20)

# PART 2

# Change the center on mass of the robot to be on top of the left leg
while mov1 >= -0.16:
	mov1 -= 0.016
	ref.ref[ha.LHR] = mov1
	ref.ref[ha.RHR] = mov1
	ref.ref[ha.LAR]= -mov1
	ref.ref[ha.RAR] = -mov1
	r.put(ref)
	#print mov1
	time.sleep(2)

# Take the right leg out of the ground
while mov2 <= 0.5:
	mov2 += 0.1
  ref.ref[ha.RKN] = 2.0*mov2
  ref.ref[ha.RHP] = -1.0*mov2
  ref.ref[ha.RAP]= -1.0*mov2
	r.put(ref)
	#print mov2
	time.sleep(2)

time.sleep(5)

# Move body to be almost parallel to the ground
mov3 = 0
while mov3 <= 0.5:
	mov3 += 0.1
  ref.ref[ha.RKN] = -1.7*mov3 + 2.0*mov2
  ref.ref[ha.RHP] = 1.7*mov3 -1.0*mov2
  ref.ref[ha.LSR] = 1.2*mov3
  ref.ref[ha.RSR] = -1.2*mov3
	ref.ref[ha.LHP] = -1.5*mov3
	r.put(ref)
	time.sleep(6)

time.sleep(5)

# Movement up and down 0.1m two times at 0.2Hz
amplitude = 0.1
freq = 0.2
period = 1/freq
loop_freq = 20
count = 0
mov4 = 0
ang1 = math.asin(amplitude/0.3);

while count < 40:	
 	t1 = time.time()
  #print t1

  mov4 = (ang1/2)*(math.sin(math.pi/2 + count*0.1*math.pi) - 1)
  ref.ref[ha.LKN] = -2.0*mov4
  ref.ref[ha.LHP] = +1.0*mov4 -1.7*mov3  	
 	ref.ref[ha.LAP] = +1.0*mov4
 	r.put(ref)
	 
 	t2 = time.time()
  #print t2	
 	temp = period/20 - (t2 - t1)
  if temp < 0:
 		temp = 0
 	time.sleep(temp)
 	count += 1

time.sleep(5)

# Take the robot out of the parallel position to the ground
while mov3 >= 0.0:
	mov3 -= 0.1
 	ref.ref[ha.RKN] = -1.7*mov3 + 2.0*mov2
 	ref.ref[ha.RHP] = 1.7*mov3 -1.0*mov2
  ref.ref[ha.LSR] = 1.2*mov3
  ref.ref[ha.RSR] = -1.2*mov3
	ref.ref[ha.LHP] = -1.5*mov3
	r.put(ref)
	time.sleep(6)

time.sleep(5)

# Put the right leg on the ground
while mov2 >= 0.0:
	mov2 -= 0.1
    	ref.ref[ha.RKN] = 2.0*mov2
    	ref.ref[ha.RHP] = -1.0*mov2
    	ref.ref[ha.RAP]= -1.0*mov2
	r.put(ref)
	print mov2
	time.sleep(6)

time.sleep(5)

# Make sure some joints are zero
ref.ref[ha.LKN] = 0.0
ref.ref[ha.LHP] = 0.0
ref.ref[ha.LAP]= 0.0
ref.ref[ha.RKN] = 0.0
ref.ref[ha.RHP] = 0.0
ref.ref[ha.RAP]= 0.0
r.put(ref)

# Change the center of mass to the center of the robot
while mov1 <= 0.0:
	mov1 += 0.016
	ref.ref[ha.LHR] = mov1
	ref.ref[ha.RHR] = mov1
	ref.ref[ha.LAR]= -mov1
	ref.ref[ha.RAR] = -mov1
	r.put(ref)
	print mov1
	time.sleep(2)


# Close the connection to the channels
r.close()
s.close()
