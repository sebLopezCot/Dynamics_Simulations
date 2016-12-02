
# Author: Sebastian Lopez-Cot
# 12/2/2016
# This code simulates a mass-spring system with an optional dampener

from numpy import sin, cos, sqrt, exp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class MassSpringSystem:
	""" Mass Spring System

	init_state is [y, doty] in meters, where y is the position
	below the anchored point, doty is the velocity, and ddoty is the
	acceleration.

	"""

	def __init__(self,
				init_conditions = [1.0, 0],
				M = 1.0, 	# mass
				K = 100.0, 	# spring constant
				L0 = 1.0, 	# unstreched length
				C = 5.0, # damping coeff
				Z = 0.0, # damping ratio
				G = 9.81,	# accel due to gravity
				origin=(0,0)):
		self.init_conditions = init_conditions
		self.params = (M, K, L0, C, Z, G)
		self.origin = origin
		self.time_elapsed = 0

	def position(self):
		""" compute the current y position of the spring mass system """
		(M, K, L0, C, Z, G) = self.params

		fund_freq = sqrt(K/M)
		Z = C / (2.0 * M * fund_freq) if Z == 0 else Z	# damping ratio
		t = self.time_elapsed
		y0, v0 = self.init_conditions

		c1 = y0
		c2 = (v0 + Z * y0) / sqrt(1 - Z**2)

		y =  exp(-Z*t) * (c1 * cos(fund_freq * t)  + c2 * sin(fund_freq * t)) - L0 - (M*G/K)

		return ((0,0),(0,y))

	def step(self, dt):
		""" execute one time step of length dt and update state"""
		self.time_elapsed += dt


#-----------------------------------------------------------------
# set up initial state and global variables
mass_spring = MassSpringSystem([1.0, 0])
dt = 1./30 # 30 fps

#-----------------------------------------------------------------
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
						xlim=(-2,2), ylim=(-4,0))
ax.grid()

line, = ax.plot([],[], 'o-', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
position_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)

def init():
	""" initialize animation """
	line.set_data([],[])
	time_text.set_text('')
	position_text.set_text('')
	return line, time_text, position_text

def animate(i):
	""" perform animation step """
	global mass_spring, dt
	mass_spring.step(dt)

	line.set_data(*mass_spring.position())
	time_text.set_text('time = %.1f' % mass_spring.time_elapsed)
	position_text.set_text('position = %.3f m' % mass_spring.position()[1][1])
	return line, time_text, position_text

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000*dt - (t1-t0)

ani = animation.FuncAnimation(fig, animate, frames=300,
								interval=interval, blit=True, init_func=init)

plt.show()
