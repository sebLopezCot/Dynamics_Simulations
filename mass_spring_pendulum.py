
# Author: Sebastian Lopez-Cot
# 12/2/2016
# This code simulates a pendulum where the arm of the pendulum
# is a spring.

from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

class MassSpringPendulum:
	""" Mass spring pendulum 

	init_state is [r, dotr, theta, dottheta] in meters, m/s, degrees, degrees/s,
	where r is the linear displacement of the pendulum arm from static equilibrium, 
	dotr is the rate of change of r, theta is the anglular position of the mass from 
	the vertical, and dottheta is the rate of change of theta.

	"""

	def __init__(self,
				init_state = [1.0, 0, 60.0, 0],
				M = 1.0,	# mass
				K = 15.0, 	# spring constant
				L0 = 1.0, 	# unstreched length
				G = 9.81, 	# accel due to gravity
				origin=(0,0)):
		self.init_state = np.asarray(init_state, dtype='float')
		self.params = (M, K, L0, G)
		self.origin = origin
		self.time_elapsed = 0

		self.state = self.init_state
		self.state[2] *= np.pi / 180.0
		self.state[3] *= np.pi / 180.0


	def position(self):
		"""compute the current x,y position of the mass"""
		(M, K, L0, G) = self.params
		(r, dotr, theta, dottheta) = self.state

		x = np.cumsum([self.origin[0], (L0 + r) * sin(theta)])
		y = np.cumsum([self.origin[0], -(L0 + r) * cos(theta)])

		return (x,y)

	def dstate_dt(self, state, t):
		""" compute the derivative of the given state"""
		(M, K, L0, G) = self.params
		(r, dotr, theta, dottheta) = state

		deriv = np.zeros_like(state)
		deriv[0] = dotr
		deriv[2] = dottheta

		deriv[1] = (L0 + r) * dottheta**2 + G*cos(theta) - K*r/M

		deriv[3] = -1.0 * (2 * dotr * dottheta + G * sin(theta)) / (L0 + r)

		return deriv

	def step(self, dt):
		"""execute one time step of length dt and update state"""
		self.state = integrate.odeint(self.dstate_dt, self.state, [0, dt])[1]
		self.time_elapsed += dt

#------------------------------------------------------------
# set up initial state and global variables
pendulum = MassSpringPendulum([0.25, 0.0, 90.0, 0.0])
dt = 1./30 # 30 fps

#------------------------------------------------------------
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-3, 3), ylim=(-4, 2))
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def animate(i):
    """perform animation step"""
    global pendulum, dt
    pendulum.step(dt)
    
    line.set_data(*pendulum.position())
    time_text.set_text('time = %.1f' % pendulum.time_elapsed)
    return line, time_text

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000 * dt - (t1 - t0)

ani = animation.FuncAnimation(fig, animate, frames=300,
                              interval=interval, blit=True, init_func=init)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
#ani.save('mass_spring_pendulum.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()	

