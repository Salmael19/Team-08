#!/usr/bin/env python
# coding: utf-8

# In[7]:


from numpy import *
from numpy.linalg import *

from scipy.integrate import *
from scipy.signal import *

from matplotlib.pyplot import *


# Chariot Model
# ==============
# 
# We consider a mobile robot of "chariot" type, evolving in the plane.
# 
#   - From above, its frame appears to be a disk of diameter $D$. 
#   
#   - It has two fixed wheels (their orientation with respect to the frame does not change; there is no need for a steering wheel!). They are parallel and located at opposite ends of the frame (hence separated by a distance of $D$) ; these wheels have the common radius $R$.
# 
#   - The robot is symmetric with respect to the axis that joins the two wheels. For convenience, a green sticker is attached at the boundary of the frame on one side of this axis, on the robot axis of symmetry ; this side will is called the "front" of the robot, the point itself is called the "nose" of the robot. When we talk about the "left wheel" or the "right wheel", we assume that we are looking towards the front of the robot.
# 
# We will denote 
# 
#   - $(x, y)$ the coordinates of the wheel axis in the orthonormal and direct frame $(0, e_x, e_y)$ of the plane.
#   
#   - $\theta$ the angle of the wheels direction with respect to vector $e_x$ ; with $\theta = 0$ when the front of the robot faces the right ($e_x$) and $\theta = \pi/2$ when he faces upwards ($e_y$).
# 
#   - $\phi_l$ and $\phi_r$ the rotation angles of the left and right wheel respectively ; by convention, when these angles increase, the robot moves forward.

# ### Parameter Definition
# 
# 
# From now on, we assume that the frame diameter is $D = 1 \mbox{ m}$ and the wheel radius is $R = 10 \mbox{ cm}$.
# 
# 🧩 Define the corresponding Python variables `D` and `R` (express every length in meters, and more generally in the sequel, always use the [international standard of units](https://en.wikipedia.org/wiki/SI_derived_unit)) for numerical values. 

# In[9]:


D=1  
R=0.10


# ### Graphical Representation
# 
# 🧩 Implement a function `draw_robot` with arguments `x`, `y` and `theta` that draws the robot in the plane (top view) for arbitrary values of $(x, y)$ and $\theta$. Represent the robot frame as a circle, the wheels as lines and the nose as an orange point.
# 
# 🗝️ **Hint.** Use the function `plot` of `matplotlib.pyplot`.

# In[10]:


def draw_robot(x, y, theta):
    draw_arena()
    import matplotlib.pyplot as plt
    import numpy as np
    

    #coordinates of circle center
    Xo = -D/2*cos(np.pi/2-theta)+x
    Yo = D/2*sin(np.pi/2-theta)+y
    #coordinates of wheel line points
    #line 1
    X0 = R*cos(theta)+x
    Y0 = R*sin(theta)+y
    
    X1 = -R*cos(theta)+x
    Y1 = -R*sin(theta)+y
    
    #line 2
    X2 = R*cos(theta)-D*cos(np.pi/2-theta)+x
    Y2 = R*sin(theta)+D*sin(np.pi/2-theta)+y
    
    X3 = -R*cos(theta)-D*cos(np.pi/2-theta)+x
    Y3 = -R*sin(theta)+D*sin(np.pi/2-theta)+y
    
    #draw circle
    circle=plt.Circle((Xo, Yo), D/2, fc='y')
    
    #draw lines
    line1=plt.Line2D((X0,Y0), (X1,Y1))
    line2=plt.Line2D((X2, Y2), (X3, Y3))
    #plt.scatter((X0,Y0), (X1,Y1),(X2,Y2), (X3,Y3), linewidths=5)
    # plt.scatter((X2,Y2), (X3,Y3))
    plt.gca().add_line(line1)
    plt.gca().add_line(line2)
    plt.gca().add_patch(circle)
    #axes.add_line(line1)
    #axes.add_line(line2)
    #axes.add_patch(circle)
    pass
    #draw_arena()
draw_robot(2, 2, np.pi/2)
    
    


# 🧩 Represent the robot in when $(x, y, \theta) = (0, 0, 0), (2, 2, \pi/2), (0, 4, -\pi), (-4, 4, -\pi), (-8, 4, -\pi)$.
# 
# 🗝️ **Hint.** Use the following `draw_arena` function beforehand.

# In[2]:


def draw_arena(xmin=-12, xmax=12, ymin=-9, ymax=9):
    wh_ratio = (xmax - xmin) / (ymax - ymin)
    figsize = fig_width, fig_height = 16, 16 / wh_ratio
    figure(figsize=figsize)
    axes = gca()
    axes.axis([xmin, xmax, ymin, ymax])
    axes.set_aspect(1)
    xticks(arange(xmin, xmax+1, 1))
    yticks(arange(ymin, ymax+1, 1))
    grid(True)
    plot([xmin, xmax], [0, 0], linestyle="-", color="grey")
    plot([0, 0], [ymin, ymax], linestyle="-", color="grey")


# In[4]:


draw_arena()
pass


# ### Kinematic Modeling
# 
# We assume that we can control the wheels angular velocities $\omega_l = \dot{\phi}_l$ and $\omega_r = \dot{\phi}_r$ (this is a *kinematic* model of the robot).
# We also assume that the chariot wheels roll without slipping. 
# 
# 🧩 Establish the differential equations that relate $\dot{x}$,
# $\dot{y}$ and $\dot{\theta}$ with $\omega_l$ and $\omega_r$.

# **Answer:**

# ### Analysis
# 
# 🧩 Show that if $\omega_l$ and $\omega_r$ are continuously differentiable functions of $(x, y, \theta)$ (which includes the case of constant inputs), this system is well-posed.

# **Answer:**

# 🧩 Show that if $\omega_l$ and $\omega_r$ are constants, the maximal solution associated to any initial state is global. Is is still true of $\omega_l$ and $\omega_r$ are arbitrary continuously differentiable functions of $(x, y, \theta)$ ?

# **Answer:**

# ### Vector Field
# 
# In the sequel, `omega` is a function that computes the input vector $\omega := (\omega_l, \omega_r)$ given the time $t$ and $(x, y, \theta)$ (thus, it may depend on the time and/or on the system state if needed). For example:

# In[5]:


def omega(t, xytheta):
    omega_l = 7.5 * pi
    omega_r = 12.5 * pi
    return array([omega_l, omega_r])


# 🧩 Implement a function `fun(t, xytheta)` that computes $(\dot{x}, \dot{y}, \dot{\theta})$ given the time $t$, $(x,y,\theta)$ (and  -- implicitly -- the inputs $\omega$ computed by the function `omega`).

# In[6]:


def fun(t, xytheta):
    pass


# ### Simulation Setup
# 
# 🧩 Simulate the robot dynamics for 4 seconds when $(x_0, y_0, \theta_0)(t=0) = (0.0, 0.0, 0.0)$,
# and the wheels velocities are the constants $\omega_l = 7.5 \pi$ and $\omega_r = 12.5 \pi$.
# (Compute the dense outputs.)
#    

# In[7]:


pass


# 🧩 Use the simulation results to draw the graphs of $x(t)$, $y(t)$ and $\theta(t)$.

# In[8]:


pass


# 🧩 Use the simulation results to display the robot location every 0.5 second.

# In[9]:


pass


# 🧩 What should be the analytical expression of $x(t)$, $y(t)$ and $\theta(t)$? 

# **Answer:**

# 🧩 Do your experimental results match the theoretical results ? Draw the curves of the simulation error at time $t$ for $x$, $y$ and $\theta$.

# In[10]:


pass


# 🧩 If needed, adjust the simulation parameters until this error is uniformly less than $1e-5$ for each variable.

# In[11]:


pass

