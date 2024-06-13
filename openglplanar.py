import sys
import os
import math
import numpy as np
import time
import json
# INSTALL OpenGL module
# git clone https://github.com/mcfletch/pyopengl
# cd pyopengl
# pip install -e .
# cd accelerate
# pip install -e .
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import asyncio
# INSTALL websocket : pip install websockets
from websockets.sync.client import connect

# Window dimensions
width, height = 1200, 600

# ROBOT MODEL PARAMETER
Xoffset = 0.0
Yoffset = 0.0
d1 = 0.1
a1 = 0.02
a2 = 0.13
a3 = 0.02
d4 = 0.15
a5 = 0.0
d6 = 0.125

L0 = d1;
L1 = a1;
L2 = a2;
L3 = a3;
L4 = d4;
L5 = a5;
L6 = d6;
L7 = 0;

x, y, z = 0.0, 0.0, 0.0
RTD = 180.0 / math.pi
DTR = math.pi / 180.0

# Joint Angle
joint1 = 0.0 * DTR;
joint2 = 20.0 * DTR;
joint3 = 45.0 * DTR;
joint4 = 0.0 * DTR;
joint5 = 60.0 * DTR;
joint6 = 0.0 * DTR;


# Initialize Variable
k = 0
t = 0
linemode = 0;

error_x_old, error_y_old, error_z_old = 0, 0, 0
integral_error_x, integral_error_y, integral_error_z = 0, 0, 0

dq1, dq2, dq3, dq4, dq5, dq6 = 0, 0, 0, 0, 0, 0

x_init, y_init, z_init = 0, 0, 0

x, y, z = 0, 0, 0

# PID Controller gain
Kp, Ki, Kd = 10.0, 5.0, 20.0

# Sampling Time
dt = 0.001
traj = 1.0

# Setpoint
x_spnt = 0.25
y_spnt = 0.2
z_spnt = 0.15

MODE_SIMULATION = 1
MODE_REMOTE = 2
mode = MODE_REMOTE

# Define color constants
green1 = [0.8, 1.0, 0.8, 1.0]
blue1 = [0.1, 0.1, 1.0, 1.0]
blue2 = [0.2, 0.2, 1.0, 1.0]
blue3 = [0.3, 0.3, 1.0, 1.0]
yellow1 = [0.1, 0.1, 0.0, 1.0]
yellow2 = [0.2, 0.2, 0.0, 1.0]
pink6 = [0.8, 0.55, 0.6, 1.0]
yellow5 = [0.8, 0.8, 0.0, 1.0]
abu2 = [0.5, 0.5, 0.5, 1.0]
gray1 = [0.1, 0.1, 0.1, 1.0]
gray2 = [0.2, 0.2, 0.2, 1.0]
gray3 = [0.3, 0.3, 0.3, 1.0]
gray4 = [0.4, 0.4, 0.4, 1.0]
gray5 = [0.5, 0.5, 0.5, 1.0]
gray6 = [0.6, 0.6, 0.6, 1.0]
gray7 = [0.7, 0.7, 0.7, 1.0]
gray8 = [0.8, 0.8, 0.7, 1.0]
gray9 = [0.9, 0.9, 0.7, 1.0]

colors = [
    (1.0, 0.0, 0.0, 1.0), # Red
    (0.0, 1.0, 0.0, 1.0), # Green
    (0.0, 0.0, 1.0, 1.0), # Blue
    (1.0, 1.0, 0.0, 1.0), # Yellow
    (1.0, 0.0, 1.0, 1.0), # Magenta
    (0.0, 1.0, 1.0, 1.0)  # Cyan
]

# Initialize OpenGL quadric object
obj = gluNewQuadric()

# Initialize rotation and translation parameters
spin, rotation_x, rotation_y, rotation_z = 0, 0, 0, 0
translation_x, translation_y, translation_z = 0.0, 0.0, 0.0

def set_spin(x, y, z):
    global rotation_x, rotation_y, rotation_z
    rotation_x, rotation_y, rotation_z = x, y, z

def reset():
    global rotation_x, rotation_y, rotation_z
    global translation_x, translation_y, translation_z
    rotation_x, rotation_y, rotation_z = 0, 0, 0
    translate_x, translate_y, translate_z = 0.0, 0.0, 0.0

def spin_display():
    global spin
    spin += 1.0
    if spin > 360.0:
        spin -= 360.0
    glutPostRedisplay()

def drawOneLine(x1, y1, x2, y2):
    glBegin(GL_LINES)
    glVertex3f(x1, y1, 0.0)
    glVertex3f(x2, y2, 0.0)
    glEnd()


def model_cylinder(lowerRadius, upperRadius, length, res, color1, color2):
    glPushMatrix()
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1)
    glTranslatef(0, 0, -length / 2)
    gluCylinder(obj, lowerRadius, upperRadius, length, 20, res)
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2)
    gluDisk(obj, 0.01, lowerRadius, 20, res)
    glTranslatef(0, 0, length)
    gluDisk(obj, 0.01, upperRadius, 20, res)
    glPopMatrix()


def model_box(width, depth, height, color1, color2, color3, color):
    width /= 2.0
    depth /= 2.0
    height /= 2.0
    glBegin(GL_QUADS)
    # top
    if color == 1:
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1)
    glVertex3f(-width, -depth, height)
    glVertex3f(width, -depth, height)
    glVertex3f(width, depth, height)
    glVertex3f(-width, depth, height)
    glEnd()
    glBegin(GL_QUADS)
    # bottom
    if color == 1:
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1)
    glVertex3f(-width, -depth, -height)
    glVertex3f(width, -depth, -height)
    glVertex3f(width, depth, -height)
    glVertex3f(-width, depth, -height)
    glEnd()
    glBegin(GL_QUAD_STRIP)
    # sides
    if color == 1:
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2)
    glVertex3f(-width, -depth, height)
    glVertex3f(-width, -depth, -height)
    glVertex3f(width, -depth, height)
    glVertex3f(width, -depth, -height)
    glVertex3f(width, depth, height)
    glVertex3f(width, depth, -height)
    glVertex3f(-width, depth, height)
    glVertex3f(-width, depth, -height)
    glVertex3f(-width, -depth, height)
    glEnd()


def disp_floor(grid):
    if grid:
        glPushMatrix()
        dx, dy = 4.5, 4.5
        amount = 15
        x_min, x_max = -dx / 2.0, dx / 2.0
        x_sp = dx / amount
        y_min, y_max = -dy / 2.0, dy / 2.0
        y_sp = dy / amount

        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1)
        for i in range(49):
            drawOneLine(-2.4 + 0.1 * i, -2.4, -2.4 + 0.1 * i, 2.4)
        for i in range(49):
            drawOneLine(-2.4, -2.4 + 0.1 * i, 2.4, -2.4 + 0.1 * i)
        glPopMatrix()


def disp_robot():
    global Xoffset, Yoffset, d1
    global a1, a2, a3, d4, a5, d6
    global joint1, joint2, joint3, joint4, joint5, joint6

    glPushMatrix() 
    model_box(0.3, 0.5, 0.05, gray8, gray7, gray6, 1)    
    glTranslatef(Xoffset, Yoffset, d1/2)    
    # Draw base
    model_cylinder(0.1, 0.1, d1, 2, blue1, yellow2)   
    # Menuju joint-1
    glTranslatef(0, 0, d1/2)    
    glRotatef(rtod(joint1), 0, 0, 1)    

    # Gambar link1
    glPushMatrix()    
    glTranslatef(0, 0, a1/2)    
    model_cylinder(0.03, 0.03, a1, 2, colors[0], gray1)    
    glPopMatrix()    

    # Menuju joint-2
    glTranslatef(0, 0, a1)    
    glRotatef(rtod(joint2), 1, 0, 0)  

    # Gambar link2
    glPushMatrix()   
    glTranslatef(0, 0, a2/2)    
    model_cylinder(0.03, 0.03, a2, 2, colors[1], gray2)    
    glPopMatrix()

    # Menuju joint-3
    glTranslatef(0, 0, a2)    
    glRotatef(rtod(joint3), 1, 0, 0)  

    # Gambar link3
    glPushMatrix()   
    glTranslatef(0, 0, a3/2)    
    model_cylinder(0.03, 0.03, a3, 2, colors[2], gray3)    
    glPopMatrix()    

    # Menuju joint-4
    glTranslatef(0, 0, a3)    
    glRotatef(rtod(joint4), 0, 0, 1)  

    # Gambar link4
    glPushMatrix()   
    glTranslatef(0, 0, d4/2)    
    model_cylinder(0.03, 0.03, d4, 2, colors[3], gray4)    
    glPopMatrix()

    # Menuju joint-5
    glTranslatef(0, 0, d4)    
    glRotatef(rtod(joint5), 1, 0, 0)  

    # Gambar link5
    glPushMatrix()   
    glTranslatef(0, 0, a5/2)    
    model_cylinder(0.03, 0.03, a5, 2, colors[4], gray5)    
    glPopMatrix()

    # Menuju joint-6
    glTranslatef(0, 0, a5)    
    glRotatef(rtod(joint6), 0, 0, 1)  

    # Gambar link6
    glPushMatrix()   
    glTranslatef(0, 0, d6/2)    
    model_cylinder(0.03, 0.03, d6, 2, colors[5], gray6)    
    glPopMatrix()

    glPopMatrix()

def drawLine(p1, p2):
    glBegin(GL_LINES)
    glVertex2f(*p1)
    glVertex2f(*p2)
    glEnd()


def rtod(rad):
    return rad * RTD


def reshape(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, 2, 0.2, 8)
    gluLookAt(0.3, 0.0, 1.5, -0.1, 0.0, 0.4, 0.0, 0.0, 1.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def display():    
    global rotation_x, rotation_y, rotation_z
    global translation_x, translation_y, translation_z
    global k
    global websocket
    global joint1, joint2, joint3, joint4, joint5, joint6, x, y, z

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # Apply translation and rotation
    glTranslatef(translation_x, translation_y, translation_z)
    glRotatef(rotation_x, 1.0, 0.0, 0.0)
    glRotatef(rotation_y, 0.0, 1.0, 0.0)
    glRotatef(rotation_z, 0.0, 0.0, 1.0)

    disp_floor(True)

    k+=1;
    if (mode==MODE_REMOTE) :
        websocket.send("states")
        message = websocket.recv()
        data = json.loads(message)
        # print(data)
        # print(data["joint"])
        t = data['t']
        ksisa = data['ksisa']
        joint1 = float(data['joint']['1'])
        joint2 = float(data['joint']['2'])
        joint3 = float(data['joint']['3'])
        joint4 = float(data['joint']['4'])
        joint5 = float(data['joint']['5'])
        joint6 = float(data['joint']['6'])
        joint1cmd = float(data['jointcmd']['1'])
        joint2cmd = float(data['jointcmd']['2'])
        joint3cmd = float(data['jointcmd']['3'])
        joint4cmd = float(data['jointcmd']['4'])
        joint5cmd = float(data['jointcmd']['5'])
        joint6cmd = float(data['jointcmd']['6'])
        x = float(data['end']['x'])
        y = float(data['end']['y'])
        z = float(data['end']['z'])
        # fremote.write(f"{t:.3f}, {ksisa:d}, {joint1cmd:.3f}, {joint1:.3f}, {joint2cmd:.3f}, {joint2:.3f}, {x:.3f}, {y:.3f}\n")
        fremote.write(f"{t:.3f}, {ksisa:d}, {joint1cmd:.3f}, {joint1:.3f}, {joint2cmd:.3f}, {joint2:.3f}, {joint3cmd:.3f}, {joint3:.3f}, {joint4cmd:.3f}, {joint4:.3f}, {joint5cmd:.3f}, {joint5:.3f}, {joint6cmd:.3f}, {joint6:.3f}, {x:.3f}, {y:.3f}, {z:.3f}\n")
        fremote.flush()
        if (k%1==0) :
            print(f"Received: {message}")
    if (mode==MODE_SIMULATION):
        control_local()
    disp_robot()

    glFlush()
    glutSwapBuffers()
    time.sleep(0.05)


def forward_kinematic(joint1, joint2, joint3, joint4, joint5, joint6):
    x = L1*math.cos(joint1) + L5*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) - math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L6*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L7*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L3*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) + L4*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2)) + L2*math.cos(joint1)*math.cos(joint2)
    y = L4*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)) - L3*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) + L1*math.sin(joint1) - L5*(math.cos(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) + math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L6*(math.sin(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L7*(math.sin(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) + L2*math.cos(joint2)*math.sin(joint1)
    z = L0 + L3*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - L4*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + L5*(math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))) - L6*(math.cos(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))) - L7*(math.cos(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))) + L2*math.sin(joint2)
    return x, y, z

def PID_controller(error_x, error_y, error_z):
    global error_x_old, error_y_old, error_z_old, integral_error_x, integral_error_y, integral_error_z
    der_error_x = (error_x - error_x_old) / dt
    der_error_y = (error_y - error_y_old) / dt
    der_error_z = (error_z - error_z_old) / dt
    
    integral_error_x += error_x * dt
    integral_error_y += error_y * dt
    integral_error_z += error_z * dt
    
    ddx = Kp * error_x + Ki * integral_error_x + Kd * der_error_x
    ddy = Kp * error_y + Ki * integral_error_y + Kd * der_error_y
    ddz = Kp * error_z + Ki * integral_error_z + Kd* der_error_z
    
    error_x_old = error_x
    error_y_old = error_y
    error_z_old = error_z
    
    return ddx, ddy, ddz

def inverse_jacobian(ddx, ddy, ddz, joint1, joint2, joint3, joint4, joint5, joint6):
    # Calculate the components of the Jacobian matrix
    J11 = L3*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) - L4*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)) - L1*math.sin(joint1) + L5*(math.cos(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) + math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) + L6*(math.sin(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) + L7*(math.sin(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L2*math.cos(joint2)*math.sin(joint1)
    J12 = L6*(math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) - L5*(math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) + math.cos(joint4)*math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L7*(math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) - L3*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2)) + L4*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) - L2*math.cos(joint1)*math.sin(joint2)
    J13 = L6*(math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) - L5*(math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) + math.cos(joint4)*math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L7*(math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) - L3*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2)) + L4*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))
    J14 = L5*math.cos(joint5)*(math.cos(joint4)*math.sin(joint1) - math.sin(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + L6*math.sin(joint5)*(math.cos(joint4)*math.sin(joint1) - math.sin(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + L7*math.sin(joint5)*(math.cos(joint4)*math.sin(joint1) - math.sin(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)))
    J15 = L6*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) - math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) - L5*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L7*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) - math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2)))
    J16 = 0

    J21 = L1*math.cos(joint1) + L5*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) - math.sin(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L6*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L7*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint4) + math.cos(joint4)*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3))) + math.cos(joint5)*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2))) + L3*(math.cos(joint1)*math.cos(joint2)*math.cos(joint3) - math.cos(joint1)*math.sin(joint2)*math.sin(joint3)) + L4*(math.cos(joint1)*math.cos(joint2)*math.sin(joint3) + math.cos(joint1)*math.cos(joint3)*math.sin(joint2)) + L2*math.cos(joint1)*math.cos(joint2)
    J22 = L5*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) - math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L4*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) - L3*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)) - L6*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L7*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L2*math.sin(joint1)*math.sin(joint2)
    J23 = L5*(math.sin(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) - math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L4*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) - L3*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)) - L6*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L7*(math.cos(joint5)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)))
    J24 = -L5*math.cos(joint5)*(math.cos(joint1)*math.cos(joint4) - math.sin(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - L6*math.sin(joint5)*(math.cos(joint1)*math.cos(joint4) - math.sin(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - L7*math.sin(joint5)*(math.cos(joint1)*math.cos(joint4) - math.sin(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1)))
    J25 = L5*(math.sin(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) - math.cos(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L6*(math.cos(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) + math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2))) - L7*(math.cos(joint5)*(math.cos(joint1)*math.sin(joint4) + math.cos(joint4)*(math.sin(joint1)*math.sin(joint2)*math.sin(joint3) - math.cos(joint2)*math.cos(joint3)*math.sin(joint1))) + math.sin(joint5)*(math.cos(joint2)*math.sin(joint1)*math.sin(joint3) + math.cos(joint3)*math.sin(joint1)*math.sin(joint2)))
    J26 = 0

    J31 = 0
    J32 = L3*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + L4*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - L5*(math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3))) + L6*(math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3))) + L7*(math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3))) + L2*math.cos(joint2)
    J33 = L3*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + L4*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - L5*(math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3))) + L6*(math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3))) + L7*(math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) + math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)))
    J34 = -L6*math.sin(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - L7*math.sin(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)) - L5*math.cos(joint5)*math.sin(joint4)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))
    J35 = L5*(math.cos(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) - math.cos(joint4)*math.sin(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))) + L6*(math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2))) + L7*(math.sin(joint5)*(math.cos(joint2)*math.cos(joint3) - math.sin(joint2)*math.sin(joint3)) + math.cos(joint4)*math.cos(joint5)*(math.cos(joint2)*math.sin(joint3) + math.cos(joint3)*math.sin(joint2)))
    J36 = 0

    # Construct the Jacobian matrix
    J = np.array([
        [J11, J12, J13, J14, J15, J16],
        [J21, J22, J23, J24, J25, J26],
        [J31, J32, J33, J34, J35, J36]
    ])

    # Calculate the pseudo-inverse of the Jacobian matrix
    J_pseudo_inv = np.linalg.pinv(J)

    # Calculate the reference angular accelerations
    ddq = J_pseudo_inv @ np.array([ddx, ddy, ddz])
    return ddq[0], ddq[1], ddq[2], ddq[3], ddq[4], ddq[5]

def trajectory_line(t, x, y, z, linemode, x_spnt, y_spnt, z_spnt):
    global x_init, y_init, z_init
    if t < 0.02:
        x_init = x
        y_init = y
        z_init = z
        
        if linemode == 1:
            linemode = 0
        else:
            linemode = 1
            
        if linemode == 0:
            x_spnt = x + 0.1
            y_spnt = y + 0.1
            z_spnt = z + 0.1
        else:
            x_spnt = x - 0.1
            y_spnt = y - 0.1
            z_spnt = z - 0.1
    
    x_d = (x_spnt - x_init) * t / traj + x_init
    y_d = (y_spnt - y_init) * t / traj + y_init
    z_d = (z_spnt - z_init) * t / traj + z_init
    
    return x_d, y_d, z_d, linemode

def control_local():
    global joint1, joint2, joint3, joint4, joint5, joint6, dq1, dq2, dq3, dq4, dq5, dq6, x, y, z, t, linemode, x_spnt, y_spnt, z_spnt
    
    x, y, z = forward_kinematic(joint1, joint2, joint3, joint4, joint5, joint6)
    x_d, y_d, z_d, linemode = trajectory_line(traj * (t / traj - math.trunc(t / traj)), x, y, z, linemode, x_spnt, y_spnt, z_spnt)
    
    error_x = x_d - x
    error_y = y_d - y
    error_z = z_d - z
    
    ddx, ddy, ddz = PID_controller(error_x, error_y, error_z)
    
    ddq1_ref, ddq2_ref, ddq3_ref, ddq4_ref, ddq5_ref, ddq6_ref = inverse_jacobian(ddx, ddy, ddz, joint1, joint2, joint3, joint4, joint5, joint6)
    
    k = 5.0
    v1 = k * ddq1_ref
    v2 = k * ddq2_ref
    v3 = k * ddq3_ref
    v4 = k * ddq4_ref
    v5 = k * ddq5_ref
    v6 = k * ddq6_ref
    
    dq1 = dq1 + (2.083 * v1 - 2.71 * dq1) * dt
    dq2 = dq2 + (2.083 * v2 - 2.71 * dq2) * dt
    dq3 = dq3 + (2.083 * v3 - 2.71 * dq3) * dt
    dq4 = dq4 + (2.083 * v4 - 2.71 * dq4) * dt
    dq5 = dq5 + (2.083 * v5 - 2.71 * dq5) * dt
    dq6 = dq6 + (2.083 * v6 - 2.71 * dq6) * dt
    
    joint1 = joint1 + dq1 * dt
    joint2 = joint2 + dq2 * dt
    joint3 = joint3 + dq3 * dt
    joint4 = joint4 + dq4 * dt
    joint5 = joint5 + dq5 * dt
    joint6 = joint6 + dq6 * dt
    
    # print(f"t: {t:.2f}, x_spnt: {x_spnt:.2f}, y_spnt: {y_spnt:.2f}, z_spnt: {z_spnt:.2f}, x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, joint1: {joint1:.2f}, joint2: {joint2:.2f}, joint3: {joint3:.2f}, joint4: {joint4:.2f}, joint5: {joint5:.2f}, joint6: {joint6:.2f}, linemode: {linemode}")

    print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, joint1: {joint1:.2f}, joint2: {joint2:.2f}, joint3: {joint3:.2f}, joint4: {joint4:.2f}, joint5: {joint5:.2f}, joint6: {joint6:.2f}")
    flocal.write(f"{k:d}, {joint1:.2f}, {joint2:.2f}, {joint3:.2f}, {joint4:.2f}, {joint5:.2f}, {joint6:.2f}, {x:.2f}, {y:.2f}, {z:.2f}\n")
    flocal.flush()

    t = t + dt

# Function to handle keyboard input
def keyboard(key, x, y):
    global rotation_x, rotation_y, rotation_z
    global translation_x, translation_y, translation_z
    global mode
    global joint1, joint2, joint3, joint4, joint5, joint6
    match key:
        case b'\x1b':
            os._exit(1)
        case b'a':
            translation_x += 0.1
        case b'A':
            translation_x -= 0.1
        case b's':
            translation_y += 0.1
        case b'S':
            translation_y -= 0.1
        case b'd':
            translation_z += 0.1
        case b'D':
            translation_z -= 0.1
        case b'q':
            rotation_x += 1.0
        case b'Q':
            rotation_x -= 1.0
        case b'w':
            rotation_y += 1.0
        case b'W':
            rotation_y -= 1.0
        case b'e':
            rotation_z += 1.0
        case b'E':
            rotation_z -= 1.0
        case b'1':
            joint1 += 5.0 * DTR
        case b'!':
            joint1 -= 5.0 * DTR
        case b'2':
            joint2 += 5.0 * DTR
        case b'@':
            joint2 -= 5.0 * DTR
        case b'3':
            joint3 += 5.0 * DTR
        case b'#':
            joint3 -= 5.0 * DTR
        case b'4':
            joint4 += 5.0 * DTR
        case b'$':
            joint4 -= 5.0 * DTR
        case b'5':
            joint5 += 5.0 * DTR
        case b'%':
            joint5 -= 5.0 * DTR
        case b'6':
            joint6 += 5.0 * DTR
        case b'^':
            joint6 -= 5.0 * DTR
        case b'b':
            data = {"cmd": "joint", "joint": "dq1", "value": 10};
            websocket.send(json.dumps(data));
        case b'n':
            data = {"cmd": "joint", "joint": "dq2", "value": 10};
            websocket.send(json.dumps(data));
        case b'm':
            data = {"cmd": "task", "axis": "dx", "value": 0.05};
            websocket.send(json.dumps(data));
        case b',':
            if (mode == MODE_REMOTE) :
                mode = MODE_SIMULATION
                print("MODE SIMULATION")
            else :
                mode = MODE_REMOTE
                print("MODE REMOTE")
        case _:
            # do nothing
            type(key)
            print("selesai")

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE)
    glutInitWindowSize(width, height)
    glutCreateWindow(b"Robot Simulation")
    glEnable(GL_DEPTH_TEST)

    light_ambient =  [0.2, 0.2, 0.2, 1.0]
    light_diffuse =  [0.4, 0.4, 0.4, 1.0]
    light_specular = [0.3, 0.3, 0.3, 1.0]
    light_position = [2, 0.1, 7,1.0]
    spot_direction = [0.0, -0.1, -1.0, 1.0]

    glClearColor(0.0, 0.0, 0.0, 0.0)    
  
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
    glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
    glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    # glEnable(GL_LIGHTING)
    # glEnable(GL_LIGHT0)
    # glLightfv(GL_LIGHT0, GL_POSITION, [1, -1, 1, 0])

    glutDisplayFunc(display)
    glutIdleFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutMainLoop()


if __name__ == "__main__":
    print("Init WebSocket to 192.168.4.1")
    websocket = connect("ws://192.168.4.1/ws")
    print("Running in simulation mode")
    flocal = open("loglocal.csv", "w")
    fremote = open("logremote.csv", "w")
    main()