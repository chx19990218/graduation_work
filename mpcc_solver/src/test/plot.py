import matplotlib.pyplot as plt
import os
import numpy as np
import Tkinter as tk
import math



des_p_x = []
odom_p_x = []
des_v_x = []
odom_v_x = []
des_a_x = []

des_p_y = []
odom_p_y = []
des_v_y = []
odom_v_y = []
des_a_y = []

des_p_z = []
odom_p_z = []
des_v_z = []
odom_v_z = []
des_a_z = []

des_a_x = []
des_a_y = []
des_a_z = []

center_point_x_ = []
center_point_y_ = []
outer_point_x_ = []
outer_point_y_ = []
inner_point_x_ = []
inner_point_y_ = []

t = []

logPath = os.path.abspath('.')
logPath += "/log/"
x_log = open(logPath + "x.txt", "r")
y_log = open(logPath + "y.txt", "r")
z_log = open(logPath + "z.txt", "r")
corridor_log = open(logPath + "corridor.txt", "r")

x_lines = x_log.readlines()
count = 1
time = 0
for line in x_lines:
  if count<2:
    count = count + 1
    continue
  time += 0.02
  t.append(time)
  index1 = line.find('/')
  des_p_x.append(float(line[0:index1]))
  index2 = line.find('/', index1+1, len(line))
  odom_p_x.append(float(line[index1+1:index2]))
  index3 = line.find('/', index2+1, len(line))
  des_v_x.append(float(line[index2+1:index3]))
  index4 = line.find('/', index3+1, len(line))
  odom_v_x.append(float(line[index3+1:index4]))
  des_a_x.append(float(line[index4+1:len(line)]))

y_lines = y_log.readlines()
count = 1
for line in y_lines:
  if count<2:
    count = count + 1
    continue
  index1 = line.find('/')
  des_p_y.append(float(line[0:index1]))
  index2 = line.find('/', index1+1, len(line))
  odom_p_y.append(float(line[index1+1:index2]))
  index3 = line.find('/', index2+1, len(line))
  des_v_y.append(float(line[index2+1:index3]))
  index4 = line.find('/', index3+1, len(line))
  odom_v_y.append(float(line[index3+1:index4]))
  des_a_y.append(float(line[index4+1:len(line)]))

z_lines = z_log.readlines()
count = 1
for line in z_lines:
  if count<2:
    count = count + 1
    continue
  index1 = line.find('/')
  des_p_z.append(float(line[0:index1]))
  index2 = line.find('/', index1+1, len(line))
  odom_p_z.append(float(line[index1+1:index2]))
  index3 = line.find('/', index2+1, len(line))
  des_v_z.append(float(line[index2+1:index3]))
  index4 = line.find('/', index3+1, len(line))
  odom_v_z.append(float(line[index3+1:index4]))
  des_a_z.append(float(line[index4+1:len(line)]))

corridor_lines = corridor_log.readlines()
for line in corridor_lines:
  index1 = line.find('/')
  center_point_x_.append(float(line[0:index1]))
  index2 = line.find('/', index1+1, len(line))
  center_point_y_.append(float(line[index1+1:index2]))
  index3 = line.find('/', index2+1, len(line))
  outer_point_x_.append(float(line[index2+1:index3]))
  index4 = line.find('/', index3+1, len(line))
  outer_point_y_.append(float(line[index3+1:index4]))
  index5 = line.find('/', index4+1, len(line))
  inner_point_x_.append(float(line[index4+1:index5]))
  inner_point_y_.append(float(line[index5+1:len(line)]))



def px_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("p_x/m")
  plt.grid()
  
  plt.plot(t, des_p_x, 'r')
  plt.plot(t, odom_p_x, 'b')
  plt.show()

def py_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("p_y/m")
  plt.grid()
  
  plt.plot(t, des_p_y, 'r')
  plt.plot(t, odom_p_y, 'b')
  plt.show()

def pz_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("p_z/m")
  plt.grid()
  
  plt.plot(t, des_p_z, 'r')
  plt.plot(t, odom_p_z, 'b')
  plt.show()

def vx_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("v_x/m/s")
  plt.grid()
  
  plt.plot(t, des_v_x, 'r')
  plt.plot(t, odom_v_x, 'b')
  plt.show()

def vy_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("v_y/m/s")
  plt.grid()
  
  plt.plot(t, des_v_y, 'r')
  plt.plot(t, odom_v_y, 'b')
  plt.show()

def vz_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("v_z/m")
  plt.grid()
  
  plt.plot(t, des_v_z, 'r')
  plt.plot(t, odom_v_z, 'b')
  plt.show()

def a_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("a/m/s^2")
  plt.grid()
  
  plt.plot(t, des_a_x, 'r')
  plt.plot(t, des_a_y, 'g')
  plt.plot(t, des_a_z, 'b')
  plt.show()

def traj_callback():
  plt.figure()
  plt.xlabel("t/s")
  plt.ylabel("p_z/m")
  plt.grid()

  plt.plot(center_point_x_, center_point_y_, linestyle='--', color='y')
  plt.plot(outer_point_x_, outer_point_y_, color='k')
  plt.plot(inner_point_x_, inner_point_y_, color='k')
  
  plt.plot(des_p_x, des_p_y, 'r')
  plt.plot(odom_p_x, odom_p_y, 'b')
  plt.show()

frame = tk.Tk()
frame.title('Visualization')
frame.geometry('500x200+1000+400')

bigmenu = tk.Menu(frame)
datamenu = tk.Menu(bigmenu, tearoff = 0)
datamenu.add_command(label='px', command=px_callback)
datamenu.add_command(label='py', command=py_callback)
datamenu.add_command(label='pz', command=pz_callback)
datamenu.add_separator()
datamenu.add_command(label='vx', command=vx_callback)
datamenu.add_command(label='vy', command=vy_callback)
datamenu.add_command(label='vz', command=vz_callback)
datamenu.add_separator()
datamenu.add_command(label='a', command=a_callback)
datamenu.add_separator()
datamenu.add_command(label='trajectory', command=traj_callback)

bigmenu.add_cascade(label = 'data',menu = datamenu)
frame.config(menu = bigmenu)

message = tk.StringVar()
message.set('*********** Welcome  to  DataPloter ***********')
label = tk.Label(frame, textvariable=message).place(x=40,y=100)

# tk.Button(frame, text="plot",width=7,height=2, command=plot_callback).place(x=400,y=20)

frame.mainloop()