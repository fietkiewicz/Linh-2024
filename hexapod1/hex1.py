"""
Uses hex1.xml
Usage: python hex1.py
  or
Usage: python hex1.py hex1.txt
"""
import sys # For sys.argv
import time
import numpy as np
import mujoco
import mujoco.viewer
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import mujocoviewer2

def init_and_run():
    filename = 'hex1.xml'
    print('filename = ', filename)
    model = mujoco.MjModel.from_xml_path(filename)
    data = mujoco.MjData(model)

    N = 100000
    i = 0
    phase = int(start_phrase.get())
    # Phases (start @ forward/down): -1=start, 0=back, 1=up, 2=forward, 3=down
    # Horizontal force: negative=forward, positive=backward
    # Vertical force: negative=up, positive=down
    # controllerYaw0 = [0, 0, 0, 0, 0, 0, 0, 0] # negative=forward, positive=backward
    # controllerLift0 = [0, 0, 0, 0, 0, 0, 0, 0] # negative=up, positive=down
    controllerYaw0 = [float(ctr1_1.get()), float(ctr1_2.get()), float(ctr1_3.get()), float(ctr1_4.get()), 0, 0, 0, 0] # negative=forward, positive=backward
    controllerLift0 = [float(ctr2_1.get()), float(ctr2_2.get()), float(ctr2_3.get()), float(ctr2_4.get()), 0, 0, -1, -1] # negative=up, positive=down
    controllerYaw1 = [0, 0, 0, 0, -float(ctr1_1.get()), -float(ctr1_2.get()), -float(ctr1_3.get()), -float(ctr1_4.get())] # negative=forward, positive=backward
    controllerLift1 = [0, 0, -1, -1, float(ctr2_1.get()), float(ctr2_2.get()), float(ctr2_3.get()), float(ctr2_4.get())] # negative=up, positive=down

    print("Initial values: ")
    printDebug(phase, data)
    viewer = mujocoviewer2.MujocoViewer(model, data)

    # Hack to let it fall before moving
    for j in range(0,100):
        if (j % int(speed.get()) == 0):
            viewer.render()
        j += 1
        mujoco.mj_step(model, data)

    while viewer.is_alive and i < N:
        # Controller handling
        # print('phase = ', phase, ', value = ', controllerValue)
        if phase == 0 and data.joint('pitch_mid_right').qpos[0] <= float(angle_1.get()):
            phase = 1
            printDebug(phase, data)
        elif phase == 1 and data.joint('yaw_mid_right').qpos[0] >= float(angle_2.get()):
            phase = 2
            printDebug(phase, data)
        elif phase == 2 and data.joint('pitch_mid_right').qpos[0] >= float(angle_3.get()):
            phase = 3
            printDebug(phase, data)
        elif phase == 3 and data.joint('yaw_mid_right').qpos[0] <= float(angle_4.get()):
            phase = 4
            printDebug(phase, data)
        elif phase == 4 and data.joint('pitch_mid_left').qpos[0] <= float(angle_5.get()):
            phase = 5
            printDebug(phase, data)
        elif phase == 5 and data.joint('yaw_mid_left').qpos[0] <= float(angle_6.get()):
            phase = 6
            printDebug(phase, data)
        elif phase == 6 and data.joint('pitch_mid_left').qpos[0] >= float(angle_7.get()):
            phase = 7
            printDebug(phase, data)
        elif phase == 7 and data.joint('yaw_mid_left').qpos[0] >= float(angle_8.get()):
            phase = 0
            printDebug(phase, data)

        # Apply force
        data.ctrl[0] = controllerYaw0[phase] * (-1) # yaw_front_left (GROUP 0)
        data.ctrl[1] = controllerLift0[phase] # lift_front_left (GROUP 0)
        data.ctrl[3] = controllerYaw1[phase] * (-1) # yaw_front_right (GROUP 1)
        data.ctrl[4] = controllerLift1[phase] # lift_front_right (GROUP 1)
        data.ctrl[6] = controllerYaw1[phase] # yaw_mid_left (GROUP 1)
        data.ctrl[7] = controllerLift1[phase] # lift_mid_left (GROUP 1)
        data.ctrl[9] = controllerYaw0[phase] # yaw_mid_right (GROUP 0)
        data.ctrl[10] = controllerLift0[phase] # lift_mid_right (GROUP 0)
        data.ctrl[12] = controllerYaw1[phase] * (-1) # yaw_back_right (GROUP 1)
        data.ctrl[13] = controllerLift1[phase] # lift_back_right (GROUP 1)
        data.ctrl[15] = controllerYaw0[phase] * (-1) # yaw_back_left (GROUP 0)
        data.ctrl[16] = controllerLift0[phase] # lift_back_left (GROUP 0)

        # Simulation step
        if (i % int(speed.get()) == 0):
            viewer.render()
        i += 1

        mujoco.mj_step(model, data)
    viewer.close()

def printDebug(phase, data):
        print('phase = ', phase)
        print(data.joint('yaw_mid_right').qpos[0], ", ", data.joint('pitch_mid_right').qpos[0])

def save_setting():
    fileload = fd.asksaveasfile(mode = "w+")
    save = []

    save.append(str(ctr1_1.get()))
    save.append(str(ctr1_2.get()))
    save.append(str(ctr1_3.get()))
    save.append(str(ctr1_4.get()))
    save.append(str(ctr2_1.get()))
    save.append(str(ctr2_2.get()))
    save.append(str(ctr2_3.get()))
    save.append(str(ctr2_4.get()))
    save.append(str(start_1.get()))
    save.append(str(start_2.get()))
    save.append(str(speed.get()))
    save.append(str(angle_1.get()))
    save.append(str(angle_2.get()))
    save.append(str(angle_3.get()))
    save.append(str(angle_4.get()))
    save.append(str(angle_5.get()))
    save.append(str(angle_6.get()))
    save.append(str(angle_7.get()))
    save.append(str(angle_8.get()))
    save.append(str(start_phrase.get()))

    fileload.write('\n'.join(save))
    fileload.close()

def load_setting(filename = ""):
    if (len(filename) > 0):
        fileload = open(filename,"r")
    else:
        fileload = open(fd.askopenfilename(),"r")
    save = []

    for line in fileload.readlines():
        save.append((line))

    fileload.close()

    ctr1_1.delete(0, tk.END)
    ctr1_2.delete(0, tk.END)
    ctr1_3.delete(0, tk.END)
    ctr1_4.delete(0, tk.END)
    ctr2_1.delete(0, tk.END)
    ctr2_2.delete(0, tk.END)
    ctr2_3.delete(0, tk.END)
    ctr2_4.delete(0, tk.END)
    start_1.delete(0, tk.END)
    start_2.delete(0, tk.END)
    speed.delete(0, tk.END)
    angle_1.delete(0, tk.END)
    angle_2.delete(0, tk.END)
    angle_3.delete(0, tk.END)
    angle_4.delete(0, tk.END)
    angle_5.delete(0, tk.END)
    angle_6.delete(0, tk.END)
    angle_7.delete(0, tk.END)
    angle_8.delete(0, tk.END)
    start_phrase.delete(0, tk.END)

    ctr1_1.insert(0, float(save[0].strip()))
    ctr1_2.insert(0, float(save[1].strip()))
    ctr1_3.insert(0, float(save[2].strip()))
    ctr1_4.insert(0, float(save[3].strip()))
    ctr2_1.insert(0, float(save[4].strip()))
    ctr2_2.insert(0, float(save[5].strip()))
    ctr2_3.insert(0, float(save[6].strip()))
    ctr2_4.insert(0, float(save[7].strip()))
    start_1.insert(0, float(save[8].strip()))
    start_2.insert(0, float(save[9].strip()))
    speed.insert(0, round(float(save[10].strip())))
    angle_1.insert(0, float(save[11].strip()))
    angle_2.insert(0, float(save[12].strip()))
    angle_3.insert(0, float(save[13].strip()))
    angle_4.insert(0, float(save[14].strip()))
    angle_5.insert(0, float(save[15].strip()))
    angle_6.insert(0, float(save[16].strip()))
    angle_7.insert(0, float(save[17].strip()))
    angle_8.insert(0, float(save[18].strip()))
    start_phrase.insert(0, round(float(save[19].strip())))

# GUI

window = tk.Tk()
window.title("Hexapod")
window.geometry("360x400")

ctrl1_label = Label(window, text = "First Controler Amplitude").grid(row=0, column=0, columnspan=4, pady=2)
ctr1_1 = Entry(window, width = 7)
ctr1_1.grid(row=1, column=0, pady=2, padx = 5)
ctr1_2 = Entry(window, width = 7)
ctr1_2.grid(row=1, column=1, pady=2, padx = 5)
ctr1_3 = Entry(window, width = 7)
ctr1_3.grid(row=1, column=2, pady=2, padx = 5)
ctr1_4 = Entry(window, width = 7)
ctr1_4.grid(row=1, column=3, pady=2, padx = 5)

ctrl2_label = Label(window, text = "Second Controler Amplitude").grid(row=2, column=0, columnspan=4, pady=2)
ctr2_1 = Entry(window, width = 7)
ctr2_1.grid(row=3, column=0, pady=2, padx = 5)
ctr2_2 = Entry(window, width = 7)
ctr2_2.grid(row=3, column=1, pady=2, padx = 5)
ctr2_3 = Entry(window, width = 7)
ctr2_3.grid(row=3, column=2, pady=2, padx = 5)
ctr2_4 = Entry(window, width = 7)
ctr2_4.grid(row=3, column=3, pady=2, padx = 5)

start_label = Label(window, text = "Starting Forces").grid(row=4, column=0, columnspan=2, pady=2)
start_1 = Entry(window, width = 7)
start_1.grid(row=5, column=0, padx=5, pady=2)
start_2 = Entry(window, width = 7)
start_2.grid(row=5, column=1, padx=5, pady=2)

speed_label = Label(window, text = "Speed").grid(row=4, column=3, pady=2)
speed = Entry(window, width = 7)
speed.grid(row=5, column=3, padx=5, pady=2)

angle_label = Label(window, text = "Angle Phase").grid(row=6, column=0, columnspan=4, pady=2)
angle_1 = Entry(window, width = 7)
angle_1.grid(row=7, column=0, pady=2, padx = 5)
angle_2 = Entry(window, width = 7)
angle_2.grid(row=7, column=1, pady=2, padx = 5)
angle_3 = Entry(window, width = 7)
angle_3.grid(row=7, column=2, pady=2, padx = 5)
angle_4 = Entry(window, width = 7)
angle_4.grid(row=7, column=3, pady=2, padx = 5)

angle_label = Label(window, text = "Angle Phase (left)").grid(row=8, column=0, columnspan=4, pady=2)
angle_5 = Entry(window, width = 7)
angle_5.grid(row=9, column=0, pady=2, padx = 5)
angle_6 = Entry(window, width = 7)
angle_6.grid(row=9, column=1, pady=2, padx = 5)
angle_7 = Entry(window, width = 7)
angle_7.grid(row=9, column=2, pady=2, padx = 5)
angle_8 = Entry(window, width = 7)
angle_8.grid(row=9, column=3, pady=2, padx = 5)

start_phrase_label = Label(window, text="Starting Phrase").grid(row=10, column=0, columnspan=3, pady=2)
start_phrase = Entry(window, width = 7)
start_phrase.grid(row=10, column=3, pady=2, padx=2)


init_btn = Button(window, text="Init and Run", width=30, command=init_and_run).grid(row=11,column=0, columnspan=4, pady=2)
save_btn = Button(window, text="Save", width=12, command=save_setting).grid(row=12, column=0, columnspan=2, padx=5, pady=2)
load_btn = Button(window, text="Load", width=12, command=load_setting).grid(row=12, column=2, columnspan=2, padx=5, pady=2)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "box3.txt"
load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()
