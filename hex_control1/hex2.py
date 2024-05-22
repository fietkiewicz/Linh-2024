"""
Uses hex1.xml
Usage: python hex2.py
  or
Usage: python hex2.py hex1.txt
"""
import sys # For sys.argv
import time
import numpy as np
import mujoco
import mujoco.viewer
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import tkinter.ttk
import mujocoviewer2
import mediapy as media

def init_and_run():
    filename = 'hex1.xml'
    print('filename = ', filename)
    model = mujoco.MjModel.from_xml_path(filename)
    data = mujoco.MjData(model)

    N = 100000
    i = 0
    phase = 0
    frames = []
    # Phases (start @ forward/down): -1=start, 0=back, 1=up, 2=forward, 3=down
    # Horizontal force: negative=forward, positive=backward
    # Vertical force: negative=up, positive=down
    # controllerYaw0 = [0, 0, 0, 0, 0, 0, 0, 0] # negative=forward, positive=backward
    # controllerLift0 = [0, 0, 0, 0, 0, 0, 0, 0] # negative=up, positive=down
    controllerYaw0 = [float(ctr1_1.get()), float(ctr1_2.get()), float(ctr1_3.get()), float(ctr1_4.get()), 0, 0, 0, 0] # negative=forward, positive=backward
    controllerLift0 = [float(ctr2_1.get()), float(ctr2_2.get()), float(ctr2_3.get()), float(ctr2_4.get()), 0, 0, -1, -1] # negative=up, positive=down
    controllerYaw1 = [0, 0, 0, 0, -float(ctr1_5.get()), -float(ctr1_6.get()), -float(ctr1_7.get()), -float(ctr1_8.get())] # negative=forward, positive=backward
    controllerLift1 = [0, 0, -1, -1, float(ctr2_5.get()), float(ctr2_6.get()), float(ctr2_7.get()), float(ctr2_8.get())] # negative=up, positive=down

    print("Initial values: ")
    printDebug(phase, data)
    print("Body id: ")
    print(model.geom('torso').id)

    if live.get() == True:
        viewer = mujocoviewer2.MujocoViewer(model, data)
    else:
        viewer = mujocoviewer2.MujocoViewer(model, data, 'offscreen')

    if tracking.get() == True:
        viewer.cam.fixedcamid = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING # following 
        viewer.cam.trackbodyid = int(trackbody.get()) # this is torso id
    viewer.cam.distance = int(distance.get())
    viewer.cam.elevation = int(elevation.get())

    # Hack to let it fall before moving
    for j in range(0,100):
        if (j % int(speed.get()) == 0):
            if live.get() == True:
                viewer.render()  
            else:
                img = viewer.read_pixels()
                frames.append(img)  
        j += 1
        mujoco.mj_step(model, data)

    if live.get() == True:
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
                # viewer.cam.lookat[0] += 0.01         # x,y,z offset from the object (works if trackbodyid=-1)
                # viewer.cam.lookat[1] += 0.01         # comment these line out if you want to see something magic
                # viewer.cam.lookat[2] += 0.01
            i += 1

            mujoco.mj_step(model, data)
        viewer.close()

    else:
        while viewer.is_alive and i < int(steps.get()):
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
                img = viewer.read_pixels()
                frames.append(img)
                # viewer.cam.lookat[0] += 0.01         # x,y,z offset from the object (works if trackbodyid=-1)
                # viewer.cam.lookat[1] += 0.01         # comment these line out if you want to see something magic
                # viewer.cam.lookat[2] += 0.01
            i += 1
            mujoco.mj_step(model, data)
        viewer.close()
        media.write_video(save_directory.get(), frames, fps=30)
        print("Finished!")

def printDebug(phase, data):
        print('phase = ', phase)
        print(data.joint('yaw_mid_right').qpos[0], ", ", data.joint('yaw_mid_left').qpos[0])

def save_setting():
    fileload = fd.asksaveasfile(mode = "w+")
    save = []

    save.append(str(ctr1_1.get()))
    save.append(str(ctr1_2.get()))
    save.append(str(ctr1_3.get()))
    save.append(str(ctr1_4.get()))
    save.append(str(ctr1_5.get()))
    save.append(str(ctr1_6.get()))
    save.append(str(ctr1_7.get()))
    save.append(str(ctr1_8.get()))
    save.append(str(ctr2_1.get()))
    save.append(str(ctr2_2.get()))
    save.append(str(ctr2_3.get()))
    save.append(str(ctr2_4.get()))
    save.append(str(ctr2_5.get()))
    save.append(str(ctr2_6.get()))
    save.append(str(ctr2_7.get()))
    save.append(str(ctr2_8.get()))
    
    save.append(str(angle_1.get()))
    save.append(str(angle_2.get()))
    save.append(str(angle_3.get()))
    save.append(str(angle_4.get()))
    save.append(str(angle_5.get()))
    save.append(str(angle_6.get()))
    save.append(str(angle_7.get()))
    save.append(str(angle_8.get()))

    save.append(str(speed.get()))
    save.append(str(save_directory.get()))
    save.append(str(live.get()))
    save.append(str(steps.get()))
    save.append(str(tracking.get()))

    save.append(str(elevation.get()))
    save.append(str(distance.get()))
    save.append(str(trackbody.get()))

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
    ctr1_5.delete(0, tk.END)
    ctr1_6.delete(0, tk.END)
    ctr1_7.delete(0, tk.END)
    ctr1_8.delete(0, tk.END)
    ctr2_1.delete(0, tk.END)
    ctr2_2.delete(0, tk.END)
    ctr2_3.delete(0, tk.END)
    ctr2_4.delete(0, tk.END)
    ctr2_5.delete(0, tk.END)
    ctr2_6.delete(0, tk.END)
    ctr2_7.delete(0, tk.END)
    ctr2_8.delete(0, tk.END)

    angle_1.delete(0, tk.END)
    angle_2.delete(0, tk.END)
    angle_3.delete(0, tk.END)
    angle_4.delete(0, tk.END)
    angle_5.delete(0, tk.END)
    angle_6.delete(0, tk.END)
    angle_7.delete(0, tk.END)
    angle_8.delete(0, tk.END)

    speed.delete(0, tk.END)
    save_directory.delete(0, tk.END)
    steps.delete(0, tk.END)

    elevation.delete(0, tk.END)
    distance.delete(0, tk.END)
    trackbody.delete(0, tk.END)

    ctr1_1.insert(0, float(save[0].strip()))
    ctr1_2.insert(0, float(save[1].strip()))
    ctr1_3.insert(0, float(save[2].strip()))
    ctr1_4.insert(0, float(save[3].strip()))
    ctr1_5.insert(0, float(save[4].strip()))
    ctr1_6.insert(0, float(save[5].strip()))
    ctr1_7.insert(0, float(save[6].strip()))
    ctr1_8.insert(0, float(save[7].strip()))
    ctr2_1.insert(0, float(save[8].strip()))
    ctr2_2.insert(0, float(save[9].strip()))
    ctr2_3.insert(0, float(save[10].strip()))
    ctr2_4.insert(0, float(save[11].strip()))
    ctr2_5.insert(0, float(save[12].strip()))
    ctr2_6.insert(0, float(save[13].strip()))
    ctr2_7.insert(0, float(save[14].strip()))
    ctr2_8.insert(0, float(save[15].strip()))

    angle_1.insert(0, float(save[16].strip()))
    angle_2.insert(0, float(save[17].strip()))
    angle_3.insert(0, float(save[18].strip()))
    angle_4.insert(0, float(save[19].strip()))
    angle_5.insert(0, float(save[20].strip()))
    angle_6.insert(0, float(save[21].strip()))
    angle_7.insert(0, float(save[22].strip()))
    angle_8.insert(0, float(save[23].strip()))

    speed.insert(0, int(save[24].strip()))
    save_directory.insert(0, save[25].strip())

    if save[26].strip() == "True":
        live.set(True)
    else:
        live.set(False)

    steps.insert(0, int(save[27].strip()))

    if save[28].strip() == "True":
        tracking.set(True)
    else:
        tracking.set(False)

    elevation.insert(0, int(save[29].strip()))
    distance.insert(0, int(save[30].strip()))
    trackbody.insert(0, int(save[31].strip()))

window = tk.Tk()
window.title("Hexapod")
window.geometry("500x275")

ctrl1_label = Label(window, text="Horizontal (yaw) Force (4 phases)").grid(row=0, column=1, columnspan=4, pady=2)
ctrl1_label_l = Label(window, text="Left").grid(row=1, column=0)
ctrl1_label_r = Label(window, text="Right").grid(row=2, column=0)
ctr1_1 = Entry(window, width = 7)
ctr1_1.grid(row=1, column=1, pady=2, padx = 5)
ctr1_2 = Entry(window, width = 7)
ctr1_2.grid(row=1, column=2, pady=2, padx = 5)
ctr1_3 = Entry(window, width = 7)
ctr1_3.grid(row=1, column=3, pady=2, padx = 5)
ctr1_4 = Entry(window, width = 7)
ctr1_4.grid(row=1, column=4, pady=2, padx = 5)
ctr1_5 = Entry(window, width = 7)
ctr1_5.grid(row=2, column=1, pady=2, padx = 5)
ctr1_6 = Entry(window, width = 7)
ctr1_6.grid(row=2, column=2, pady=2, padx = 5)
ctr1_7 = Entry(window, width = 7)
ctr1_7.grid(row=2, column=3, pady=2, padx = 5)
ctr1_8 = Entry(window, width = 7)
ctr1_8.grid(row=2, column=4, pady=2, padx = 5)

ctrl2_label = Label(window, text="Vertical (pitch) Force (4 phases)").grid(row=3, column=1, columnspan=4, pady=2)
ctrl2_label_l = Label(window, text="Left").grid(row=4, column=0)
ctrl2_label_r = Label(window, text="Right").grid(row=5, column=0)
ctr2_1 = Entry(window, width = 7)
ctr2_1.grid(row=4, column=1, pady=2, padx = 5)
ctr2_2 = Entry(window, width = 7)
ctr2_2.grid(row=4, column=2, pady=2, padx = 5)
ctr2_3 = Entry(window, width = 7)
ctr2_3.grid(row=4, column=3, pady=2, padx = 5)
ctr2_4 = Entry(window, width = 7)
ctr2_4.grid(row=4, column=4, pady=2, padx = 5)
ctr2_5 = Entry(window, width = 7)
ctr2_5.grid(row=5, column=1, pady=2, padx = 5)
ctr2_6 = Entry(window, width = 7)
ctr2_6.grid(row=5, column=2, pady=2, padx = 5)
ctr2_7 = Entry(window, width = 7)
ctr2_7.grid(row=5, column=3, pady=2, padx = 5)
ctr2_8 = Entry(window, width = 7)
ctr2_8.grid(row=5, column=4, pady=2, padx = 5)

angle_label = Label(window, text = "Angle Phase").grid(row=6, column=1, columnspan=4, pady=2)
ctrl2_label_l = Label(window, text="Right").grid(row=7, column=0)
ctrl2_label_r = Label(window, text="Left").grid(row=8, column=0)

angle_1 = Entry(window, width = 7)
angle_1.grid(row=7, column=1, pady=2, padx = 5)
angle_2 = Entry(window, width = 7)
angle_2.grid(row=7, column=2, pady=2, padx = 5)
angle_3 = Entry(window, width = 7)
angle_3.grid(row=7, column=3, pady=2, padx = 5)
angle_4 = Entry(window, width = 7)
angle_4.grid(row=7, column=4, pady=2, padx = 5)
angle_5 = Entry(window, width = 7)
angle_5.grid(row=8, column=1, pady=2, padx = 5)
angle_6 = Entry(window, width = 7)
angle_6.grid(row=8, column=2, pady=2, padx = 5)
angle_7 = Entry(window, width = 7)
angle_7.grid(row=8, column=3, pady=2, padx = 5)
angle_8 = Entry(window, width = 7)
angle_8.grid(row=8, column=4, pady=2, padx = 5)

tkinter.ttk.Separator(window, orient=VERTICAL).grid(column=5, row=0, rowspan=9, sticky='ns', padx=2)

speed_label = Label(window, text="Speed").grid(row=0, column=6, pady=2, sticky=W)
speed = Entry(window, width = 7)
speed.grid(row=0, column=7, sticky=W)

live = BooleanVar()
live_video = Radiobutton(window, text="Live", variable=live, value=True).grid(row=1, column=6, sticky=W)
save_video = Radiobutton(window, text="Save", variable=live, value=False).grid(row=2, column=6, sticky=W)
save_directory = Entry(window, width=23)
save_directory.grid(row=2, column=7, columnspan=2, sticky=W)
steps_label = Label(window, text="Steps").grid(row=3, column=7, pady=2)
steps = Entry(window, width=7)
steps.grid(row=3, column=8)

tracking = BooleanVar()
camera_label = Label(window, text="Camera").grid(row=4, column=6, pady=2, sticky=W)
track_camera = Radiobutton(window, text="Tracking", variable=tracking, value=True).grid(row=5, column=6, sticky=W)
fixed_camera = Radiobutton(window, text="Fixed", variable=tracking, value=False).grid(row=6, column=6, sticky=W)

elevation_label = Label(window, text="Elevation").grid(row=7, column=6)
distance_label = Label(window, text="Distance").grid(row=7, column=7)
trackbody_label = Label(window, text="Trackbody").grid(row=7, column=8)

elevation = Entry(window, width=7)
elevation.grid(row=8, column=6)
distance = Entry(window, width=7)
distance.grid(row=8, column=7)
trackbody = Entry(window, width=7)
trackbody.grid(row=8, column=8)

init_btn = Button(window, text="Init and Run", width=45, command=init_and_run).grid(row=9, column=0, columnspan=7, pady=2)
save_btn = Button(window, text="Save", width=10, command=save_setting).grid(row=9, column=7, columnspan=1)
load_btn = Button(window, text="Load", width=10, command=load_setting).grid(row=9, column=8, columnspan=1)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "default.txt"

load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()
