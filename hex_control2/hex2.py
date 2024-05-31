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
    controllerYawlf = [float(ctrlf_1.get()), float(ctrlf_2.get()), float(ctrlf_3.get()), float(ctrlf_4.get()), 0, 0, 0, 0] # negative=forward, positive=backward
    controllerYawlb = [float(ctrlb_1.get()), float(ctrlb_2.get()), float(ctrlb_3.get()), float(ctrlb_4.get()), 0, 0, 0, 0] # negative=forward, positive=backward
    controllerYawrm = [float(ctrrm_1.get()), float(ctrrm_2.get()), float(ctrrm_3.get()), float(ctrrm_4.get()), 0, 0, 0, 0] # negative=forward, positive=backward
    controllerYawrf = [0, 0, 0, 0, -float(ctrrf_1.get()), -float(ctrrf_2.get()), -float(ctrrf_3.get()), -float(ctrrf_4.get())] # negative=forward, positive=backward
    controllerYawrb = [0, 0, 0, 0, -float(ctrrb_1.get()), -float(ctrrb_2.get()), -float(ctrrb_3.get()), -float(ctrrb_4.get())] # negative=forward, positive=backward
    controllerYawlm = [0, 0, 0, 0, -float(ctrlm_1.get()), -float(ctrlm_2.get()), -float(ctrlm_3.get()), -float(ctrlm_4.get())] # negative=forward, positive=backward
    controllerLift0 = [float(ctr2_1.get()), float(ctr2_2.get()), float(ctr2_3.get()), float(ctr2_4.get()), 0, 0, -1, -1] # negative=up, positive=down
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
            if phase == 0 and data.joint('pitch_front_left').qpos[0] <= float(angle_1.get()):
                phase = 1
                printDebug(phase, data)
            elif phase == 1 and data.joint('yaw_front_left').qpos[0] <= float(angle_2.get()):
                phase = 2
                printDebug(phase, data)
            elif phase == 2 and data.joint('pitch_front_left').qpos[0] >= float(angle_3.get()):
                phase = 3
                printDebug(phase, data)
            elif phase == 3 and data.joint('yaw_front_left').qpos[0] >= float(angle_4.get()):
                phase = 4
                printDebug(phase, data)
            elif phase == 4 and data.joint('pitch_front_right').qpos[0] <= float(angle_5.get()):
                phase = 5
                printDebug(phase, data)
            elif phase == 5 and data.joint('yaw_front_right').qpos[0] >= float(angle_6.get()):
                phase = 6
                printDebug(phase, data)
            elif phase == 6 and data.joint('pitch_front_right').qpos[0] >= float(angle_7.get()):
                phase = 7
                printDebug(phase, data)
            elif phase == 7 and data.joint('yaw_front_right').qpos[0] <= float(angle_8.get()):
                phase = 0
                printDebug(phase, data)

            # Apply force
            data.ctrl[0] = controllerYawlf[phase] * (-1) # yaw_front_left (GROUP 0)
            data.ctrl[1] = controllerLift0[phase] # lift_front_left (GROUP 0)
            data.ctrl[3] = controllerYawrf[phase] * (-1) # yaw_front_right (GROUP 1)
            data.ctrl[4] = controllerLift1[phase] # lift_front_right (GROUP 1)
            data.ctrl[6] = controllerYawlm[phase] # yaw_mid_left (GROUP 1)
            data.ctrl[7] = controllerLift1[phase] # lift_mid_left (GROUP 1)
            data.ctrl[9] = controllerYawrm[phase] # yaw_mid_right (GROUP 0)
            data.ctrl[10] = controllerLift0[phase] # lift_mid_right (GROUP 0)
            data.ctrl[12] = controllerYawrb[phase] * (-1) # yaw_back_right (GROUP 1)
            data.ctrl[13] = controllerLift1[phase] # lift_back_right (GROUP 1)
            data.ctrl[15] = controllerYawlb[phase] * (-1) # yaw_back_left (GROUP 0)
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
            elif phase == 1 and data.joint('yaw_mid_right').qpos[0] >= float(angle_2.get()):
                phase = 2
            elif phase == 2 and data.joint('pitch_mid_right').qpos[0] >= float(angle_3.get()):
                phase = 3
            elif phase == 3 and data.joint('yaw_mid_right').qpos[0] <= float(angle_4.get()):
                phase = 4
            elif phase == 4 and data.joint('pitch_mid_left').qpos[0] <= float(angle_5.get()):
                phase = 5
            elif phase == 5 and data.joint('yaw_mid_left').qpos[0] <= float(angle_6.get()):
                phase = 6
            elif phase == 6 and data.joint('pitch_mid_left').qpos[0] >= float(angle_7.get()):
                phase = 7
            elif phase == 7 and data.joint('yaw_mid_left').qpos[0] >= float(angle_8.get()):
                phase = 0

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
        print(data.joint('yaw_front_left').qpos[0], ", ", data.joint('yaw_front_right').qpos[0])

def save_setting():
    fileload = fd.asksaveasfile(mode = "w+")
    save = []

    save.append(str(ctrlf_1.get()))
    save.append(str(ctrlf_2.get()))
    save.append(str(ctrlf_3.get()))
    save.append(str(ctrlf_4.get()))

    save.append(str(ctrlm_1.get()))
    save.append(str(ctrlm_2.get()))
    save.append(str(ctrlm_3.get()))
    save.append(str(ctrlm_4.get()))

    save.append(str(ctrlb_1.get()))
    save.append(str(ctrlb_2.get()))
    save.append(str(ctrlb_3.get()))
    save.append(str(ctrlb_4.get()))

    save.append(str(ctrrf_1.get()))
    save.append(str(ctrrf_2.get()))
    save.append(str(ctrrf_3.get()))
    save.append(str(ctrrf_4.get()))

    save.append(str(ctrrm_1.get()))
    save.append(str(ctrrm_2.get()))
    save.append(str(ctrrm_3.get()))
    save.append(str(ctrrm_4.get()))

    save.append(str(ctrrb_1.get()))
    save.append(str(ctrrb_2.get()))
    save.append(str(ctrrb_3.get()))
    save.append(str(ctrrb_4.get()))

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

    ctrlf_1.delete(0, tk.END)
    ctrlf_2.delete(0, tk.END)
    ctrlf_3.delete(0, tk.END)
    ctrlf_4.delete(0, tk.END)

    ctrlm_1.delete(0, tk.END)
    ctrlm_2.delete(0, tk.END)
    ctrlm_3.delete(0, tk.END)
    ctrlm_4.delete(0, tk.END)

    ctrlb_1.delete(0, tk.END)
    ctrlb_2.delete(0, tk.END)
    ctrlb_3.delete(0, tk.END)
    ctrlb_4.delete(0, tk.END)
    
    ctrrf_1.delete(0, tk.END)
    ctrrf_2.delete(0, tk.END)
    ctrrf_3.delete(0, tk.END)
    ctrrf_4.delete(0, tk.END)

    ctrrm_1.delete(0, tk.END)
    ctrrm_2.delete(0, tk.END)
    ctrrm_3.delete(0, tk.END)
    ctrrm_4.delete(0, tk.END)
    
    ctrrb_1.delete(0, tk.END)
    ctrrb_2.delete(0, tk.END)
    ctrrb_3.delete(0, tk.END)
    ctrrb_4.delete(0, tk.END)


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

    ctrlf_1.insert(0, float(save[0].strip()))
    ctrlf_2.insert(0, float(save[1].strip()))
    ctrlf_3.insert(0, float(save[2].strip()))
    ctrlf_4.insert(0, float(save[3].strip()))

    ctrlm_1.insert(0, float(save[4].strip()))
    ctrlm_2.insert(0, float(save[5].strip()))
    ctrlm_3.insert(0, float(save[6].strip()))
    ctrlm_4.insert(0, float(save[7].strip()))

    ctrlb_1.insert(0, float(save[8].strip()))
    ctrlb_2.insert(0, float(save[9].strip()))
    ctrlb_3.insert(0, float(save[10].strip()))
    ctrlb_4.insert(0, float(save[11].strip()))
    
    ctrrf_1.insert(0, float(save[12].strip()))
    ctrrf_2.insert(0, float(save[13].strip()))
    ctrrf_3.insert(0, float(save[14].strip()))
    ctrrf_4.insert(0, float(save[15].strip()))

    ctrrm_1.insert(0, float(save[16].strip()))
    ctrrm_2.insert(0, float(save[17].strip()))
    ctrrm_3.insert(0, float(save[18].strip()))
    ctrrm_4.insert(0, float(save[19].strip()))
    
    ctrrb_1.insert(0, float(save[20].strip()))
    ctrrb_2.insert(0, float(save[21].strip()))
    ctrrb_3.insert(0, float(save[22].strip()))
    ctrrb_4.insert(0, float(save[23].strip()))

    ctr2_1.insert(0, float(save[24].strip()))
    ctr2_2.insert(0, float(save[25].strip()))
    ctr2_3.insert(0, float(save[26].strip()))
    ctr2_4.insert(0, float(save[27].strip()))
    ctr2_5.insert(0, float(save[28].strip()))
    ctr2_6.insert(0, float(save[29].strip()))
    ctr2_7.insert(0, float(save[30].strip()))
    ctr2_8.insert(0, float(save[31].strip()))

    angle_1.insert(0, float(save[32].strip()))
    angle_2.insert(0, float(save[33].strip()))
    angle_3.insert(0, float(save[34].strip()))
    angle_4.insert(0, float(save[35].strip()))
    angle_5.insert(0, float(save[36].strip()))
    angle_6.insert(0, float(save[37].strip()))
    angle_7.insert(0, float(save[38].strip()))
    angle_8.insert(0, float(save[39].strip()))

    speed.insert(0, int(save[40].strip()))
    save_directory.insert(0, save[41].strip())

    if save[42].strip() == "True":
        live.set(True)
    else:
        live.set(False)

    steps.insert(0, int(save[43].strip()))

    if save[44].strip() == "True":
        tracking.set(True)
    else:
        tracking.set(False)

    elevation.insert(0, int(save[45].strip()))
    distance.insert(0, int(save[46].strip()))
    trackbody.insert(0, int(save[47].strip()))

window = tk.Tk()
window.title("Hexapod")
window.geometry("650x325")

ctrl1_label = Label(window, text="Horizontal (yaw) Force (4 phases)").grid(row=0, column=1, columnspan=4, pady=2)
ctrl1_label_lf = Label(window, text="Left/Front").grid(row=1, column=0)
ctrl1_label_lm = Label(window, text="Left/Middle").grid(row=2, column=0)
ctrl1_label_lb = Label(window, text="Left/Back").grid(row=3, column=0)
ctrl1_label_rf = Label(window, text="Right/Front").grid(row=4, column=0)
ctrl1_label_rm = Label(window, text="Right/Middle").grid(row=5, column=0)
ctrl1_label_rb = Label(window, text="Right/Back").grid(row=6, column=0)
ctrlf_1 = Entry(window, width = 7)
ctrlf_1.grid(row=1, column=1, pady=2, padx = 5)
ctrlf_2 = Entry(window, width = 7)
ctrlf_2.grid(row=1, column=2, pady=2, padx = 5)
ctrlf_3 = Entry(window, width = 7)
ctrlf_3.grid(row=1, column=3, pady=2, padx = 5)
ctrlf_4 = Entry(window, width = 7)
ctrlf_4.grid(row=1, column=4, pady=2, padx = 5)

ctrlm_1 = Entry(window, width = 7)
ctrlm_1.grid(row=2, column=1, pady=2, padx = 5)
ctrlm_2 = Entry(window, width = 7)
ctrlm_2.grid(row=2, column=2, pady=2, padx = 5)
ctrlm_3 = Entry(window, width = 7)
ctrlm_3.grid(row=2, column=3, pady=2, padx = 5)
ctrlm_4 = Entry(window, width = 7)
ctrlm_4.grid(row=2, column=4, pady=2, padx = 5)

ctrlb_1 = Entry(window, width = 7)
ctrlb_1.grid(row=3, column=1, pady=2, padx = 5)
ctrlb_2 = Entry(window, width = 7)
ctrlb_2.grid(row=3, column=2, pady=2, padx = 5)
ctrlb_3 = Entry(window, width = 7)
ctrlb_3.grid(row=3, column=3, pady=2, padx = 5)
ctrlb_4 = Entry(window, width = 7)
ctrlb_4.grid(row=3, column=4, pady=2, padx = 5)

ctrrf_1 = Entry(window, width = 7)
ctrrf_1.grid(row=4, column=1, pady=2, padx = 5)
ctrrf_2 = Entry(window, width = 7)
ctrrf_2.grid(row=4, column=2, pady=2, padx = 5)
ctrrf_3 = Entry(window, width = 7)
ctrrf_3.grid(row=4, column=3, pady=2, padx = 5)
ctrrf_4 = Entry(window, width = 7)
ctrrf_4.grid(row=4, column=4, pady=2, padx = 5)

ctrrm_1 = Entry(window, width = 7)
ctrrm_1.grid(row=5, column=1, pady=2, padx = 5)
ctrrm_2 = Entry(window, width = 7)
ctrrm_2.grid(row=5, column=2, pady=2, padx = 5)
ctrrm_3 = Entry(window, width = 7)
ctrrm_3.grid(row=5, column=3, pady=2, padx = 5)
ctrrm_4 = Entry(window, width = 7)
ctrrm_4.grid(row=5, column=4, pady=2, padx = 5)

ctrrb_1 = Entry(window, width = 7)
ctrrb_1.grid(row=6, column=1, pady=2, padx = 5)
ctrrb_2 = Entry(window, width = 7)
ctrrb_2.grid(row=6, column=2, pady=2, padx = 5)
ctrrb_3 = Entry(window, width = 7)
ctrrb_3.grid(row=6, column=3, pady=2, padx = 5)
ctrrb_4 = Entry(window, width = 7)
ctrrb_4.grid(row=6, column=4, pady=2, padx = 5)

ctrl2_label = Label(window, text="Vertical (pitch) Force (4 phases)").grid(row=7, column=1, columnspan=4, pady=2)
ctrl2_label_l = Label(window, text="Left").grid(row=8, column=0)
ctrl2_label_r = Label(window, text="Right").grid(row=9, column=0)
ctr2_1 = Entry(window, width = 7)
ctr2_1.grid(row=8, column=1, pady=2, padx = 5)
ctr2_2 = Entry(window, width = 7)
ctr2_2.grid(row=8, column=2, pady=2, padx = 5)
ctr2_3 = Entry(window, width = 7)
ctr2_3.grid(row=8, column=3, pady=2, padx = 5)
ctr2_4 = Entry(window, width = 7)
ctr2_4.grid(row=8, column=4, pady=2, padx = 5)
ctr2_5 = Entry(window, width = 7)
ctr2_5.grid(row=9, column=1, pady=2, padx = 5)
ctr2_6 = Entry(window, width = 7)
ctr2_6.grid(row=9, column=2, pady=2, padx = 5)
ctr2_7 = Entry(window, width = 7)
ctr2_7.grid(row=9, column=3, pady=2, padx = 5)
ctr2_8 = Entry(window, width = 7)
ctr2_8.grid(row=9, column=4, pady=2, padx = 5)

angle_label = Label(window, text = "Angle Phase").grid(row=10, column=1, columnspan=4, pady=2)
ctrl2_label_l = Label(window, text="Right").grid(row=11, column=0)
ctrl2_label_r = Label(window, text="Left").grid(row=12, column=0)

angle_1 = Entry(window, width = 7)
angle_1.grid(row=11, column=1, pady=2, padx = 5)
angle_2 = Entry(window, width = 7)
angle_2.grid(row=11, column=2, pady=2, padx = 5)
angle_3 = Entry(window, width = 7)
angle_3.grid(row=11, column=3, pady=2, padx = 5)
angle_4 = Entry(window, width = 7)
angle_4.grid(row=11, column=4, pady=2, padx = 5)
angle_5 = Entry(window, width = 7)
angle_5.grid(row=12, column=1, pady=2, padx = 5)
angle_6 = Entry(window, width = 7)
angle_6.grid(row=12, column=2, pady=2, padx = 5)
angle_7 = Entry(window, width = 7)
angle_7.grid(row=12, column=3, pady=2, padx = 5)
angle_8 = Entry(window, width = 7)
angle_8.grid(row=12, column=4, pady=2, padx = 5)

tkinter.ttk.Separator(window, orient=VERTICAL).grid(column=5, row=0, rowspan=12, sticky='ns', padx=2)

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

init_btn = Button(window, text="Init and Run", width=45, command=init_and_run).grid(row=9, column=6, columnspan=3, pady=2)
save_btn = Button(window, text="Save", width=10, command=save_setting).grid(row=10, column=6, columnspan=1)
load_btn = Button(window, text="Load", width=10, command=load_setting).grid(row=10, column=7, columnspan=1)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "default.txt"

load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()
