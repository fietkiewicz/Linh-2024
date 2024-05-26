import sys
import mujoco
import mujoco.viewer
import mujocoviewer2
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import glfw

def init_and_run():
    filename = 'hex1.xml'
    model = mujoco.MjModel.from_xml_path(filename)
    data = mujoco.MjData(model)

    N = 100000
    i = 0
    phase = 0

    viewer = mujocoviewer2.MujocoViewer(model, data)
    if tracking.get() == True:
        viewer.cam.fixedcamid = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING # following 
        viewer.cam.trackbodyid = int(trackbody.get()) # this is torso id
    viewer.cam.distance = int(distance.get())
    viewer.cam.elevation = int(elevation.get())

    controllerLift0 = [-1, -1, 1, 0, 0, 0, -1, -1]
    controllerLift1 = [0, 0, -1, -1, -1, -1, 1, 0]
    
    controllerYaw0 = [0, 1, 0, -1, 0, 0, 0, 0]
    controllerYaw1 = [0, 0, 0, 0, 0, -1, 0, 1]

    angle = {
        "left": [-0.3, -0.5, 0, 0.5, -0.3, 0.01, 0, -0.01],
        "right": [-0.3, 0.01, 0, -0.01, -0.3, 0.5, 0, -0.5],
        "straight": [-0.3, 0, 0, 0, -0.3, 0, 0, 0]
    }

    for j in range(0, 100):
        if j % int(speed.get()) == 0:
            viewer.render()
        mujoco.mj_step(model, data)

    while viewer.is_alive and i < N:

        if phase == 0 and data.joint('pitch_mid_right').qpos[0] <= angle[viewer._movement][0]:
            phase = 1
        elif phase == 1 and data.joint('yaw_mid_right').qpos[0] >= angle[viewer._movement][1]:
            phase = 2
        elif phase == 2 and data.joint('pitch_mid_right').qpos[0] >= angle[viewer._movement][2]:
            phase = 3
        elif phase == 3 and data.joint('yaw_mid_right').qpos[0] <= angle[viewer._movement][3]:
            phase = 4
        elif phase == 4 and data.joint('pitch_mid_left').qpos[0] <= angle[viewer._movement][4]:
            phase = 5
        elif phase == 5 and data.joint('yaw_mid_left').qpos[0] <= angle[viewer._movement][5]:
            phase = 6
        elif phase == 6 and data.joint('pitch_mid_left').qpos[0] >= angle[viewer._movement][6]:
            phase = 7
        elif phase == 7 and data.joint('yaw_mid_left').qpos[0] >= angle[viewer._movement][7]:
            phase = 0

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

        if i % int(speed.get()) == 0:
            viewer.render()
            # print(viewer._movement)
        i += 1
        mujoco.mj_step(model, data)
    viewer.close()

def save_setting():

    fileload = fd.asksaveasfile(mode = "w+")
    save = []
    save.append(str(tracking.get()))
    save.append(str(speed.get()))
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

    speed.delete(0, tk.END)
    elevation.delete(0, tk.END)
    distance.delete(0, tk.END)
    trackbody.delete(0, tk.END)

    if save[0].strip() == "True":
        tracking.set(True)
    else:
        tracking.set(False)

    speed.insert(0, int(save[1].strip()))
    elevation.insert(0, int(save[2].strip()))
    distance.insert(0, int(save[3].strip()))
    trackbody.insert(0, int(save[4].strip()))

window = tk.Tk()
window.title("Hexapod Control")
window.geometry("240x140")

tracking = BooleanVar()
camera_label = Label(window, text="Camera").grid(row=0, column=0, pady=2, sticky=W)
track_camera = Radiobutton(window, text="Tracking", variable=tracking, value=True).grid(row=0, column=2, sticky=W)
fixed_camera = Radiobutton(window, text="Fixed", variable=tracking, value=False).grid(row=0, column=3, sticky=W)

speed_label = Label(window, text="Speed").grid(row=1, column=0)
elevation_label = Label(window, text="Elevation").grid(row=1, column=1)
distance_label = Label(window, text="Distance").grid(row=1, column=2)
trackbody_label = Label(window, text="Trackbody").grid(row=1, column=3)

speed = Entry(window, width=7)
speed.grid(row=2, column=0, pady=2)
elevation = Entry(window, width=7)
elevation.grid(row=2, column=1, pady=2)
distance = Entry(window, width=7)
distance.grid(row=2, column=2, pady=2)
trackbody = Entry(window, width=7)
trackbody.grid(row=2, column=3, pady=2)

init_btn = Button(window, text="Init and Run", command=init_and_run, width=28).grid(row=3, column=0, columnspan=4)
save_btn = Button(window, text="Save", width=14, command=save_setting).grid(row=4, column=0, columnspan=2)
load_btn = Button(window, text="Load", width=14, command=load_setting).grid(row=4, column=2, columnspan=2)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "default_control.txt"

load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()