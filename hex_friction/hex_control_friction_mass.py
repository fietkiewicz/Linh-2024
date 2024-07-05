import sys
import mujoco
import mujoco.viewer
import mujocoviewer2
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import glfw
import xml.etree.ElementTree as ET

def init_and_run():
    filename = 'hex1.xml'
    tree = ET.parse(filename)

    for geom in tree.iter('geom'):
        if friction.get() and geom.get('name') in ["toe_front_left", "toe_front_right", "toe_mid_left", "toe_mid_right", "toe_back_left", "toe_back_right"]:
            geom.set("friction", friction.get())
        if friction2.get() and geom.get('name') == "floor":
            geom.set("friction", friction2.get())
        if mass.get() and geom.get('name') == "torso":
            geom.set("mass", mass.get())

    tree.write('output.xml', encoding="utf-8")

    model = mujoco.MjModel.from_xml_path('output.xml')
    data = mujoco.MjData(model)

    N = 100000
    i = 0
    phase = 0

    viewer = mujocoviewer2.MujocoViewer(model, data)
    if tracking.get() == True:
        viewer.cam.fixedcamid = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING # following 
        viewer.cam.trackbodyid = int(trackbody.get()) # this is torso id
    viewer.cam.distance = float(distance.get())
    viewer.cam.elevation = int(elevation.get())

    controllerYawlf = {
        "left" : [0, 0, 0, 0, 0, 0, 0, 0],
        "right": [0, 1, 0, -1, 0, 0, 0, 0],
        "straight": [0, 1, 0, -1, 0, 0, 0, 0]
    }
    controllerYawlb = {
        "left" : [0, 0, 0, 0, 0, 0, 0, 0],
        "right": [0, 1, 0, -1, 0, 0, 0, 0],
        "straight": [0, 1, 0, -1, 0, 0, 0, 0]
    }
    controllerYawrm = {
        "left" : [0, 1, 0, -1, 0, 0, 0, 0],
        "right": [0, 0, 0, 0, 0, 0, 0, 0],
        "straight": [0, 1, 0, -1, 0, 0, 0, 0]
    }
    controllerYawrf = {
        "left" : [0, 0, 0, 0, 0, -1, 0, 1],
        "right": [0, 0, 0, 0, 0, 0, 0, 0],
        "straight": [0, 0, 0, 0, 0, -1, 0, 1]
    }
    controllerYawrb = {
        "left" : [0, 0, 0, 0, 0, -1, 0, 1],
        "right": [0, 0, 0, 0, 0, 0, 0, 0],
        "straight": [0, 0, 0, 0, 0, -1, 0, 1]
    }
    controllerYawlm = {
        "left" : [0, 0, 0, 0, 0, 0, 0, 0],
        "right": [0, 0, 0, 0, 0, -1, 0, 1],
        "straight": [0, 0, 0, 0, 0, -1, 0, 1]
    }
    
    controllerLift0 = [-1, -1, 1, 0, 0, 0, -1, -1]
    controllerLift1 = [0, 0, -1, -1, -1, -1, 1, 0]

    angle = {
        "left": [-0.3, 0.5, 0, -0.5, -0.3, 0.01, 0, -0.01],
        "right": [-0.3, 0.01, 0, -0.01, -0.3, -0.5, 0, 0.5],
        "straight": [-0.3, 0, 0, 0, -0.3, 0, 0, 0]
    }

    for j in range(0, 100):
        if j % int(speed.get()) == 0:
            viewer.render()
        mujoco.mj_step(model, data)

    while viewer.is_alive and i < N:

        if phase == 0 and data.joint('pitch_front_left').qpos[0] <= angle[viewer._movement][0]:
            phase = 1
        elif phase == 1 and data.joint('yaw_front_left').qpos[0] <= angle[viewer._movement][1]:
            phase = 2
        elif phase == 2 and data.joint('pitch_front_left').qpos[0] >= angle[viewer._movement][2]:
            phase = 3
        elif phase == 3 and data.joint('yaw_front_left').qpos[0] >= angle[viewer._movement][3]:
            phase = 4
        elif phase == 4 and data.joint('pitch_front_right').qpos[0] <= angle[viewer._movement][4]:
            phase = 5
        elif phase == 5 and data.joint('yaw_front_right').qpos[0] >= angle[viewer._movement][5]:
            phase = 6
        elif phase == 6 and data.joint('pitch_front_right').qpos[0] >= angle[viewer._movement][6]:
            phase = 7
        elif phase == 7 and data.joint('yaw_front_right').qpos[0] <= angle[viewer._movement][7]:
            phase = 0

        data.ctrl[0] = controllerYawlf[viewer._movement][phase] * (-1) # yaw_front_left (GROUP 0)
        data.ctrl[1] = controllerLift0[phase] # lift_front_left (GROUP 0)
        data.ctrl[3] = controllerYawrf[viewer._movement][phase] * (-1) # yaw_front_right (GROUP 1)
        data.ctrl[4] = controllerLift1[phase] # lift_front_right (GROUP 1)
        data.ctrl[6] = controllerYawlm[viewer._movement][phase] # yaw_mid_left (GROUP 1)
        data.ctrl[7] = controllerLift1[phase] # lift_mid_left (GROUP 1)
        data.ctrl[9] = controllerYawrm[viewer._movement][phase] # yaw_mid_right (GROUP 0)
        data.ctrl[10] = controllerLift0[phase] # lift_mid_right (GROUP 0)
        data.ctrl[12] = controllerYawrb[viewer._movement][phase] * (-1) # yaw_back_right (GROUP 1)
        data.ctrl[13] = controllerLift1[phase] # lift_back_right (GROUP 1)
        data.ctrl[15] = controllerYawlb[viewer._movement][phase] * (-1) # yaw_back_left (GROUP 0)
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
    save.append(str(live.get()))
    save.append(str(video_directory.get()))
    save.append(str(step.get()))
    save.append(str(friction.get()))
    save.append(str(friction2.get()))
    save.append(str(mass.get()))
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
    
    friction.delete(0, tk.END)
    friction2.delete(0, tk.END)
    speed.delete(0, tk.END)
    elevation.delete(0, tk.END)
    distance.delete(0, tk.END)
    trackbody.delete(0, tk.END)
    video_directory.delete(0, tk.END)
    step.delete(0, tk.END)

    if save[0].strip() == "True":
        tracking.set(True)
    else:
        tracking.set(False)

    speed.insert(0, int(save[1].strip()))
    elevation.insert(0, int(save[2].strip()))
    distance.insert(0, float(save[3].strip()))
    trackbody.insert(0, int(save[4].strip()))

    if save[5].strip() == "True":
        live.set(True)
    else:
        live.set(False)

    video_directory.insert(0, save[6].strip())
    step.insert(0, int(save[7].strip()))
    friction.insert(0, float(save[8].strip()))
    friction2.insert(0, float(save[9].strip()))
    mass.insert(0, float(save[10].strip()))

window = tk.Tk()
window.title("Hexapod Control")
window.geometry("350x200")

friction_label = Label(window, text="Friction (cube)").grid(row=0, column=0, pady=2)
friction = Entry(window, width=7)
friction.grid(row=0, column=1, pady=2)

friction_label_2 = Label(window, text="Friction (floor)").grid(row=0, column=2, pady=2)
friction2 = Entry(window, width=7)
friction2.grid(row=0, column=3, pady=2)

mass_label = Label(window, text="Mass").grid(row=1, column=0, pady=2)
mass = Entry(window, width = 7)
mass.grid(row=1, column=1, pady=2)

note = Label(window, text="Remove value to set back to default").grid(row=1, column=2, columnspan=2)

tracking = BooleanVar()
camera_label = Label(window, text="Camera").grid(row=2, column=0, pady=2, sticky=W)
track_camera = Radiobutton(window, text="Tracking", variable=tracking, value=True).grid(row=2, column=2, sticky=W)
fixed_camera = Radiobutton(window, text="Fixed", variable=tracking, value=False).grid(row=2, column=3, sticky=W)

speed_label = Label(window, text="Speed").grid(row=3, column=0)
elevation_label = Label(window, text="Elevation").grid(row=3, column=1)
distance_label = Label(window, text="Distance").grid(row=3, column=2)
trackbody_label = Label(window, text="Trackbody").grid(row=3, column=3)

speed = Entry(window, width=7)
speed.grid(row=4, column=0, pady=2)
elevation = Entry(window, width=7)
elevation.grid(row=4, column=1, pady=2)
distance = Entry(window, width=7)
distance.grid(row=4, column=2, pady=2)
trackbody = Entry(window, width=7)
trackbody.grid(row=4, column=3, pady=2)

live = BooleanVar()
live_camera = Radiobutton(window, text="Live", variable=live, value=True).grid(row=5, column=0, sticky=W)
video_camera = Radiobutton(window, text="Video", variable=live, value=False).grid(row=5, column=1, sticky=W)
video_directory = Entry(window, width=10)
video_directory.grid(row=5, column=2)
step = Entry(window, width=7)
step.grid(row=5, column=3)

init_btn = Button(window, text="Init and Run", command=init_and_run, width=28).grid(row=6, column=0, columnspan=4)
save_btn = Button(window, text="Save", width=14, command=save_setting).grid(row=7, column=0, columnspan=2)
load_btn = Button(window, text="Load", width=14, command=load_setting).grid(row=7, column=2, columnspan=2)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "default_control.txt"

load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()