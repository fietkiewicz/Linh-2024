import sys
import mujoco
import mujoco.viewer
import mujocoviewer2
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import mediapy as media
import glfw
import numpy as np

def init_and_run():
    filename = 'rat_hindlimbs_on_ground.xml'
    model = mujoco.MjModel.from_xml_path(filename)
    data = mujoco.MjData(model)

    if live.get() == True: N = 100000
    else: N = int(step.get())
    i = 0
    phase = 0
    frames = []

    viewer = mujocoviewer2.MujocoViewer(model, data)

    if tracking.get() == True:
        viewer.cam.fixedcamid = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING # following 
        viewer.cam.trackbodyid = int(trackbody.get()) # this is torso id
    viewer.cam.distance = float(distance.get())
    viewer.cam.elevation = int(elevation.get())

    hip_flexor = [0, 1, 0, 0]
    ankle_flexor = [1, 1, 0, 0]
    knee_flexor = [1, 1, 0, 0]

    while viewer.is_alive and i < N:

        phase = (i // 100) % 4

        data.ctrl[1] = hip_flexor[phase]
        data.ctrl[3] = knee_flexor[phase]
        data.ctrl[5] = ankle_flexor[phase]
        data.ctrl[7] = hip_flexor[(phase + 2) % 4]
        data.ctrl[9] = knee_flexor[(phase + 2) % 4]
        data.ctrl[11] = ankle_flexor[(phase + 2) % 4]

        if i % int(speed.get()) == 0:
            viewer.render()
            if live.get() == False:
                img = np.zeros((viewer.viewport.height, viewer.viewport.width, 3), dtype=np.uint8)
                mujoco.mjr_readPixels(img, None, viewer.viewport, viewer.ctx)
                img = np.flipud(img)
                frames.append(img)
        i += 1
        mujoco.mj_step(model, data)
    viewer.close()

    if not live.get():
        media.write_video(video_directory.get(), frames, fps=10)
        print("Video finished!")

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

window = tk.Tk()
window.title("Hexapod Control")
window.geometry("240x150")

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

live = BooleanVar()
live_camera = Radiobutton(window, text="Live", variable=live, value=True).grid(row=3, column=0, sticky=W)
video_camera = Radiobutton(window, text="Video", variable=live, value=False).grid(row=3, column=1, sticky=W)
video_directory = Entry(window, width=10)
video_directory.grid(row=3, column=2)
step = Entry(window, width=7)
step.grid(row=3, column=3)

init_btn = Button(window, text="Init and Run", command=init_and_run, width=28).grid(row=4, column=0, columnspan=4)
save_btn = Button(window, text="Save", width=14, command=save_setting).grid(row=5, column=0, columnspan=2)
load_btn = Button(window, text="Load", width=14, command=load_setting).grid(row=5, column=2, columnspan=2)

if (len(sys.argv) > 1):
    filename = sys.argv[1]
else:
    filename = "default_control.txt"

load_setting(filename)
print("Loading settings from ", filename)

window.mainloop()