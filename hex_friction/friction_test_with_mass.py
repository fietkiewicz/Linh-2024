import sys
import mujoco
import mujoco.viewer
import mujocoviewer2
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd
import mediapy as media
import xml.etree.ElementTree as ET

MJCF = """
<mujoco>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="none"/>
    <material name="grid" texture="grid" texrepeat="6 6"
     texuniform="true" reflectance=".2"/>
     <material name="wall" rgba='.5 .5 .5 1'/>
  </asset>

  <default>
    <geom type="box" size=".05 .05 .05" />
    <joint type="free"/>
  </default>

  <worldbody>
    <light name="light" pos="-.2 0 1"/>
    <geom name="ground" type="plane" size=".5 .5 10" material="grid"
     zaxis="-.3 0 1" friction=".1"/>
    <camera name="y" pos="-.1 -.6 .3" xyaxes="1 0 0 0 1 2"/>
    <body pos="0 0 .1">
      <joint/>
      <geom/>
    </body>
    <body pos="0 .2 .1">
      <joint/>
      <geom name="cube" friction=".33"/>
    </body>
  </worldbody>

</mujoco>
"""
def init_and_run():

    doc = ET.fromstring(MJCF)
    tree = ET.ElementTree(doc)

    for geom in tree.iter('geom'):
        if geom.get('name') == "cube":
            if friction.get():
                geom.set("friction", friction.get())
            if mass.get():
                geom.set("mass", mass.get())
        if geom.get('name') == "ground":
            if friction2.get():
                geom.set("friction", friction2.get())
    
    tree.write('output.xml', encoding="utf-8")

    model = mujoco.MjModel.from_xml_path('output.xml')
    data = mujoco.MjData(model)

    if live.get() == True: N = 100000
    else: N = int(step.get())
    i = 0
    frames = []

    if live.get() == True:
        viewer = mujocoviewer2.MujocoViewer(model, data)
    else:
        viewer = mujocoviewer2.MujocoViewer(model, data, 'offscreen')

    if tracking.get() == True:
        viewer.cam.fixedcamid = 0
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING # following 
        viewer.cam.trackbodyid = int(trackbody.get()) # this is torso id
    viewer.cam.distance = float(distance.get())
    viewer.cam.elevation = int(elevation.get())

    while viewer.is_alive and i < N:

        if i % int(speed.get()) == 0:
            if live.get() == True:
                viewer.render()
            else:
                img = viewer.read_pixels()
                frames.append(img)
        i += 1
        mujoco.mj_step(model, data)
    viewer.close()

    if not live.get():
        media.write_video(video_directory.get(), frames, fps=30)
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