import mujoco
import mediapy as media

duration = 2.5  # (seconds)
framerate = 60  # (Hz)

filename = 'hex1.xml'
model = mujoco.MjModel.from_xml_path(filename)
data = mujoco.MjData(model)

# Make renderer, render and show the pixels
renderer = mujoco.Renderer(model)

# Simulate and display video.
frames = []
mujoco.mj_resetData(model, data)  # Reset state and time.
while data.time < duration:
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
        renderer.update_scene(data, camera='top')
        pixels = renderer.render()
        frames.append(pixels)
media.show_video(frames, fps=framerate)