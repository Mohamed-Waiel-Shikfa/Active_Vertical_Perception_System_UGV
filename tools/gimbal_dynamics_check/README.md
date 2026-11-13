# Gimbal dynamics check (developer tool, not part of the image)

Runs the generated lift + two C-200T gimbals on a rocking, turning, lifting base in a
headless Gazebo Harmonic server (DART), without the Jackal model or camera sensors.

    python3 make_world.py out            # writes out/robot.urdf and out/world.sdf
    # build libGreenhouseSystems.so from ../../native/Systems.cc and driver.cc against gz-sim8
    GZ_SIM_SYSTEM_PLUGIN_PATH=<dir with libGreenhouseSystems.so>:<gz-sim systems> \
      ./driver out/world.sdf '{"camera":2,"pitch":-30,"yaw":20,"roll":10}'

`result.txt` is the output of the run reported in docs/TECHNICAL.md (gz-sim 8.15,
gz-physics 7.8, DART 6.13, 2 ms step).
