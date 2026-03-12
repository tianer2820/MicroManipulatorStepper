from open_micro_stage_api import OpenMicroStageInterface
import time
import csv

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM5')
oms.read_device_state_info()


while True:
    input("run loop")
    oms.home()
    for x in range(0,30):
        for y in range(0,30):
            for z in range(0,30):
                x_dest = x/10
                y_dest = y/10
                z_dest = z/10
                print(f"Moving to {x_dest}, {y_dest}, {z_dest}")
                oms.set_pose(x_dest,y_dest,z_dest)
                oms.wait_for_stop()
                time.sleep(0.1)
