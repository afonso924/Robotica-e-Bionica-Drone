import logging
import time
import math
import struct
import asyncio
import threading
from bleak import BleakClient, BleakScanner
import struct
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
# from cflib.utils.reset_estimator import reset_estimator
from threading import Lock

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')
IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"  
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"

# Outputs errors from the logging framework
logging.basicConfig(level=logging.ERROR)

latest_data = None
data_lock = Lock()


#----------------------------------------------------------------------------------------
#Just for scanning devices(useful to make sure the snapki is being detected)


async def scan_devices():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Found: {device.name} | Address: {device.address}")

asyncio.run(scan_devices())


#----------------------------------------------------------------------------------------
#IMU sensor functions

def start_imu_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_to_imu())


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')


async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("Connected to IMU Sensor")

        # Enable notifications
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        
        # Keep listening
        await asyncio.sleep(1200)  # How long the code extracts the data
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)



async def imu_notification_handler(sender, data):
    #print(f"Raw data: {data}")
    process_imu_data(data)



def decide_movement(force_1, force_2, roll_deg, pitch_deg):
    #This function decides movement based on the force sensor and orientation data
    #The values were decided based on tests made with the snapki

    # 1. Going Up
    if  1000 < force_1 < 1300:
        return "high"
    
    # 1. Going Up
    if force_1 > 1300:
        return "high2"
    

    # 2. Going Down
    elif force_2 > 1500:
        return "low"

    # 3. Rotation Left
    elif abs(roll_deg) < 30:  
        return "rotateL"

    # 4. Rotation Right
    elif abs(roll_deg - 180) < 30:  
        return "rotateR"

    # 5. Move Backward
    elif pitch_deg > 50:
        return "backward"

     # 6. Move Forward
    elif  pitch_deg < -30:
        return "forward"
    
    # If none match
    return "stop"




def process_imu_data(data):
    global latest_data

    # Guarantees it is receiving the right amount of data
    if len(data) < 220:
        print(f"Warning: Received only {len(data)} bytes, expected at least 220!")
        return

    # number of floats to extract
    num_floats = 55  # 55 floats × 4 bytes each = 220 bytes

    # Unpack all 55 floats at once
    values = struct.unpack("<" + "f" * num_floats, data[:220])

    # Assigning values based on the table positions
    accel_x1, accel_y1, accel_z1 = values[0], values[1], values[2]
    gyro_x1, gyro_y1, gyro_z1 = values[3], values[4], values[5]
    mag_x1, mag_y1, mag_z1 = values[6], values[7], values[8]
    force_1, force_2 = values[9], values[10]  # First force sensor readings
    
    
    #=========================================

    #calc roll, pitch and 
    roll = math.atan2(accel_y1, accel_z1)
    pitch = math.atan(-accel_x1 / math.sqrt(accel_y1**2 + accel_z1**2))

    # Calculate yaw using magnetometer data
    # Compensate magnetometer readings with current roll and pitch
    mag_x_comp = mag_x1 * math.cos(pitch) + mag_z1 * math.sin(pitch)
    mag_y_comp = (mag_x1 * math.sin(roll) * math.sin(pitch) +
                  mag_y1 * math.cos(roll) -
                  mag_z1 * math.sin(roll) * math.cos(pitch))
    yaw = math.atan2(-mag_y_comp, mag_x_comp)

    # Convert radians to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    # Print results(useful to see the data in real time and if it is working as intended)
    print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
    print(f"Accel: x={accel_x1}, y={accel_y1}, z={accel_z1}")
    print(f"Gyro: x={gyro_x1}, y={gyro_y1}, z={gyro_z1}")
    print(f"Magnetometer: x={mag_x1}, y={mag_y1}, z={mag_z1}")
    print(f"Force Sensors: F1={force_1}, F2={force_2}")
    
    with data_lock:
        latest_data = decide_movement(force_1, force_2, roll_deg, pitch_deg)
    
    
   
    



#----------------------------------------------------------------------------------------
#Crazyflie functions

def decide_commands(latest_data):
    # Default hoooover (no movement)
    vx = 0.0       # forward/backward velocity
    vy = 0.0       # left/right velocity
    yaw_rate = 0.0 # rotation rate (yaw)
    z = 0.0        # vertical velocity (up/down)

    if latest_data == "rotateR":
        yaw_rate = 30.0  # rotate right (not sure about the units)
    elif latest_data == "rotateL":
        yaw_rate = -30.0 # rotate left
    elif latest_data == "forward":
        vx = 0.2         # move forward
    elif latest_data == "backward":
        vx = -0.2        # move backward
    elif latest_data == "high":
        z = 0.2          # move up
    elif latest_data == "low":
        z = -0.2         # move down

    return vx, vy, yaw_rate, z





def fly_drone():
    global latest_data
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        reset_estimator(cf)
        print("reset estimator..........")

        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        print("ariming......")

        # Initiate flight
        print("[Crazyflie] Takeoff...")
        for i in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, i / 30) # takeoff
            time.sleep(0.1)

        for j in range(5):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.3)

        print('[Crazyflie] Hovering...')

        try:
            while True:
                

                with data_lock:
                     data_copy = latest_data
                vx, vy, yaw_rate, z = decide_commands(data_copy)
                
                print(data_copy)

                #drone using that configurations
                cf.commander.send_hover_setpoint(vx, vy, yaw_rate, 0.3)
                
                #wait a bit 
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Landing and stopping...")

        cf.commander.send_stop_setpoint()
        cf.commander.send_notify_setpoint_stop()


         

#-----------------------------------------------------------------------------------------
#Threads


imu_thread = threading.Thread(target=start_imu_thread,daemon = True)
drone_thread = threading.Thread(target=fly_drone,daemon = True)


imu_thread.start()
drone_thread.start()

# Keeping the main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped by user")
