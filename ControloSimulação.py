import asyncio
import math
from bleak import BleakClient
from bleak import BleakScanner
import socket
import time


HOST = "127.0.0.1"  
PORT = 5005

# Create a connection to the Webots controller
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))


async def scan_devices():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Found: {device.name} | Address: {device.address}")

asyncio.run(scan_devices())


IMU_SENSOR_ADDRESS = "84:71:27:AC:20:D2"  #Sensor address
IMU_CHARACTERISTIC_UUID = "14181dce-eb95-46c5-8431-3b4fe0e0a12d"  

async def imu_notification_handler(sender, data):
    #print(f"Raw data: {data}")
    process_imu_data(data)

def decide_movement(force_1, force_2, roll_deg, pitch_deg):
    #Decides the movement based on force sensor and orientation data.

    # 1. Going Up
    if  1000 < force_1 < 1300:
        return "high"
    
    # 1.2. Going Up at a higher speed
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

async def connect_to_imu():
    async with BleakClient(IMU_SENSOR_ADDRESS) as client:
        print("Connected to IMU Sensor")

        # Enable notifications
        await client.start_notify(IMU_CHARACTERISTIC_UUID, imu_notification_handler)
        
        # Keep listening
        await asyncio.sleep(300)  # time interval receiving the data
        await client.stop_notify(IMU_CHARACTERISTIC_UUID)


import struct

def process_imu_data(data):

    if len(data) < 220:
        print(f"Warning: Received only {len(data)} bytes, expected at least 220!")
        return

    # number of floats we need to extract
    num_floats = 55  

    # Unpack all 55 floats at once
    values = struct.unpack("<" + "f" * num_floats, data[:220])

    # Assigning values based on the table positions
    accel_x1, accel_y1, accel_z1 = values[0], values[1], values[2]
    gyro_x1, gyro_y1, gyro_z1 = values[3], values[4], values[5]
    mag_x1, mag_y1, mag_z1 = values[6], values[7], values[8]
    force_1, force_2 = values[9], values[10]  
    
    
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

    print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
    #======================================

    # Print results
    print(f"Accel: x={accel_x1}, y={accel_y1}, z={accel_z1}")
    print(f"Gyro: x={gyro_x1}, y={gyro_y1}, z={gyro_z1}")
    print(f"Magnetometer: x={mag_x1}, y={mag_y1}, z={mag_z1}")
    print(f"Force Sensors: F1={force_1}, F2={force_2}")
    
    
    commands = decide_movement(force_1, force_2, roll_deg, pitch_deg)
   
    
    try:
        print(f"Sending: {commands}")
        client.sendall(commands.encode())
        time.sleep(0.1)  # Wait time before sending next command
    except KeyboardInterrupt:
        print("Stopping fake IMU.")



asyncio.run(connect_to_imu())


