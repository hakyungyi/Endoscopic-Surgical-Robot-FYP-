from dynamixel_sdk import *  # Uses Dynamixel SDK library
from scipy.signal import find_peaks
import time
import numpy as np
import matplotlib.pyplot as plt

######################### SET UP ###########################
# Control table address
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PRESENT_CURRENT = 126
ADDR_MX_PRESENT_POSITION = 132
ADDR_MX_PROFILE_VELOCITY = 112  # Address for profile velocity

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = [0, 3]
BAUDRATE = 3000000
DEVICENAME = '/dev/tty.usbserial-FT7WBBBJ'  # Change it according to your system

# Initialize PortHandler instance
port_handler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packet_handler = PacketHandler(PROTOCOL_VERSION)

# Open port
if port_handler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if port_handler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable torque for motors
for motor_id in DXL_ID:
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_MX_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print(f"Torque enabled for motor {motor_id}")


#################################################### FUNCTIONS ######################################################
# Function to convert two's complement to signed integer
def twos_complement(value, bit_width):
    if value & (1 << (bit_width - 1)):
        value -= 1 << bit_width
    return value

# Function to read present current
def read_present_current(dxl_id):
    dxl_present_current, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, dxl_id, ADDR_MX_PRESENT_CURRENT)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to read present current for motor {dxl_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    elif dxl_error != 0:
        print(f"Error encountered while reading present current for motor {dxl_id}: {packet_handler.getRxPacketError(dxl_error)}")
        return None
    else:
        current_signed = twos_complement(dxl_present_current, 16)
        #print(f"Current for motor {dxl_id}:" , current_signed * 3.36, "raw:", dxl_present_current)
        return current_signed * 3.36  # Convert to mA
        

# Function to read present position
def read_present_position(dxl_id):
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to read present position for motor {dxl_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return None
    elif dxl_error != 0:
        print(f"Error encountered while reading present position for motor {dxl_id}: {packet_handler.getRxPacketError(dxl_error)}")
        return None
    else:
        return dxl_present_position
    

# Function to set goal position
def set_goal_position(dxl_id, goal_position):
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set goal position for motor {dxl_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Error encountered while setting goal position for motor {dxl_id}: {packet_handler.getRxPacketError(dxl_error)}")

# Function to set profile velocity
def set_profile_velocity(dxl_id, profile_velocity):
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, dxl_id, ADDR_MX_PROFILE_VELOCITY, profile_velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set profile velocity for motor {dxl_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Error encountered while setting profile velocity for motor {dxl_id}: {packet_handler.getRxPacketError(dxl_error)}")
    else:
        print(f"Profile velocity for motor {dxl_id} set to {profile_velocity}")

# Moving average filter function
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Find Local Maxima
def find_two_highest_local_maxima(data):
    peaks, _ = find_peaks(data)
    if len(peaks) < 2:
        raise ValueError("Not enough peaks found to calculate two highest local maxima.")
    sorted_peaks = sorted(peaks, key=lambda x: data[x], reverse=True)
    highest_peak = sorted_peaks[0]
    second_highest_peak = sorted_peaks[1]
    return highest_peak, second_highest_peak


# Lists to store data
time_data = []
current_data_0 = []
current_data_3 = []
position_data_0 = []
position_data_3 = []

# Convert position to degrees
def position_to_degrees(position, initial_position):
    return (position - initial_position) * 0.088  # Convert to degrees assuming 4096 positions for 360 degrees

def find_two_highest_peaks(data):
    sorted_data = sorted(enumerate(data), key=lambda x: x[1], reverse=True)
    highest_peak = sorted_data[0]
    second_highest_peak = sorted_data[1]
    return highest_peak, second_highest_peak

#################################################### RUN  ######################################################
# Set the goal positions and gather data
start_time = time.time()
initial_position_0 = read_present_position(0)
initial_position_3 = read_present_position(3)

# Define the rotation sequence
rotation_sequence = [
    {"direction": 1, "amount": 500},
    {"direction": -1, "amount": 1000},
    {"direction": 1, "amount": 500}
]

# Set profile velocity (adjust the value as needed)
for motor_id in DXL_ID:
    set_profile_velocity(motor_id, 10)  # Example value for profile velocity

def is_movement_complete(motor_ids, goal_positions, tolerance=10):
    for motor_id, goal_position in zip(motor_ids, goal_positions):
        current_position = read_present_position(motor_id)
        if abs(current_position - goal_position) > tolerance:
            return False
    return True

# In the main loop:
for step in rotation_sequence:
    direction = step["direction"]
    amount = step["amount"]
    
    goal_positions = []
    for motor_id in DXL_ID:
        present_position = read_present_position(motor_id)
        goal_position = present_position + (direction * amount)
        set_goal_position(motor_id, goal_position)
        goal_positions.append(goal_position)
    
    # Collect data while motors are moving
    step_start_time = time.time()
    while time.time() - step_start_time < 10:  # 10-second timeout
        if is_movement_complete(DXL_ID, goal_positions):
            break       
        current_time = time.time() - start_time
        time_data.append(current_time)
        current_data_0.append(read_present_current(0))
        current_data_3.append(read_present_current(3))
        position_data_0.append(position_to_degrees(read_present_position(0), initial_position_0))
        position_data_3.append(position_to_degrees(read_present_position(3), initial_position_3))
        time.sleep(0.1)
    
    # Optional: Add a small delay between movements
    time.sleep(0.2)


# Disable torque and close port
for motor_id in DXL_ID:
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_MX_TORQUE_ENABLE, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print(f"Torque disabled for motor {motor_id}")

port_handler.closePort()

# Apply moving average filter
window_size = 8  # Adjust the window size as needed
smoothed_current_0 = moving_average(current_data_0, window_size)
smoothed_current_3 = moving_average(current_data_3, window_size)

#################################################### LOCAL MAXIMA ######################################################
# Define the total duration of the program (this should be calculated based on your actual program's execution time)
total_duration = time_data[-1]  # Assuming time_data has the timestamps of your data collection

# Set the boundaries for peak detection
min_time = 2  # Peaks should be detected after 2 seconds
max_time = total_duration - 2  # Peaks should be detected 2 seconds before the end

# Convert time bounds to indices for peak detection
min_index = np.searchsorted(time_data, min_time)
max_index = np.searchsorted(time_data, max_time)

# Restrict the data for peak detection to the valid time window
restricted_current_0 = smoothed_current_0[min_index:max_index]
restricted_current_3 = smoothed_current_3[min_index:max_index]
restricted_time_data = time_data[min_index:max_index]

# Find the peaks with a minimum distance of 2 seconds between them (distance is calculated in terms of index steps)
sampling_rate = len(time_data) / total_duration  # Estimate sampling rate
min_distance = int(2 * sampling_rate)  # 2 seconds worth of indices

# Detect peaks for Motor 0 and Motor 3
peaks_0, _ = find_peaks(restricted_current_0, distance=min_distance)
peaks_3, _ = find_peaks(restricted_current_3, distance=min_distance)

# Adjust peak indices to match the original data
peaks_0 = [peak + min_index for peak in peaks_0]
peaks_3 = [peak + min_index for peak in peaks_3]

# Ensure at least two peaks are found, otherwise handle the error
if len(peaks_0) < 2 or len(peaks_3) < 2:
    raise ValueError("Not enough peaks found to calculate two highest local maxima.")

# Find the two highest peaks
sorted_peaks_0 = sorted(peaks_0, key=lambda x: smoothed_current_0[x], reverse=True)
sorted_peaks_3 = sorted(peaks_3, key=lambda x: smoothed_current_3[x], reverse=True)

peak_0_1, peak_0_2 = sorted_peaks_0[:2]
peak_3_1, peak_3_2 = sorted_peaks_3[:2]

# Get the time indices of the peaks
index_0_1 = peak_0_1
index_0_2 = peak_0_2
index_3_1 = peak_3_1
index_3_2 = peak_3_2

# Calculate the corresponding position angles
angle_0_1 = position_data_0[index_0_1]
angle_0_2 = position_data_0[index_0_2]
angle_3_1 = position_data_3[index_3_1]
angle_3_2 = position_data_3[index_3_2]

# Calculate the differences between the position angles
angle_diff_0 = abs(angle_0_1 - angle_0_2)
angle_diff_3 = abs(angle_3_1 - angle_3_2)

home_0 = (angle_0_1 + angle_0_2)/2
home_3 = (angle_3_1 + angle_3_2)/2

print(f"Motor 0: Highest peak angles are {angle_0_1:.2f} and {angle_0_2:.2f} with a difference of {angle_diff_0:.2f} degrees. Home Position = {home_0:.2f} degree")
print(f"Motor 3: Highest peak angles are {angle_3_1:.2f} and {angle_3_2:.2f} with a difference of {angle_diff_3:.2f} degrees. Home Position = {home_3:.2f} degree")

#################################################### GRAPH PLOT ######################################################
# Plotting the current and position data
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
ax1.plot(time_data[:len(smoothed_current_0)], smoothed_current_0, label='Motor 0 Current (mA)')
ax2.plot(time_data[:len(smoothed_current_3)], smoothed_current_3, label='Motor 3 Current (mA)')

# Mark the peaks for Motor 0
ax1.plot(time_data[peak_0_1], smoothed_current_0[peak_0_1], 'ro')
ax1.plot(time_data[peak_0_2], smoothed_current_0[peak_0_2], 'ro')

# Mark the peaks for Motor 3
ax2.plot(time_data[peak_3_1], smoothed_current_3[peak_3_1], 'ro')
ax2.plot(time_data[peak_3_2], smoothed_current_3[peak_3_2], 'ro')


ax1.set_ylabel('Current (mA)')
ax2.set_ylabel('Current (mA)')
ax3.set_ylabel('Position (degrees)')
ax3.set_xlabel('Time (s)')

# Plot positions vs time
ax3.plot(time_data, position_data_0, label='Motor 0 Position (degrees)')
ax3.plot(time_data, position_data_3, label='Motor 3 Position (degrees)')

ax1.legend()
ax2.legend()
ax3.legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show the plot
plt.show()
