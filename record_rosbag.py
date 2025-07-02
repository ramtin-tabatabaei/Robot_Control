import subprocess
import datetime
import os
import time
import signal

# Define a signal handler for SIGINT
def signal_handler(sig, frame):
    print('Stopping recording...')
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    process.wait()
    print('Recording stopped.')
    exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

# Ensure the directory exists
directory = "/home/pal/rosbags/"
if not os.path.exists(directory):
    os.makedirs(directory)

# Format the current date and time as a string
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

# Define the bag file name using the current date and time
bag_file_name = f"{directory}record_{current_time}.bag"

# Define the topics to record
topics = "/filtered_image_raw /ee_state_topic"

# Define the command to start recording all topics to the named bag file
command = f"rosbag record --lz4 {topics} -O {bag_file_name}"

# Start recording
process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

# Keep the script running until Ctrl+C is pressed
print("Recording... Press Ctrl+C to stop.")
while True:
    time.sleep(1)
