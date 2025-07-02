import rospy
import sys
from std_msgs.msg import String  # Import required message type

def send_audio(text):
    """
    Function to send the audio file path as a ROS message.
    """
    # Path to the WAV file on the robot
    audio_file = f"/home/pal/Audios/{text}.wav"

    rospy.loginfo(f"📤 Sending audio file path: {audio_file}")
    audio_pub.publish(audio_file)  # Publish the audio file path


# Initialize ROS node
rospy.init_node('audio_sender', anonymous=True)

# Define the ROS publisher
audio_pub = rospy.Publisher("/play_audio", String, queue_size=10)


# Wait for the publisher to establish connection
rospy.sleep(1)

# Ensure command-line argument is provided
if len(sys.argv) < 2:
    rospy.logerr("❌ No number provided. Usage: python script.py <number>")
    sys.exit(1)

try:
    # Convert argument to an integer
    received_number = int(sys.argv[1])
except ValueError:
    rospy.logerr("❌ Invalid input! Please enter a valid number (1, 2, or 3).")
    sys.exit(1)

# Determine the message based on the number
if received_number == 0:
    message = "StartSolving"
elif received_number == 1:
    message = "Participantsturn"
elif received_number == 2:
    message = "Papernospace"
elif received_number == 3:
    message = "no"
else:
    rospy.logerr("❌ Invalid number! Please use 1, 2, or 3.")
    sys.exit(1)

# Send the audio file path
send_audio(message)
