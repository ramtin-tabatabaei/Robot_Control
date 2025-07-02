import openai
# openai.api_key = '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'
response = openai.audio.speech.create(
    model="tts-1",  # or "tts-1-hd" for higher quality
    voice="alloy",   # voices: "nova", "echo", "onyx", "fable", "shimmer", "alloy"
    input="I am not that slow."
)
# Save audio to file
with open("output.mp3", "wb") as f:
    f.write(response.content)

from pydub import AudioSegment

# Load MP3
sound = AudioSegment.from_mp3("output.mp3")

# Export as WAV
sound.export("output.wav", format="wav")


import paramiko

# Replace with your TIAGo robot's details
robot_ip = "tiago-196c"  # Replace with actual IP
username = "pal"            # Default user is often 'pal'
password = "pal"            # Replace with actual password
local_audio = "output.wav"  # Prefer WAV format for easy playback with aplay
remote_audio = "/tmp/output.wav"


# Connect via SSH
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(robot_ip, username=username, password=password)

# Transfer the file via SFTP
sftp = ssh.open_sftp()
sftp.put(local_audio, remote_audio)
sftp.close()
print(f"Audio file uploaded to {remote_audio}")

# Play the audio on TIAGo using aplay
stdin, stdout, stderr = ssh.exec_command(f"aplay {remote_audio}")
print(stdout.read().decode())
print(stderr.read().decode())

ssh.close()
