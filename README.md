# Robot_Control

🤖 Robot Control – README
This project includes the code and procedures used to run a robot-assisted puzzle task experiment. Below is a description of the available scripts and how they are used in the experimental setup.

📁 GUI Scripts
1. GUI_V2.py
This is the main GUI used during the experiment.

In this version:

Collision detection is enabled.

The robot moves slowly for safety.

2. GUI_Cal.py
This script is used before starting the experiment.

It helps calibrate the robot and the table setup, including the positioning of puzzle pieces.

🖥 GUI Workflow
When you run the GUI script, a window appears with the following functionality:

➤ Participant ID Input
You must first enter the participant ID.

The failure condition and recovery behavior are automatically determined based on the ID.

➤ Puzzle Shape Buttons
Six puzzle shapes are shown as buttons.

Clicking a shape starts the process for that specific piece.

➤ Head Control
Rotate Head: Adjust the robot’s head rotation. Default: row = -55, roll = -55. You can change these manually.

➤ Camera View
Get Camera: Shows the live view from the robot's head camera.

➤ TF Function
TF Function: Calculates the position of puzzle pieces.

If successful: Button turns green.

If failed: Button turns red.

➤ Orientation Check
Check Orientation: Verifies if the pieces are placed in the correct orientation.

Green = correct, Red = incorrect.

⚙️ Control Options
➤ Mode Button
Mode: 1

Controls placement logic for small triangle pieces.

If set to 1: Places triangles as expected.

If set to 2: Switches triangle 1 and triangle 2 (useful if participant misplaces them).

➤ Camera Mode Button
Camera Mode: 0: Uses the position data from the TF function.

Changing to 1, 2, etc. overrides with predefined positions:

1 = upper right

2 = upper left

etc.

➤ Flip Button (Triangles)
Toggles triangle orientation (e.g., upside-down vs. right-side-up).

🔊 Speech / Audio Interaction
➤ Talking Buttons
Trigger specific audio files that correspond to robot speech actions.

➤ Failure & Reaction Encoding
Each object has two associated numbers:

First Number (Failure Type):

0 = No failure

1 = Freezing failure

2 = Planning failure

3 = Grasping failure

Second Number (Robot's Reaction):

0 = No reaction

1 = Partial awareness

2 = Full awareness

🤖 Robot Arm Positioning Scripts
➤ GetUP.py
Moves the robot arm from home to ready position.

➤ Getdown.py
Returns the robot arm from ready to home position.

🗣 Optional: OpenAI TTS (Text-to-Speech)
➤ OpenAi-tts.py
Not used in the main experiment, but available for generating custom audio.

You can enter any text, and it will convert it into an audio file for the robot to play.

⚙️ Object Manipulation Scripts
There are also two scripts used for controlling the robot’s object movement, not detailed here.
