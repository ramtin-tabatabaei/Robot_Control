import tkinter as tk
import subprocess
import threading
from tkinter import ttk
import numpy as np
def button_pressed(button, puzzle_number, object_number, participant_id, selected_value_failure, selected_value_verbal):
    global counter_object
    color = button.cget("bg")

    if color == "#d9d9d9":
        button.config(bg='green')
        counter_object +=1
        print(counter_object)

    subprocess.run(['python3', 'RobotActions_v2.py', str(object_number), str(puzzle_number), str(participant_id), str(selected_value_failure), str(selected_value_verbal), str(counter_object), str(toggle_state), str(toggle_state_camera), str(toggle_state_camera_orientation)])

def button_pressed_talking(script, i):
    subprocess.run(['python3', script, str(i)])


def button_pressed_home(value):
    global Puzzle_Id
    Puzzle_Id = value
    home_window.destroy()
    create_gui(value)

def run_file(file_path):
    subprocess.run(['python3', file_path])

def run_file2(file_path, b):
    result = subprocess.run(['python3', file_path, str(Puzzle_Id)], capture_output=True, text=True)
    output = result.stdout.strip()
    print(output)
    if output == "1":
        b.config(bg='green')
    elif output == "0":
        b.config(bg="red")


def run_in_thread(func, *args):
    thread = threading.Thread(target=func, args=args)
    thread.start()
def go_home():
    window.destroy()
    create_home_window()

def create_home_window():
    global home_window
    home_window = tk.Tk()
    home_window.title("Home - Select Puzzle")
    home_window.geometry("500x500")
    tk.Label(home_window, text="Select a Puzzle", font=("Arial", 14, "bold")).pack(pady=10)
    puzzles = [
        ("Puzzle 1 (Rocket)", 1),
        ("Puzzle 2 (Turtle)", 2),
        ("Puzzle 3 (Cat)", 3),
        ("Puzzle 4 (Rabbit)", 4),
        ("Puzzle 5 (Dog)", 5),
        ("Puzzle 6 (Swan)", 6),
    ]
    for text, value in puzzles:
        tk.Button(home_window, text=text, font=("Arial", 12), width=20, height=2,
                  command=lambda v=value: button_pressed_home(v)).pack(pady=5)
    home_window.mainloop()
def Predefined_values_failure(participant_id, puzzle_number):
    Failures_Values = np.array([
        [1, 1, 1],
        [2, 1, 3],
        [3, 1, 2],
        [2, 2, 2],
        [1, 2, 3],
        [3, 2, 1],
        [3, 3, 3],
        [1, 3, 2],
        [2, 3, 1]
    ])
    Verbal_Values = np.array([
        [1, 2, 0],
        [1, 0, 2],
        [2, 1, 0],
        [2, 0, 1],
        [0, 1, 2],
        [0, 2, 1]
    ])
    output_f = np.zeros(4, dtype=int)  # Ensure it has valid values
    output_v = np.zeros(4, dtype=int)  # Ensure it has valid values
    if puzzle_number % 2 == 0:
        row_f = ((int(participant_id)-1) % 9)
        row_v = int((int(participant_id)-1) / 9)
        print(row_f)
        print(row_v)
        # row_f = (int(participant_id) % 9)-1
        # row_v = int(int(participant_id) / 9)
        # print(row_f)
        # print(row_v)

        # if row_f or row_v < 0:
        #     print("error")  # Avoid negative index issues
        Failure = Failures_Values[row_f, int(puzzle_number / 2) - 1]
        Verbal = Verbal_Values[row_v, int(puzzle_number / 2) - 1]
        if 0 <= Failure < len(output_f):  # Ensure index is within bounds
            output_f[Failure] = Failure
            output_v[Failure] = Verbal
        else:
            print(f"Warning: Failure index {Failure} is out of bounds for puzzle {puzzle_number}")
    return output_f, output_v

def create_gui(puzzle_number):
    global window, counter_object, toggle_state, toggle_state_camera, toggle_state_camera_orientation
    counter_object = 0
    toggle_state = 1  # Initial state
    toggle_state_camera = 0  # Initial state
    toggle_state_camera_orientation = "Up"  # Initial state

    def toggle_button_action(button):
        global toggle_state
        toggle_state = 1 if toggle_state == 2 else 2  # Toggle between 1 and 2
        print(toggle_state)  # Print the new state
        button.config(text=f"Mode: {toggle_state}")  # Update button text

    def toggle_button_action_camera(button):
        global toggle_state_camera
        toggle_state_camera = (toggle_state_camera + 1) % 5  # Cycle through 0,1,2,3,4
        print(toggle_state_camera)  # Print the new state
        button.config(text=f"Camera Mode: {toggle_state_camera}")  # Update button text

    def toggle_button_action_camera_orientation(button):
        global toggle_state_camera_orientation
        if toggle_state_camera_orientation  == "Up":
            toggle_state_camera_orientation  = "Down"
        else:
            toggle_state_camera_orientation  = "Up"

        print(toggle_state_camera_orientation)  # Print the new state
        button.config(text=f"{toggle_state_camera_orientation}")  # Update button text



    window = tk.Tk()
    window.title(f"Puzzle {puzzle_number} - Actions")
    window.geometry("700x900")
    tk.Label(window, text=f"Puzzle {puzzle_number} - Actions", font=("Arial", 14, "bold")).pack(pady=10)
    action_frame = tk.Frame(window)
    action_frame.pack()
    def on_button_rotatehead():
        selected_value = dropdown_var.get()
        subprocess.run(['python3', action_head[0][1], selected_value])
    dropdown_var = tk.StringVar(value="Yaw:-55, roll:-55")
    dropdown_headrotation = ttk.Combobox(action_frame, textvariable=dropdown_var, values=["Yaw:-30, roll:-55", "Yaw:-55, roll:-55", "Yaw:-45, roll:-45"])
    dropdown_headrotation.grid(row=0, column=1, padx=10, pady=5)
    action_head = [
        ("Rotate Head", 'head-rotate.py'),
    ]
    btn_headrotation = tk.Button(action_frame, text=action_head[0][0], font=("Arial", 12), width=15, height=2)
    btn_headrotation.config(command=on_button_rotatehead)
    btn_headrotation.grid(row=0, column=0, padx=10, pady=10)
    actions = [
        ("Get Camera", 'get_camera.py'),
        ("Ready Position", 'ready_position.py'),
        ("TF Function", 'TF2.py'), 
        ("Check Orientation", 'Safety.py'), 
            ]
    
    for i, (text, script) in enumerate(actions):
        button_action = tk.Button(action_frame, text=text, font=("Arial", 12), width=20, height=2)
        button_action.grid(row=i//2+1, column=i%2, padx=10, pady=10)
        button_action.config(command=lambda s=script, b=button_action: run_in_thread(run_file2, s, b))


    button_action = tk.Button(action_frame, text="Mode: 1", font=("Arial", 12), width=20, height=2)
    button_action.grid(row=3, column=0, padx=10, pady=10)
    button_action.config(command=lambda b=button_action: toggle_button_action(b))

    # button_action = tk.Button(action_frame, text="Camera Mode: 0", font=("Arial", 12), width=20, height=2)
    # button_action.grid(row=3, column=1, padx=10, pady=10)
    # button_action.config(command=lambda b=button_action: toggle_button_action_camera(b))

    # Configure columns to ensure equal space
    action_frame.columnconfigure(1, weight=1)
    action_frame.columnconfigure(2, weight=1)

    # Define button width to be the same
    button_width = 8  # Adjust as needed

    # Create the first button (small, placed in column 1)
    button_action = tk.Button(action_frame, text="Camera Mode: 0", font=("Arial", 12), width=button_width, height=2)
    button_action.grid(row=3, column=1, padx=5, pady=10, sticky="ew")
    button_action.config(command=lambda b=button_action: toggle_button_action_camera(b))

    # Create the second button (same size, placed next to the first in column 2)
    button_action = tk.Button(action_frame, text="Up", font=("Arial", 12), width=button_width, height=2)
    button_action.grid(row=3, column=2, padx=5, pady=10, sticky="ew")
    button_action.config(command=lambda b=button_action: toggle_button_action_camera_orientation(b))

        

    tk.Label(window, text=f"Puzzle {puzzle_number} - Talking", font=("Arial", 14, "bold")).pack(pady=10)
    talking_frame = tk.Frame(window)
    talking_frame.pack()
  
    Talkings = [
        ("Start Solving", 'talking.py'),
        ("Your turn", 'talking.py'),
        ("Check the task", 'talking.py'),
        ("No", 'talking.py')
        ]
    for i, (text, script) in enumerate(Talkings):
        tk.Button(talking_frame, text=text, font=("Arial", 12), width=20, height=2,
                  command=lambda s=script, i=i: button_pressed_talking(s, i)).grid(row=i//2+1, column=i%2, padx=10, pady=10)



    tk.Label(window, text=f"Puzzle {puzzle_number} - Objects", font=("Arial", 14, "bold")).pack(pady=10)
    object_frame = tk.Frame(window)
    object_frame.pack()
    objects = ["Object 1", "Object 2", "Object 3", "Object 4"]
    predefined_values_f, predefined_values_v = Predefined_values_failure(participant_id, puzzle_number)
    predefined_values_failure = {"Object 1": str(predefined_values_f[0]), "Object 2": str(predefined_values_f[1]), "Object 3": str(predefined_values_f[2]), "Object 4": str(predefined_values_f[3])}
    selected_values_failures = {obj: tk.StringVar(window, value=predefined_values_failure[obj]) for obj in objects}
    predefined_values_verbals = {"Object 1": str(predefined_values_v[0]), "Object 2": str(predefined_values_v[1]), "Object 3": str(predefined_values_v[2]), "Object 4": str(predefined_values_v[3])}
    selected_values_verbals = {obj: tk.StringVar(window, value=predefined_values_verbals[obj]) for obj in objects}
    def on_button_press(obj_name, button, obj_index):
        selected_value_failure = selected_values_failures[obj_name].get()
        selected_value_verbal = selected_values_verbals[obj_name].get()
        print(selected_value_failure)
        print(selected_value_verbal)
        button_pressed(button, puzzle_number, obj_index, participant_id, selected_value_failure, selected_value_verbal)
    for i, obj in enumerate(objects, start=1):
        btn = tk.Button(object_frame, text=obj, font=("Arial", 12), width=15, height=2)
        btn.config(command=lambda o=obj, b=btn, i=i: on_button_press(o, b, i))
        btn.grid(row=i, column=0, padx=10, pady=5)
        dropdown_failures = tk.OptionMenu(object_frame, selected_values_failures[obj], "0", "1", "2", "3")
        dropdown_failures.grid(row=i, column=1, padx=10, pady=5)
        dropdown_verbal = tk.OptionMenu(object_frame, selected_values_verbals[obj], "0", "1", "2")
        dropdown_verbal.grid(row=i, column=2, padx=10, pady=5)
    tk.Button(window, text="Home", font=("Arial", 12, "bold"), bg="red", fg="white", width=15, height=2,
              command=go_home).pack(pady=20)
    window.mainloop()


def submit_and_close(entry_widget, window):
    global participant_id
    participant_id = entry_widget.get()
    window.destroy()
    create_home_window()


def start_app():
    start_window = tk.Tk()
    start_window.title("Welcome")
    start_window.geometry("400x200")
    tk.Label(start_window, text="Enter Participant ID:", font=("Arial", 12)).pack(pady=10)
    participant_id_entry = tk.Entry(start_window, font=("Arial", 12))
    participant_id_entry.pack(pady=5)
    tk.Button(start_window, text="Submit", font=("Arial", 12, "bold"), bg="green", fg="white",
              command=lambda: submit_and_close(participant_id_entry, start_window)).pack(pady=10)
    start_window.mainloop()


if __name__ == "__main__":
    start_app()