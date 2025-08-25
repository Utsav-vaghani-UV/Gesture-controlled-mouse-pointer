import tkinter as tk
import subprocess

def run_controller():
    subprocess.run(["python", "C:\\Users\\wagha\\OneDrive\\Desktop\\Gesture Mouse Controller\\src\\Gesture_Controller.py"])

# Create the main window
root = tk.Tk()
root.title("Gesture Mouse Controller")

# Create a label for the title
title_label = tk.Label(root, text="Gesture Mouse Controller", font=("Helvetica", 16))
title_label.pack(pady=20)

# Create a button to start the Gesture_controller.py file
start_button = tk.Button(root, text="Start", command=run_controller)
start_button.pack(pady=10)

# Run the Tkinter event loop
root.mainloop()
