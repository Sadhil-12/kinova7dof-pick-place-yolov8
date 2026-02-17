import subprocess
import tkinter as tk
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

# VENV_PYTHON_EXE = sys.executable
VENV_PYTHON_EXE = r"C:\Users\sadhil\Documents\VS CODE\Machine Learning\ml_env\Scripts\python.exe"
PYTHON_SCRIPT = PROJECT_ROOT / "bottle_tracking_and_saving" / "homography_main.py"
CPP_EXE = PROJECT_ROOT / "build" / "Debug" / "kinova_app.exe"

def run_python_opencv():
    subprocess.Popen(
        [VENV_PYTHON_EXE, PYTHON_SCRIPT],
        cwd=PROJECT_ROOT,
        creationflags=subprocess.CREATE_NEW_CONSOLE
    )
    
def run_cpp():
    subprocess.Popen(
        [CPP_EXE],
        cwd=PROJECT_ROOT,
        creationflags=subprocess.CREATE_NEW_CONSOLE
    )


# ---- GUI ----
root = tk.Tk()
root.title("Program Launcher")
root.geometry("300x150")

tk.Button(
    root,
    text="Calibrate Camera / Start Tracking",
    command=run_python_opencv,
    height=2
).pack(pady=10)

tk.Button(
    root,
    text="Pick and Place Object",
    command=run_cpp,
    height=2
).pack(pady=5)

root.mainloop()