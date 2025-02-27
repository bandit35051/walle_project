import os
import json
import threading

# Set up directories relative to this file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "data")
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)
MEMORY_FILE = os.path.join(DATA_DIR, "wall_e_memory.json")

# Shared global state and locks
learned_expressions = {}
learned_movements = {}
feedback_memory = {}

wall_e_state = {
    "mood": "neutral",
    "battery": 100,
    "faces_detected": 0,
    "last_command": "None",
    "explored_areas": [],
    "cpu_usage": 0,
    "temperature": 0,
    "improvement_suggestions": []
}
state_lock = threading.Lock()

def save_memory(data=None):
    """Save the current state to disk."""
    try:
        to_save = data if data is not None else wall_e_state
        with open(MEMORY_FILE, 'w') as f:
            json.dump(to_save, f)
        print("Memory saved successfully.")
    except Exception as e:
        print(f"Failed to save memory: {e}")

def load_memory():
    """Load memory from disk."""
    try:
        with open(MEMORY_FILE, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"Failed to load memory: {e}")
        return {}
