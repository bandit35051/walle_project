# data_logger.py
import csv
import time
import os

# Define the path to the sensor log file in a data folder.
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
os.makedirs(DATA_DIR, exist_ok=True)
DATA_FILE = os.path.join(DATA_DIR, "sensor_log.csv")

def log_sensor_data(battery, front_distance, rear_distance):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(DATA_FILE, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, battery, front_distance, rear_distance])
    print(f"Logged data at {timestamp}: Battery={battery}, Front={front_distance}, Rear={rear_distance}")

if __name__ == "__main__":
    # Example: Log dummy data every 5 seconds.
    while True:
        log_sensor_data(100, 50, 50)
        time.sleep(5)
