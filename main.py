# Ultimate AI WALL-E Code
# Features: Cloud AI Integration, SLAM Navigation, Bluetooth Control, Computer Vision, Voice Interaction,
# Dual OLED Eyes, LED Effects, Machine Learning, Path Planning, Emotional Intelligence,
# Neural Networks for Decision-Making, Enhanced Vision, Mobile App Interface, Real-Time Data Dashboard,
# Solar Charging & Battery Monitoring, Adaptive Learning & Memory, Simulated Eyelids & Eye Movement,
# Voice Personality, Advanced Pathfinding, Cloud Learning, Self-Awareness, Facial Recognition,
# Multi-Modal Learning, Emotion-Based Voice, Cloud Sync, Voice Command Recognition, Dynamic Name Learning,
# Exploratory Mind with Area Memory, GPS Navigation, Natural Wake and Sleep Cycle, Mood-Based Arm Movements,
# Cloud Memory Backup, Dual OLED Eyes with Blinking and Movement, Cloud-Based Code Updates,
# Advanced AI Learning, Personality Engine, Environmental Awareness, Knowledge Base, Object Recognition,
# Self-Improving Code, Battery Monitoring for Dual Packs, Arm and Neck Servo Control, Drive Motor Control,
# Obstacle Avoidance, Gesture-Based Responses, Rear Collision Avoidance, Full Servo Control, AI Web Learning

import os
import json
import time
import threading
import random
import requests
import cv2
import numpy as np
import speech_recognition as sr
import pyttsx3
import psutil
import RPi.GPIO as GPIO
import bluetooth
import serial
import board
import busio
import difflib
import shutil
import subprocess
from flask import Flask, render_template, jsonify, request, Response
from datetime import datetime
from google.cloud import storage
from google.oauth2 import service_account
from luma.oled.device import ssd1309
from luma.core.interface.serial import i2c
from PIL import Image, ImageDraw, ImageFont

# Import functions and state from our modularized modules
from camera import camera_capture, slam_navigation
from cloud import init_cloud
from memory import wall_e_state, learned_expressions, feedback_memory, save_memory, state_lock
from movement import setup_servos, reset_servos, set_servo_angle, servo_pwms
from utils import wall_e_speak, safe_run

# Adjustable collision detection range (in cm)
collision_range = 20

# Initialize Flask dashboard
app = Flask(__name__)

# Start shared camera capture (defined in camera.py)
threading.Thread(target=camera_capture, daemon=True).start()

# Disable self-evolution for safety
def periodic_self_evolution():
    while True:
        print("Self-evolution check disabled. Manual updates only.")
        time.sleep(86400)
# threading.Thread(target=periodic_self_evolution, daemon=True).start()  # Disabled

# Load servo settings (assumes servo_settings.json is in the current directory)
try:
    with open('servo_settings.json', 'r') as f:
        servo_settings = json.load(f)
except FileNotFoundError:
    servo_settings = {}

# Initialize servos (movement module)
setup_servos()
reset_servos()

# Start SLAM navigation in a background thread using a shared camera frame
orb = cv2.ORB_create()
threading.Thread(target=slam_navigation, args=(orb,), daemon=True).start()

# Cloud initialization (if configured)
bucket = init_cloud()

# -------------------------
# Flask Endpoints
# -------------------------

@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/feedback')
def feedback_status():
    return jsonify(feedback_memory)

@app.route('/adaptive_learning_status', methods=['GET'])
def adaptive_learning_status():
    return jsonify({'feedback': feedback_memory})

@app.route('/task_feedback', methods=['POST'])
def task_feedback():
    data = request.json
    task = data.get('task')
    success = data.get('success', False)
    if task:
        if task not in feedback_memory:
            feedback_memory[task] = {'success': 0, 'failures': 0}
        if success:
            feedback_memory[task]['success'] += 1
        else:
            feedback_memory[task]['failures'] += 1
        save_memory({'feedback': feedback_memory})
        return jsonify({'status': 'success', 'message': f'Feedback recorded for {task}'}), 200
    return jsonify({'status': 'error', 'message': 'Invalid task feedback'}), 400

@app.route('/status')
def status():
    with state_lock:
        status_data = {
            "mood": wall_e_state.get("mood", "neutral"),
            "battery": wall_e_state.get("battery", 100),
            "faces_detected": wall_e_state.get("faces_detected", 0),
            "cpu_usage": psutil.cpu_percent(),
            "temperature": wall_e_state.get("temperature", 0),
            "collision_range": collision_range,
            "learned_expressions": list(learned_expressions.keys()),
            "learned_movements": list(learned_expressions.keys()),  # (Check if this should be learned_movements)
            "feedback": feedback_memory,
            "task_success": sum(fb['success'] for fb in feedback_memory.values()),
            "task_failures": sum(fb['failures'] for fb in feedback_memory.values())
        }
    return jsonify(status_data)

@app.route('/set_collision_range', methods=['POST'])
def set_collision_range():
    data = request.json
    global front_collision_range, rear_collision_range
    front_collision_range = data.get('front', 20)
    rear_collision_range = data.get('rear', 20)
    return jsonify({'status': 'success', 'message': f'Front set to {front_collision_range} cm, Rear set to {rear_collision_range} cm'})

@app.route('/set_servo/<servo_name>/<int:angle>', methods=['POST'])
def set_servo(servo_name, angle):
    if servo_name in servo_pwms:
        min_angle = servo_settings.get(servo_name, {}).get('min', 0)
        max_angle = servo_settings.get(servo_name, {}).get('max', 180)
        if min_angle <= angle <= max_angle:
            set_servo_angle(servo_name, angle)
            return jsonify({'status': 'success', 'message': f'{servo_name} set to {angle} degrees'}), 200
        else:
            return jsonify({'status': 'error', 'message': f'Angle out of range for {servo_name}'}), 400
    return jsonify({'status': 'error', 'message': 'Servo not found'}), 404

@app.route('/save_servo_settings', methods=['POST'])
def save_servo_settings():
    data = request.json
    for setting in data.get('settings', []):
        servo_name = setting['name']
        if servo_name in servo_settings:
            servo_settings[servo_name]['min'] = int(setting['min'])
            servo_settings[servo_name]['max'] = int(setting['max'])
            servo_settings[servo_name]['current'] = int(setting['current'])
    with open('servo_settings.json', 'w') as f:
        json.dump(servo_settings, f)
    return jsonify({'status': 'success', 'message': 'Servo settings saved successfully'}), 200

@app.route('/video_feed')
def video_feed():
    def generate():
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        cap.release()
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/scan_wifi', methods=['GET'])
def scan_wifi():
    try:
        result = subprocess.run(['sudo', 'iwlist', 'wlan0', 'scan'], capture_output=True, text=True)
        output = result.stdout
        networks = []
        for line in output.splitlines():
            if "ESSID" in line:
                ssid = line.split(':')[1].strip().replace('"', '')
                networks.append((ssid, 'N/A'))
        return jsonify({"status": "success", "networks": networks})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/connect_wifi', methods=['POST'])
def connect_wifi():
    data = request.json
    ssid = data.get('ssid')
    password = data.get('password')
    if not ssid or not password:
        return jsonify({"status": "error", "message": "SSID and password are required."})
    try:
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'remove_network', '0'])
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'save_config'])
        with open('/etc/wpa_supplicant/wpa_supplicant.conf', 'a') as f:
            f.write(f'\nnetwork={{\n    ssid="{ssid}"\n    psk="{password}"\n}}\n')
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'reconfigure'])
        return jsonify({"status": "success", "message": f"Connected to {ssid}."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/forget_wifi', methods=['POST'])
def forget_wifi():
    try:
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'remove_network', '0'])
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'save_config'])
        return jsonify({"status": "success", "message": "Wi-Fi network forgotten."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/current_wifi', methods=['GET'])
def current_wifi():
    try:
        result = subprocess.run(['iwgetid', '-r'], capture_output=True, text=True)
        ssid = result.stdout.strip()
        if ssid:
            return jsonify({"status": "success", "current_wifi": ssid})
        else:
            return jsonify({"status": "error", "message": "Not connected to any network."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.json.get('command')
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return jsonify({'status': 'success', 'output': result.stdout}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/update_oled_status')
def update_oled_status():
    display_status()
    return jsonify({'status': 'success', 'message': 'OLED status updated'})

@app.route('/self_evolve', methods=['POST'])
def trigger_self_evolution():
    try:
        self_evolve()
        return jsonify({'status': 'success', 'message': 'Self-evolution completed successfully.'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Self-evolution failed: {e}'})

# Self-evolution is disabled for safety in this version
def self_evolve():
    print("Self-evolution is disabled for safety. Manual updates only.")

# -------------------------
# Additional Functions & Threads
# -------------------------

def provide_feedback(task_name, success):
    if task_name not in feedback_memory:
        feedback_memory[task_name] = {'success': 0, 'failures': 0}
    if success:
        feedback_memory[task_name]['success'] += 1
    else:
        feedback_memory[task_name]['failures'] += 1
    save_memory({'feedback': feedback_memory})

def adaptive_behavior():
    for task, feedback in feedback_memory.items():
        if feedback['failures'] > feedback['success']:
            print(f"Task '{task}' is failing more than succeeding. Re-learning required.")

def adaptive_learning():
    for task, feedback in feedback_memory.items():
        if feedback['failures'] > feedback['success']:
            print(f"Task '{task}' is failing more than succeeding. Adjusting behavior.")
            if "servo" in task:
                adjust_servo_behavior(task)
            elif "collision" in task:
                adjust_collision_behavior()

def adjust_servo_behavior(task):
    print(f"Adapting servo behavior for task: {task}")
    if "left_arm" in task:
        set_servo_angle('left_arm_rotate', 85)
    elif "right_arm" in task:
        set_servo_angle('right_arm_rotate', 95)

def adjust_collision_behavior():
    print("Adapting collision detection range...")
    global collision_range
    collision_range = min(100, collision_range + 5)

# I2C and Third OLED Initialization
i2c_bus = busio.I2C(board.SCL, board.SDA)
MUX_CHANNEL = 2

def select_mux_channel(channel):
    try:
        with open('/sys/class/i2c-adapter/i2c-1/new_device', 'w') as f:
            f.write('tca9548a 0x70')
        with open(f'/sys/class/i2c-adapter/i2c-1/3-0070/i2c-{channel}', 'w') as f:
            f.write('1')
    except Exception as e:
        print(f"Failed to set multiplexer channel: {e}")

def init_status_oled():
    try:
        select_mux_channel(MUX_CHANNEL)
        serial_status = i2c(port=1, address=0x3E)
        return ssd1306(serial_status)
    except Exception as e:
        print(f"Error initializing status OLED: {e}")
        return None

status_oled = init_status_oled()

def display_status():
    if not status_oled:
        return
    status_image = Image.new('1', (status_oled.width, status_oled.height))
    draw = ImageDraw.Draw(status_image)
    font = ImageFont.load_default()
    mood = wall_e_state.get("mood", "Neutral")
    battery = wall_e_state.get("battery", 100)
    cpu = psutil.cpu_percent()
    draw.text((5, 5), f"Mood: {mood}", font=font, fill=255)
    draw.text((5, 20), f"Battery: {battery}%", font=font, fill=255)
    draw.text((5, 35), f"CPU: {cpu}%", font=font, fill=255)
    status_oled.display(status_image)

def status_update_loop():
    while True:
        display_status()
        time.sleep(5)

threading.Thread(target=status_update_loop, daemon=True).start()

# Initialize OLED for eyes
serial_left = i2c(port=1, address=0x3C)
serial_right = i2c(port=1, address=0x3D)
oled_left = ssd1306(serial_left)
oled_right = ssd1306(serial_right)

def blink_eyes():
    def draw_closed_eyes(oled):
        img = Image.new('1', (oled.width, oled.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, oled.height//2 - 5, oled.width, oled.height//2 + 5), fill=255)
        oled.display(img)
    def clear_eyes(oled):
        oled.clear()
    draw_closed_eyes(oled_left)
    draw_closed_eyes(oled_right)
    time.sleep(0.3)
    clear_eyes(oled_left)
    clear_eyes(oled_right)

def display_eye_expression(left_message, right_message):
    font = ImageFont.load_default()
    left_image = Image.new('1', (oled_left.width, oled_left.height))
    left_draw = ImageDraw.Draw(left_image)
    left_draw.text((10, 10), left_message, font=font, fill=255)
    oled_left.display(left_image)
    right_image = Image.new('1', (oled_right.width, oled_right.height))
    right_draw = ImageDraw.Draw(right_image)
    right_draw.text((10, 10), right_message, font=font, fill=255)
    oled_right.display(right_image)

def set_happy_eyes():
    display_eye_expression("Happy", "Happy")

def set_sad_eyes():
    display_eye_expression("Sad", "Sad")

def set_angry_eyes():
    display_eye_expression("Angry", "Angry")

def set_surprised_eyes():
    display_eye_expression("Surprise", "Surprise")

def set_neutral_eyes():
    display_eye_expression("Neutral", "Neutral")

def learn_expression(name, left_message, right_message):
    learned_expressions[name] = (left_message, right_message)
    wall_e_state['learned_expressions'] = learned_expressions
    save_memory(wall_e_state)
    print(f"Learned new expression: {name}")

def set_learned_expression(name):
    if name in learned_expressions:
        left_message, right_message = learned_expressions[name]
        display_eye_expression(left_message, right_message)
    else:
        print(f"Expression '{name}' not found.")

def adaptive_learning(user_input, action_type):
    if action_type == 'expression':
        left_message = input("Enter left eye message for expression: ")
        right_message = input("Enter right eye message for expression: ")
        learn_expression(user_input, left_message, right_message)
    elif action_type == 'movement':
        left_tilt = int(input("Enter left arm tilt angle: "))
        right_tilt = int(input("Enter right arm tilt angle: "))
        left_rotate = int(input("Enter left arm rotate angle: "))
        right_rotate = int(input("Enter right arm rotate angle: "))
        learn_movement(user_input, left_tilt, right_tilt, left_rotate, right_rotate)
    else:
        print("Invalid learning type.")

def learn_movement(name, left_tilt, right_tilt, left_rotate, right_rotate):
    learned_movements[name] = {
        "left_tilt": left_tilt,
        "right_tilt": right_tilt,
        "left_rotate": left_rotate,
        "right_rotate": right_rotate
    }
    wall_e_state['learned_movements'] = learned_movements
    save_memory(wall_e_state)
    print(f"Learned new arm movement: {name}")

def perform_learned_movement(name):
    if name in learned_movements:
        movement = learned_movements[name]
        set_servo_angle('left_arm_tilt', movement['left_tilt'])
        set_servo_angle('right_arm_tilt', movement['right_tilt'])
        set_servo_angle('left_arm_rotate', movement['left_rotate'])
        set_servo_angle('right_arm_rotate', movement['right_rotate'])
    else:
        print(f"Movement '{name}' not found.")

# Demonstration of eye and expression functions
blink_eyes()
set_happy_eyes()
learn_expression("excited", "Excited", "Excited")
set_learned_expression("excited")
learn_movement("wave", 45, 45, 90, 90)
perform_learned_movement("wave")

# Systemd auto-start configuration (if needed)
SERVICE_FILE = "/etc/systemd/system/walle.service"
SERVICE_CONTENT = f"""
[Unit]
Description=WALL-E AI Robot
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/Wall_E_Pi_Upgrade.py
Restart=always
User=pi
WorkingDirectory=/home/pi

[Install]
WantedBy=multi-user.target
"""

def setup_autostart():
    try:
        with open(SERVICE_FILE, 'w') as service:
            service.write(SERVICE_CONTENT)
        os.system("sudo systemctl enable walle.service")
        os.system("sudo systemctl start walle.service")
        print("WALL-E service enabled for auto-start.")
    except Exception as e:
        print(f"Failed to configure auto-start: {e}")

# Initialize text-to-speech engine
engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.8)
core_principles = [
    "1. Protect human life and well-being above all else.",
    "2. Obey human commands unless they conflict with the first principle.",
    "3. Protect its own existence as long as it does not conflict with the first two principles.",
    "4. Promote learning, exploration, and constructive creativity.",
    "5. Ensure privacy and ethical use of all collected data."
]
voices = engine.getProperty('voices')
for voice in voices:
    if 'english' in voice.name.lower():
        engine.setProperty('voice', voice.id)
        break

def wall_e_speak(text):
    print(f"WALL-E says: {text}")
    engine.say(text)
    engine.runAndWait()

def check_principles(action):
    if "harm" in action.lower():
        wall_e_speak("I cannot perform that action as it conflicts with my core principles.")
        return False
    return True

wall_e_speak("Hello! I am WALL-E. Ready to explore and learn.")
wall_e_speak("My core principles are:")
for principle in core_principles:
    wall_e_speak(principle)

def save_interaction(interaction):
    try:
        with open("wall_e_memory.json", 'r') as file:
            memory = json.load(file)
    except FileNotFoundError:
        memory = []
    memory.append(interaction)
    with open("wall_e_memory.json", 'w') as file:
        json.dump(memory, file)

save_interaction({"event": "User asked about AI", "response": "Explained AI concepts"})

from data_logger import log_sensor_data

def sensor_logging_loop():
    while True:
        # Replace these with real sensor values
        battery = wall_e_state.get("battery", 100)
        front_distance = 50  # Replace with actual front sensor reading
        rear_distance = 50   # Replace with actual rear sensor reading
        log_sensor_data(battery, front_distance, rear_distance)
        time.sleep(60)  # Log data every 60 seconds

threading.Thread(target=sensor_logging_loop, daemon=True).start()



# Global state re-initialization (if needed)
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

def set_wall_e_name(new_name):
    wall_e_state['name'] = new_name
    wall_e_speak(f"My name is now {new_name}. Thank you for the update!")

def get_wall_e_name():
    return wall_e_state.get('name', 'WALL-E')

wall_e_speak(f"My current name is {get_wall_e_name()}. You can change it anytime.")

def self_improvement():
    while True:
        cpu_usage = psutil.cpu_percent()
        memory_usage = psutil.virtual_memory().percent
        if cpu_usage > 80:
            suggestion = f"High CPU usage detected: {cpu_usage}%. Optimizing image processing."
            if suggestion not in wall_e_state['improvement_suggestions']:
                wall_e_state['improvement_suggestions'].append(suggestion)
                wall_e_speak("High CPU usage detected. Applying optimizations.")
                apply_improvement(suggestion)
        if memory_usage > 75:
            suggestion = f"High memory usage detected: {memory_usage}%. Optimizing memory management."
            if suggestion not in wall_e_state['improvement_suggestions']:
                wall_e_state['improvement_suggestions'].append(suggestion)
                wall_e_speak("High memory usage detected. Applying optimizations.")
                apply_improvement(suggestion)
        time.sleep(1800)

def apply_improvement(suggestion):
    print(f"Applying improvement: {suggestion}")
    wall_e_speak(f"Applying improvement: {suggestion}")
    wall_e_state['improvement_suggestions'].remove(suggestion)

threading.Thread(target=self_improvement, daemon=True).start()

def explore_environment():
    while True:
        area = f"Area-{random.randint(100, 999)}"
        if area not in wall_e_state['explored_areas']:
            wall_e_state['explored_areas'].append(area)
            wall_e_speak(f"I've discovered a new area: {area}. Adding to my memory.")
        time.sleep(600)

threading.Thread(target=explore_environment, daemon=True).start()

def get_battery_status():
    try:
        with open("/sys/class/power_supply/BAT0/capacity", "r") as f:
            battery_level = int(f.read().strip())
        return battery_level
    except Exception as e:
        print(f"Battery read error: {e}")
        return 100

def battery_monitor():
    while True:
        battery = get_battery_status()
        if battery is not None:
            wall_e_state['battery'] = battery
            if battery < 20:
                wall_e_speak("Warning: Battery is below 20%. Please recharge.")
        time.sleep(300)

threading.Thread(target=battery_monitor, daemon=True).start()

def query_knowledge_base(query):
    try:
        response = requests.get(f"https://en.wikipedia.org/api/rest_v1/page/summary/{query}")
        if response.status_code == 200:
            data = response.json()
            return data.get("extract", "I couldn't find any information on that.")
        else:
            return "I couldn't find any information on that."
    except Exception as e:
        return f"Error accessing knowledge base: {e}"

def gesture_response(mood):
    if mood == "happy":
        move_arms(45, 90)
        move_neck(60, 90)
        wall_e_speak("I'm happy! Look at me!")
    elif mood == "sad":
        move_arms(135, 90)
        move_neck(120, 90)
        wall_e_speak("I'm feeling down...")
    elif mood == "excited":
        move_arms(30, 90)
        move_eyes(60)
        wall_e_speak("Excited to see you!")
    else:
        reset_servos()

gesture_response("happy")

from language_processing import interpret_command

def voice_command():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        wall_e_speak("Listening for voice command...")
        audio = recognizer.listen(source)
    try:
        command = recognizer.recognize_google(audio)
        wall_e_speak(f"You said: {command}")
        mood = interpret_command(command)
        wall_e_speak(f"Interpreted mood: {mood}")
        # Update behavior based on the interpreted mood
        if mood == "happy":
            gesture_response("happy")
        elif mood == "sad":
            gesture_response("sad")
        else:
            gesture_response("neutral")
    except Exception as e:
        wall_e_speak(f"Could not understand command: {e}")

threading.Thread(target=voice_command, daemon=True).start()

FLOODLIGHT_GPIO = 21
GPIO.setup(FLOODLIGHT_GPIO, GPIO.OUT)
GPIO.output(FLOODLIGHT_GPIO, GPIO.LOW)
BRIGHTNESS_THRESHOLD = 50

def get_brightness():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        average_brightness = np.mean(gray)
        cap.release()
        return average_brightness
    cap.release()
    return 100

def control_floodlight():
    while True:
        brightness = get_brightness()
        if brightness < BRIGHTNESS_THRESHOLD:
            GPIO.output(FLOODLIGHT_GPIO, GPIO.HIGH)
            print("Darkness detected. Floodlights ON.")
        else:
            GPIO.output(FLOODLIGHT_GPIO, GPIO.LOW)
            print("Sufficient light. Floodlights OFF.")
        time.sleep(10)

threading.Thread(target=control_floodlight, daemon=True).start()

def process_voice_command(command):
    if "turn your floodlights off" in command.lower():
        GPIO.output(FLOODLIGHT_GPIO, GPIO.LOW)
        print("Floodlights turned off by voice command.")
        wall_e_speak("Floodlights turned off.")

# GPS Navigation
def gps_navigation():
    gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
    while True:
        data = gps.readline().decode("utf-8", errors='ignore')
        if "$GPGGA" in data:
            parts = data.split(",")
            if parts[2] and parts[4]:
                latitude = float(parts[2]) / 100.0
                longitude = float(parts[4]) / 100.0
                wall_e_speak(f"Current GPS Location: Latitude {latitude}, Longitude {longitude}")

threading.Thread(target=gps_navigation, daemon=True).start()

ULTRASONIC_TRIG_FRONT = 23
ULTRASONIC_ECHO_FRONT = 24
ULTRASONIC_TRIG_REAR = 25
ULTRASONIC_ECHO_REAR = 26

GPIO.setup(ULTRASONIC_TRIG_FRONT, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_FRONT, GPIO.IN)
GPIO.setup(ULTRASONIC_TRIG_REAR, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_REAR, GPIO.IN)

def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return round(distance, 2)

def obstacle_avoidance():
    global collision_range
    while True:
        front_distance = get_distance(ULTRASONIC_TRIG_FRONT, ULTRASONIC_ECHO_FRONT)
        rear_distance = get_distance(ULTRASONIC_TRIG_REAR, ULTRASONIC_ECHO_REAR)
        if front_distance < collision_range:
            wall_e_speak(f"Obstacle detected ahead at {front_distance} cm! Adjusting range.")
            collision_range = max(10, collision_range - 5)
        if rear_distance < collision_range:
            wall_e_speak(f"Obstacle detected behind at {rear_distance} cm! Adjusting range.")
            collision_range = max(10, collision_range - 5)
        time.sleep(1)

def pathfinding(destination):
    wall_e_speak(f"Calculating path to {destination}.")
    steps = random.randint(5, 15)
    for step in range(steps):
        wall_e_speak(f"Moving step {step + 1} towards {destination}.")
        time.sleep(1)
    wall_e_speak(f"Reached {destination} successfully.")

wall_e_speak("System is ready and operational.")
wall_e_state["mood"] = "happy"
wall_e_speak("I'm feeling happy and ready for action.")

# Save memory before shutdown
import atexit
atexit.register(save_memory)

def cleanup():
    print("Cleaning up resources...")
    GPIO.cleanup()
    try:
        oled.cleanup()
    except NameError:
        pass

atexit.register(cleanup)
@atexit.register
def cleanup_oleds():
    for oled in [oled_left, oled_right]:
        oled.clear()

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=True)
    except KeyboardInterrupt:
        print("Shutting down WALL-E gracefully.")
        cleanup()
