import time
import RPi.GPIO as GPIO
import threading

# Import global state and speak function from other modules
try:
    from memory import wall_e_state
except ImportError:
    wall_e_state = {"mood": "neutral"}

try:
    from utils import wall_e_speak
except ImportError:
    def wall_e_speak(text):
        print(f"WALL-E says: {text}")

# Updated Servo Control Configuration
GPIO.setmode(GPIO.BCM)

# Servo Pinout Configuration
servo_pins = {
    'left_eye_tilt': 5,         # GPIO pin for left eye tilt
    'right_eye_tilt': 6,        # GPIO pin for right eye tilt
    'head_rotate': 7,           # GPIO pin for head rotation
    'neck_upper_tilt': 19,      # GPIO pin for upper neck tilt
    'neck_lower_tilt': 20,      # GPIO pin for lower neck tilt
    'right_arm_rotate': 12,     # GPIO pin for right arm rotation
    'left_arm_rotate': 13       # GPIO pin for left arm rotation
}

# Initialize PWM for each servo
servo_pwms = {}
for name, pin in servo_pins.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50Hz frequency for servo
    pwm.start(90)            # Start at neutral position (90 degrees)
    servo_pwms[name] = pwm

# Set servo angle function
def set_servo_angle(servo_name, angle):
    if servo_name in servo_pwms:
        duty = (angle / 18) + 2  # Convert angle to duty cycle
        servo_pwms[servo_name].ChangeDutyCycle(duty)
        time.sleep(0.5)
        servo_pwms[servo_name].ChangeDutyCycle(0)
    else:
        print(f"Servo '{servo_name}' not found.")

# Move all servos to default position
def reset_servos():
    for servo in servo_pwms:
        set_servo_angle(servo, 90)

# Arm movement functions
def move_arms(left_tilt=90, right_tilt=90):
    set_servo_angle('left_arm_rotate', left_tilt)
    set_servo_angle('right_arm_rotate', right_tilt)

# Neck movement functions
def move_neck(upper_tilt=90, lower_tilt=90):
    set_servo_angle('neck_upper_tilt', upper_tilt)
    set_servo_angle('neck_lower_tilt', lower_tilt)

# Head and eye movement functions
def move_head(rotate=90):
    set_servo_angle('head_rotate', rotate)

def move_eyes(left_tilt=90, right_tilt=90):
    set_servo_angle('left_eye_tilt', left_tilt)
    set_servo_angle('right_eye_tilt', right_tilt)

# Reset servos to default position at startup
reset_servos()

# Example servo usage
wall_e_speak("Testing servo movements.")
move_arms(45, 90)
move_neck(60, 90)
move_head(90)
move_eyes(45, 45)
time.sleep(2)
reset_servos()
wall_e_speak("Servo tests complete. Ready for operation.")

# Mood-Based Movement
def mood_based_movement(mood):
    if mood == "happy":
        move_arms(45, 90)
        move_neck(60, 90)
        wall_e_speak("I'm feeling happy and ready to explore!")
    elif mood == "sad":
        move_arms(135, 90)
        move_neck(120, 90)
        wall_e_speak("I'm feeling a bit down.")
    else:
        reset_servos()

# Example usage based on mood
wall_e_state["mood"] = "happy"
mood_based_movement(wall_e_state["mood"])

# Drive Motor Control
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23

motor_pins = [LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def move_forward():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def move_backward():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)

def stop_movement():
    for pin in motor_pins:
        GPIO.output(pin, GPIO.LOW)

if __name__ == '__main__':
    # For standalone testing purposes, run a simple test routine.
    wall_e_speak("Movement module test complete.")
