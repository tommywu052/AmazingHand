#!/usr/bin/env python3
"""
AmazingHand Tracker - Simplified Version
A simplified version that uses basic inverse kinematics without complex Mujoco setup.
Now integrated with scservo_sdk for reliable motor control.
"""

import argparse
import cv2
import numpy as np
import time
import json
import os
import math
from pathlib import Path
import mediapipe as mp
import threading

# MediaPipe setup
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
HandLandmark = mp.solutions.hands.HandLandmark

# SCServo SDK imports
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except Exception as e:
    print(f"ERROR: scservo_sdk not found: {e}")
    print("Please ensure the scservo_sdk folder is in the current directory")
    raise

# SCS0009 constants from working code
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POS = 42
ADDR_RUN_TIME = 44
ADDR_RUN_SPEED = 46

# Motor control constants
DEFAULT_TIME = 200
DEFAULT_SPEED = 800
CENTER_POS = 512  # Center position for 0-1023 range
OPEN_POS = 300
CLOSE_POS = 700

# Control parameters
LP_ALPHA = 0.25
SEND_PERIOD = 0.04

class SimpleAmazingHandController:
    """Simplified controller for the AmazingHand robot."""
    
    def __init__(self, config_file="config/r_hand.json", serial_port="COM3", 
                 baudrate=1000000, simulation_mode=False, hand_side="right"):
        """
        Initialize the simplified AmazingHand controller.
        
        Args:
            config_file: Path to motor configuration file
            serial_port: Serial port for motor communication
            baudrate: Baud rate for serial communication
            simulation_mode: If True, only run simulation without real motors
            hand_side: "right" or "left" hand
        """
        self.simulation_mode = simulation_mode
        self.hand_side = hand_side
        self.serial_port = serial_port
        self.baudrate = baudrate
        
        # Load motor configuration
        self.load_motor_config(config_file)
        
        # Initialize components
        if not simulation_mode:
            self.init_motor_controller()
        
        self.init_hand_tracking()
        
        # Control parameters
        self.fps = 30
        self.running = False
        self.use_advanced_ik = False  # Default to simple IK
        self.use_curl_ik = True       # Default to curl-based IK (better thumb control)
        self.debug_enabled = False    # Default to no debug output
        
        # IK parameters
        self.finger_lengths = [0.04, 0.04, 0.04, 0.04]  # Length of each finger segment
        self.max_angles = [np.pi/2, np.pi/2]  # Max angles for each joint
        
    def load_motor_config(self, config_file):
        """Load motor configuration from JSON file."""
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.motor_config = json.load(f)
        else:
            # Default configuration for right hand
            self.motor_config = {
                "fingers": [
                    {
                        "name": "finger1",
                        "motor1": {"id": 1, "offset": 0.12217304763960307, "invert": False},
                        "motor2": {"id": 2, "offset": 0.08726646259971647, "invert": False}
                    },
                    {
                        "name": "finger2", 
                        "motor1": {"id": 3, "offset": 0.0, "invert": False},
                        "motor2": {"id": 4, "offset": 0.12217304763960307, "invert": False}
                    },
                    {
                        "name": "finger3",
                        "motor1": {"id": 5, "offset": 0.08726646259971647, "invert": False},
                        "motor2": {"id": 6, "offset": 0.12217304763960307, "invert": False}
                    },
                    {
                        "name": "finger4",
                        "motor1": {"id": 7, "offset": 0.0, "invert": False},
                        "motor2": {"id": 8, "offset": 0.12217304763960307, "invert": False}
                    }
                ]
            }
            print(f"Using default motor configuration. Save to {config_file} to customize.")
    
    def init_motor_controller(self):
        """Initialize the motor controller for SCS0009 servos using scservo_sdk."""
        try:
            print(f"Attempting to connect to motors on {self.serial_port}...")
            
            # Initialize PortHandler
            self.port_handler = PortHandler(self.serial_port)
            if not self.port_handler.openPort():
                print(f"Failed to open port {self.serial_port}")
                return
            
            # Set port baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                print(f"Failed to set baudrate {self.baudrate}")
                return
            
            # Initialize PacketHandler
            self.packet_handler = PacketHandler(self.port_handler)
            
            # Get motor IDs from config
            motor_ids = []
            for finger in self.motor_config["fingers"]:
                motor_ids.extend([finger["motor1"]["id"], finger["motor2"]["id"]])
            
            print(f"Motor IDs: {motor_ids}")
            
            # Test communication with each motor using ping
            print("Testing motor communication...")
            found_motors = []
            for motor_id in motor_ids:
                try:
                    model, res, err = self.packet_handler.ping(self.port_handler, motor_id)
                    if res == COMM_SUCCESS and err == 0:
                        print(f"✓ Motor {motor_id} responds (model={model})")
                        found_motors.append(motor_id)
                    else:
                        print(f"⚠ Motor {motor_id} no response")
                except Exception as e:
                    print(f"✗ Motor {motor_id} error: {e}")
            
            if not found_motors:
                print("No motors found! Running in simulation mode only")
                self.simulation_mode = True
                return
            
            # Initialize found motors
            print("Initializing found motors...")
            self.bus_init(found_motors)
            
            print("✓ Motor initialization completed")
            
        except Exception as e:
            print(f"✗ Failed to connect to motors: {e}")
            print("Running in simulation mode only")
            self.simulation_mode = True
    
    def bus_init(self, motor_ids):
        """Initialize motors: set position mode, enable torque, and center."""
        print("[DIAG] Forcing Position Mode, enabling torque, and centering...")
        
        # Set position mode for all motors
        for motor_id in motor_ids:
            self.set_position_mode(motor_id)
        time.sleep(0.05)
        
        # Enable torque for all motors
        for motor_id in motor_ids:
            self.set_torque(motor_id, True)
        time.sleep(0.05)
        
        # Move motors to center position
        for motor_id in motor_ids:
            self.move_servo(motor_id, CENTER_POS, run_time=250, run_speed=600)
        time.sleep(0.3)
    
    def set_position_mode(self, motor_id):
        """Set motor to position mode."""
        return self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_MODE, 0)
    
    def set_torque(self, motor_id, on=True):
        """Enable or disable torque for a motor."""
        return self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1 if on else 0)
    
    def move_servo(self, motor_id, pos, run_time=DEFAULT_TIME, run_speed=DEFAULT_SPEED):
        """Move servo to position with specified time and speed."""
        pos = int(max(0, min(1023, pos))) & 0xFFFF
        run_time = int(max(0, min(1023, run_time))) & 0xFFFF
        run_speed = int(max(0, min(1023, run_speed))) & 0xFFFF
        
        print(f"DEBUG: move_servo - motor {motor_id}, pos {pos}, time {run_time}, speed {run_speed}")
        
        try:
            result1, error1 = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_POS, pos)
            result2, error2 = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_RUN_TIME, run_time)
            result3, error3 = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_RUN_SPEED, run_speed)
            
            print(f"DEBUG: move_servo results - pos: ({result1}, {error1}), time: ({result2}, {error2}), speed: ({result3}, {error3})")
            
            # Check if any command failed - COMM_SUCCESS is 0, so we check for 0
            if result1 == 0 and result2 == 0 and result3 == 0:
                print(f"DEBUG: move_servo SUCCESS for motor {motor_id}")
                return True
            else:
                print(f"DEBUG: move_servo failed for motor {motor_id} - pos: ({result1}, {error1}), time: ({result2}, {error2}), speed: ({result3}, {error3})")
                return False
                
        except Exception as e:
            print(f"DEBUG: move_servo exception for motor {motor_id}: {e}")
            return False
    
    def init_hand_tracking(self):
        """Initialize MediaPipe hand tracking."""
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        print("Hand tracking initialized")
    
    def process_hand_landmarks(self, image):
        """Process hand landmarks from MediaPipe."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        
        if results.multi_hand_landmarks and results.multi_hand_world_landmarks:
            for idx, handedness in enumerate(results.multi_handedness):
                # Check if it's the correct hand
                if (handedness.classification[0].label == 'Right' and 
                    handedness.classification[0].score > 0.8 and 
                    self.hand_side == "right"):
                    
                    hand_landmarks = results.multi_hand_world_landmarks[idx]
                    hand_landmarks_norm = results.multi_hand_landmarks[idx]
                    
                    # Extract finger tip positions and angles
                    finger_data = self.extract_finger_data(hand_landmarks)
                    
                    return finger_data, hand_landmarks_norm
        
        return None, None
    
    def extract_finger_data(self, hand_landmarks):
        """Extract finger data including positions and angles."""
        
        finger_data = {}
        finger_mappings = {
            'finger1': [
                HandLandmark.INDEX_FINGER_MCP,
                HandLandmark.INDEX_FINGER_PIP,
                HandLandmark.INDEX_FINGER_DIP,
                HandLandmark.INDEX_FINGER_TIP
            ],
            'finger2': [
                HandLandmark.MIDDLE_FINGER_MCP,
                HandLandmark.MIDDLE_FINGER_PIP,
                HandLandmark.MIDDLE_FINGER_DIP,
                HandLandmark.MIDDLE_FINGER_TIP
            ],
            'finger3': [
                HandLandmark.RING_FINGER_MCP,
                HandLandmark.RING_FINGER_PIP,
                HandLandmark.RING_FINGER_DIP,
                HandLandmark.RING_FINGER_TIP
            ],
            'finger4': [
                HandLandmark.THUMB_MCP,
                HandLandmark.THUMB_IP,
                HandLandmark.THUMB_TIP
            ]
        }
        
        for finger_name, landmarks in finger_mappings.items():
            if len(landmarks) == 4:  # 3-joint finger
                positions = []
                for landmark in landmarks:
                    pos = hand_landmarks.landmark[landmark]
                    positions.append([pos.x, pos.y, pos.z])
                
                # Calculate angles between segments
                angles = self.calculate_finger_angles(positions)
                finger_data[finger_name] = {
                    'positions': positions,
                    'angles': angles
                }
            elif len(landmarks) == 3:  # 2-joint finger (thumb)
                positions = []
                for landmark in landmarks:
                    pos = hand_landmarks.landmark[landmark]
                    positions.append([pos.x, pos.y, pos.z])
                
                # Calculate angles for thumb
                angles = self.calculate_finger_angles(positions)
                
                # For thumb, calculate an additional angle based on thumb orientation
                # This helps coordinate the two thumb motors better
                if len(angles) >= 1:
                    # Calculate thumb orientation relative to palm
                    thumb_mcp = np.array(positions[0])
                    thumb_tip = np.array(positions[2])
                    
                    # Get wrist position for palm reference
                    wrist_pos = np.array([
                        hand_landmarks.landmark[HandLandmark.WRIST].x,
                        hand_landmarks.landmark[HandLandmark.WRIST].y,
                        hand_landmarks.landmark[HandLandmark.WRIST].z
                    ])
                    
                    # Calculate thumb spread angle (sideways movement)
                    palm_vector = thumb_mcp - wrist_pos
                    thumb_vector = thumb_tip - thumb_mcp
                    
                    # Project onto palm plane and calculate spread angle
                    palm_normal = np.array([0, 0, 1])  # Assuming palm faces camera
                    palm_vector_proj = palm_vector - np.dot(palm_vector, palm_normal) * palm_normal
                    thumb_vector_proj = thumb_vector - np.dot(thumb_vector, palm_normal) * palm_normal
                    
                    if np.linalg.norm(palm_vector_proj) > 0 and np.linalg.norm(thumb_vector_proj) > 0:
                        cos_spread = np.dot(palm_vector_proj, thumb_vector_proj) / (np.linalg.norm(palm_vector_proj) * np.linalg.norm(thumb_vector_proj))
                        cos_spread = np.clip(cos_spread, -1.0, 1.0)
                        spread_angle = np.arccos(cos_spread)
                        angles.append(spread_angle)
                    else:
                        angles.append(0.0)
                
                finger_data[finger_name] = {
                    'positions': positions,
                    'angles': angles
                }
        
        return finger_data
    
    def calculate_finger_angles(self, positions):
        """Calculate angles between finger segments."""
        if len(positions) < 3:
            return []
        
        angles = []
        for i in range(len(positions) - 2):
            # Calculate vectors between three consecutive points
            v1 = np.array(positions[i+1]) - np.array(positions[i])
            v2 = np.array(positions[i+2]) - np.array(positions[i+1])
            
            # Calculate angle between vectors
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Clamp to valid range
            angle = np.arccos(cos_angle)
            angles.append(angle)
        
        return angles
    
    def compute_curls_from_landmarks(self, hand_landmarks):
        """Compute finger curls based on distance between fingertip and MCP joint (like reference file)."""
        # Debug: Check input
        print(f"DEBUG: compute_curls_from_landmarks called with hand_landmarks: {hand_landmarks}")
        if hand_landmarks is None:
            print("DEBUG: hand_landmarks is None, returning empty curls")
            return {}
        
        # Define finger landmark pairs: (tip, mcp)
        finger_pairs = {
            'finger1': (HandLandmark.INDEX_FINGER_TIP, HandLandmark.INDEX_FINGER_MCP),
            'finger2': (HandLandmark.MIDDLE_FINGER_TIP, HandLandmark.MIDDLE_FINGER_MCP),
            'finger3': (HandLandmark.RING_FINGER_TIP, HandLandmark.RING_FINGER_MCP),
            'finger4': (HandLandmark.THUMB_TIP, HandLandmark.THUMB_MCP)
        }
        
        curls = {}
        for finger_name, (tip, mcp) in finger_pairs.items():
            try:
                # Get positions
                tip_pos = hand_landmarks.landmark[tip]
                mcp_pos = hand_landmarks.landmark[mcp]
                
                print(f"DEBUG: {finger_name} - tip_pos: ({tip_pos.x:.3f}, {tip_pos.y:.3f}), mcp_pos: ({mcp_pos.x:.3f}, {mcp_pos.y:.3f})")
                
                # Calculate 2D distance (ignore Z for curl calculation)
                d = math.hypot(tip_pos.x - mcp_pos.x, tip_pos.y - mcp_pos.y)
                
                # Map distance to curl value (0.0 = fully open, 1.0 = fully closed)
                # These values are tuned based on the reference file approach
                if finger_name == 'finger4':  # Thumb - improved curl detection for better alignment
                    # Improved curl detection for thumb - more sensitive and better aligned with MediaPipe
                    # This should make the thumb movement more obvious and correctly aligned
                    c = max(0.0, min(1.0, (0.30 - d) / 0.20))
                else:  # Other fingers - much more sensitive curl detection for maximum bending
                    # Make other fingers much more sensitive to allow them to bend fully
                    c = max(0.0, min(1.0, (0.20 - d) / 0.12))  # Even more sensitive range
                
                print(f"DEBUG: {finger_name} - distance: {d:.3f}, curl: {c:.3f}")
                
                curls[finger_name] = c
                
            except (IndexError, AttributeError) as e:
                print(f"DEBUG: Error processing {finger_name}: {e}")
                continue
        
        print(f"DEBUG: Final curls dictionary: {curls}")
        return curls
    
    def curl_based_inverse_kinematics(self, hand_landmarks):
        """Simple curl-based IK that maps finger curls directly to servo positions."""
        print(f"DEBUG: curl_based_inverse_kinematics called with hand_landmarks: {hand_landmarks}")
        
        curls = self.compute_curls_from_landmarks(hand_landmarks)
        print(f"DEBUG: Received curls: {curls}")
        
        if not curls:
            print("DEBUG: No curls received, returning empty joint_positions")
            return {}
        
        # Configuration for curl-based control
        CENTER_POS = 512
        # Increase gain for more aggressive bending - allow fingers to bend more fully
        # Index finger gets highest gain for maximum bending, others slightly less
        # INCREASED: Thumb gain for more obvious movement
        CURL_GAIN_PER = {"finger1": 600, "finger2": 550, "finger3": 520, "finger4": 650}
        DIR = {
            "finger1": (+1, -1),  # Base and middle joints move in opposite directions
            "finger2": (+1, -1),
            "finger3": (+1, -1),
            "finger4": (+1, -1)
        }
        # Center offset to compensate for the left bias when stretched
        # Reduced values since we adjusted base positions
        CENTER_OFFSET = {"finger1": 30, "finger2": 30, "finger3": 30, "finger4": 0}
        # Increase gamma bias for more aggressive closing movement
        GAMMA = 0.80  # More aggressive bias toward "closed" for better bending
        
        # Reduce movement smoothing to allow more responsive movement
        MOVEMENT_SMOOTHING = 0.95  # Smooth factor (0.0 = no movement, 1.0 = full movement)
        
        # Per-finger position limits - using values from working reference file
        # These values provide better balance between stretching and bending
        OPEN_POS_PER = {"finger1": 300, "finger2": 300, "finger3": 300, "finger4": 320}  # From reference file
        CLOSE_POS_PER = {"finger1": 880, "finger2": 900, "finger3": 900, "finger4": 880}  # From reference file
        
        joint_positions = {}
        
        # Get the actual finger configurations from the list structure
        finger_configs = self.motor_config.get('fingers', [])
        print(f"DEBUG: Available finger configs: {[f.get('name', 'unknown') for f in finger_configs]}")
        
        for finger_name in curls.keys():
            # Find the matching finger configuration
            finger_config = None
            for config in finger_configs:
                if config.get('name') == finger_name:
                    finger_config = config
                    break
            
            if not finger_config:
                print(f"DEBUG: {finger_name} not found in finger_configs")
                continue
            
            print(f"DEBUG: Processing {finger_name}: {finger_config}")
            
            # Get motor IDs for this finger
            motor1_id = finger_config.get('motor1', {}).get('id')
            motor2_id = finger_config.get('motor2', {}).get('id')
            
            if motor1_id is None or motor2_id is None:
                print(f"DEBUG: {finger_name} missing motor IDs: motor1={motor1_id}, motor2={motor2_id}")
                continue
            
            motor_ids = [motor1_id, motor2_id]
            print(f"DEBUG: {finger_name} motor IDs: {motor_ids}")
            
            if finger_name == 'finger4':  # Thumb - FIXED direction and increased movement
                # FIXED: Invert the curl value to correct the direction
                # When MediaPipe shows thumb stretched (curl=0), we want robot thumb stretched (curl=0)
                # When MediaPipe shows thumb bent (curl=1), we want robot thumb bent (curl=1)
                v = max(0.0, min(1.0, curls[finger_name]))
                
                # INCREASED: Use higher gamma for more obvious movement
                thumb_gamma = 0.70  # More aggressive for obvious movement
                if thumb_gamma != 1.0:
                    v = v ** (1.0 / thumb_gamma)
                
                # INCREASED: Calculate delta with higher gain for more obvious movement
                gain = CURL_GAIN_PER.get(finger_name, 300)
                delta = (v - 0.5) * 2.0 * gain
                
                # INCREASED: Apply higher movement smoothing for more obvious movement
                thumb_smoothing = 0.98  # More obvious movement
                delta = delta * thumb_smoothing
                
                # FIXED: Invert the direction mapping to correct the movement
                # This fixes the "bends when should stretch, stretches when should bend" issue
                servo1_pos = CENTER_POS - DIR[finger_name][0] * delta  # Note the minus sign
                servo2_pos = CENTER_POS - DIR[finger_name][1] * delta  # Note the minus sign
                
                # Clamp to finger-specific limits
                servo1_pos = max(OPEN_POS_PER[finger_name], min(CLOSE_POS_PER[finger_name], int(servo1_pos)))
                servo2_pos = max(OPEN_POS_PER[finger_name], min(CLOSE_POS_PER[finger_name], int(servo2_pos)))
                
                # Store positions for both motors
                joint_positions[motor1_id] = servo1_pos
                joint_positions[motor2_id] = servo2_pos
                
                print(f"  {finger_name}: curl={v:.3f}, gamma={thumb_gamma}, smoothing={thumb_smoothing}, servo1={servo1_pos}, servo2={servo2_pos}")
                    
            else:  # Other fingers - use simple curl-based approach WITHOUT lateral movement
                # 1) Apply gamma bias toward "closed" (like reference file)
                v = max(0.0, min(1.0, curls[finger_name]))
                if GAMMA != 1.0:
                    v = v ** (1.0 / GAMMA)  # GAMMA < 1 makes closing more aggressive
                
                # 2) Calculate delta around center, drive servos in opposite directions
                gain = CURL_GAIN_PER.get(finger_name, 300)
                delta = (v - 0.5) * 2.0 * gain
                
                # Apply movement smoothing for less aggressive response
                delta = delta * MOVEMENT_SMOOTHING
                
                # 3) Calculate base servo positions with direction mapping and center offset
                # Apply center offset to compensate for left bias when stretched
                center_offset = CENTER_OFFSET.get(finger_name, 0)
                # Calculate servo1 position with direction mapping and center offset
                servo1_pos = CENTER_POS + DIR[finger_name][0] * delta + center_offset
                servo2_pos = CENTER_POS + DIR[finger_name][1] * delta + center_offset
                
                # 4) NO LATERAL MOVEMENT - pure bending only for natural finger control
                # This removes the weaving behavior that was interfering with bending
                
                # 5) Clamp to finger-specific limits
                servo1_pos = max(OPEN_POS_PER[finger_name], min(CLOSE_POS_PER[finger_name], int(servo1_pos)))
                servo2_pos = max(OPEN_POS_PER[finger_name], min(CLOSE_POS_PER[finger_name], int(servo2_pos)))
                
                # Store positions for both motors
                joint_positions[motor1_id] = servo1_pos
                joint_positions[motor2_id] = servo2_pos
                
                print(f"  {finger_name}: curl={v:.3f}, servo1={servo1_pos}, servo2={servo2_pos} (no weaving)")
        
        print(f"DEBUG: Final joint_positions: {joint_positions}")
        return joint_positions
    
    def simple_inverse_kinematics(self, finger_data):
        """Simple inverse kinematics calculation with basic motor control."""
        joint_positions = []
        
        for finger_name in ['finger1', 'finger2', 'finger3', 'finger4']:
            if finger_name in finger_data:
                finger = finger_data[finger_name]
                angles = finger['angles']
                
                print(f"Processing {finger_name}: angles={angles}")
                
                if len(angles) >= 1:  # Thumb has 1 angle, others have 2
                    if finger_name == 'finger4':  # Thumb - only 1 angle
                        angle1 = np.clip(angles[0], 0, np.pi/2)
                        # For thumb, invert the angle to fix reversed movement
                        # When bending to palm (small angle), we want servo to move to closed position
                        # When stretching (large angle), we want servo to move to open position
                        servo1 = int((np.pi/2 - angle1) * 1023 / (np.pi/2))
                        servo2 = int((np.pi/2 - angle1) * 1023 / (np.pi/2))
                        print(f"  Thumb: angle1={angle1:.3f}, inverted servo1={servo1}, servo2={servo2}")
                    else:  # Other fingers - 2 angles
                        angle1 = np.clip(angles[0], 0, np.pi/2)
                        angle2 = np.clip(angles[1], 0, np.pi/2)
                        servo1 = int(angle1 * 1023 / (np.pi/2))
                        servo2 = int(angle2 * 1023 / (np.pi/2))
                        print(f"  {finger_name}: angle1={angle1:.3f}, angle2={angle2:.3f}, servo1={servo1}, servo2={servo2}")
                    
                    # Simple servo positioning without complex offsets/inversions for now
                    # This reverts to the previous "left stretched" version for fingers 1-3
                    print(f"  {finger_name}: Final servo1={servo1}, servo2={servo2}")
                    
                    joint_positions.extend([servo1, servo2])
                else:
                    # If not enough angles, use default positions
                    print(f"  {finger_name}: Not enough angles, using defaults")
                    joint_positions.extend([CENTER_POS, CENTER_POS])
            else:
                # Default positions for missing fingers
                print(f"  {finger_name}: Missing from finger_data, using defaults")
                joint_positions.extend([CENTER_POS, CENTER_POS])
        
        print(f"Final joint positions: {joint_positions}")
        return joint_positions
    
    def advanced_inverse_kinematics(self, finger_landmarks):
        """
        Advanced IK that calculates fingertip positions and solves for joint angles
        similar to the original Mujoco/Mink approach.
        """
        joint_positions = []
        
        # Define finger configurations (motor IDs for each finger)
        finger_configs = {
            'finger1': {'motor1': 1, 'motor2': 2},  # Index
            'finger2': {'motor1': 3, 'motor2': 4},  # Middle  
            'finger3': {'motor1': 5, 'motor2': 6},  # Ring
            'finger4': {'motor1': 7, 'motor2': 8}   # Thumb
        }
        
        # Get palm center for reference (use wrist as approximation)
        palm_center = None
        if 'finger1' in finger_landmarks and 'positions' in finger_landmarks['finger1']:
            # Use the MCP joint of finger1 as a reference for palm center
            palm_center = finger_landmarks['finger1']['positions'][0]
        
        # Process each finger
        for finger_name, finger_config in finger_configs.items():
            if finger_name not in finger_landmarks:
                continue
                
            finger_data = finger_landmarks[finger_name]
            
            # Check if we have the required data
            if 'positions' not in finger_data or 'angles' not in finger_data:
                print(f"  {finger_name}: Missing position or angle data")
                joint_positions.extend([512, 512])
                continue
            
            positions = finger_data['positions']
            angles = finger_data['angles']
            
            if finger_name == 'finger4':  # Thumb - special handling
                if len(positions) >= 3 and len(angles) >= 1:
                    # Get thumb landmarks
                    thumb_mcp = positions[0]      # Thumb MCP joint
                    thumb_ip = positions[1]      # Thumb IP joint  
                    thumb_tip = positions[2]     # Thumb tip
                    
                    # Calculate thumb spread angle (left/right movement)
                    # Use the first angle which represents MCP-IP bend
                    spread_angle = angles[0] if len(angles) > 0 else 0
                    
                    # Calculate thumb bend angle (toward/away from palm)
                    # Use the second angle if available, otherwise calculate from positions
                    if len(angles) > 1:
                        bend_angle = angles[1]
                    else:
                        # Calculate bend from thumb orientation relative to palm
                        if palm_center is not None:
                            # Calculate thumb bend based on its orientation relative to palm center
                            thumb_vector = np.array(thumb_tip) - np.array(thumb_mcp)
                            palm_to_thumb = np.array(thumb_mcp) - np.array(palm_center)
                            
                            # Project thumb vector onto palm plane
                            palm_normal = np.array([0, 0, 1])  # Assume palm faces camera
                            thumb_vector_proj = thumb_vector - np.dot(thumb_vector, palm_normal) * palm_normal
                            palm_to_thumb_proj = palm_to_thumb - np.dot(palm_to_thumb, palm_normal) * palm_normal
                            
                            if np.linalg.norm(thumb_vector_proj) > 0 and np.linalg.norm(palm_to_thumb_proj) > 0:
                                # Calculate angle between projected vectors
                                cos_bend = np.dot(thumb_vector_proj, palm_to_thumb_proj) / (np.linalg.norm(thumb_vector_proj) * np.linalg.norm(palm_to_thumb_proj))
                                cos_bend = np.clip(cos_bend, -1.0, 1.0)
                                bend_angle = np.arccos(cos_bend)
                            else:
                                bend_angle = np.pi/4  # Default to middle position
                        else:
                            # Fallback calculation without palm reference
                            thumb_vector = np.array(thumb_tip) - np.array(thumb_mcp)
                            palm_vector = np.array([0, 0, 1])  # Assume palm faces camera
                            cos_bend = np.dot(thumb_vector, palm_vector) / (np.linalg.norm(thumb_vector) * np.linalg.norm(palm_vector))
                            bend_angle = np.arccos(np.clip(cos_bend, -1, 1))
                        
                        # Use the calculated bend angle directly (reverted to previous logic)
                        bend_angle = angles[1] if len(angles) > 1 else np.pi/4
                    
                    # Map to servo positions with proper coordination
                    # Motor 1: Controls spread (left/right) - invert for natural movement
                    spread_pos = int((np.pi/2 - spread_angle) * 1023 / (np.pi/2))
                    spread_pos = np.clip(spread_pos, 0, 1023)
                    
                    # Motor 2: Controls bend (toward palm) - invert for natural movement
                    bend_pos = int((np.pi/2 - bend_angle) * 1023 / (np.pi/2))
                    bend_pos = np.clip(bend_pos, 0, 1023)
                    
                    # Apply coordination to prevent extreme movements
                    # When thumb is very bent, reduce spread movement to prevent over-extension
                    if bend_angle < np.pi/6:  # Very bent
                        spread_pos = int(spread_pos * 0.8)  # Reduce spread movement
                    
                    # When thumb is very spread, reduce bend movement to prevent over-extension
                    if spread_angle > np.pi/3:  # Very spread
                        bend_pos = int(bend_pos * 0.8)  # Reduce bend movement
                    
                    # Final clipping
                    spread_pos = np.clip(spread_pos, 0, 1023)
                    bend_pos = np.clip(bend_pos, 0, 1023)
                    
                    print(f"  Thumb: spread={spread_angle:.3f}, bend={bend_angle:.3f}")
                    print(f"  Thumb: spread_pos={spread_pos}, bend_pos={bend_pos}")
                    
                    joint_positions.extend([spread_pos, bend_pos])
                else:
                    print(f"  Thumb: Insufficient data, using center positions")
                    joint_positions.extend([512, 512])
                    
            else:  # Other fingers - use coordinated fingertip-driven IK
                if len(positions) >= 4 and len(angles) >= 2:
                    # Get finger landmarks
                    mcp = positions[0]    # MCP joint (base)
                    pip = positions[1]    # PIP joint (middle)
                    dip = positions[2]    # DIP joint (distal)
                    tip = positions[3]    # Fingertip
                    
                    # Use the calculated angles from extract_finger_data
                    angle1 = angles[0] if len(angles) > 0 else 0  # MCP-PIP angle
                    angle2 = angles[1] if len(angles) > 1 else 0  # PIP-DIP angle
                    
                    # Calculate finger orientation relative to palm center
                    if palm_center is not None:
                        # Vector from palm center to fingertip
                        palm_to_tip = np.array(tip) - np.array(palm_center)
                        palm_to_mcp = np.array(mcp) - np.array(palm_center)
                        
                        # Calculate finger's lateral deviation from palm center
                        # This helps prevent left/right weaving
                        finger_direction = np.array(tip) - np.array(mcp)
                        finger_direction = finger_direction / np.linalg.norm(finger_direction)
                        
                        # Project palm-to-tip vector onto finger direction
                        projection_length = np.dot(palm_to_tip, finger_direction)
                        projected_point = np.array(mcp) + projection_length * finger_direction
                        
                        # Calculate how much the finger deviates from the palm center line
                        deviation_vector = projected_point - np.array(palm_center)
                        deviation_magnitude = np.linalg.norm(deviation_vector)
                        
                        # Calculate finger length for normalization
                        finger_length = np.linalg.norm(np.array(tip) - np.array(mcp))
                        
                        # Normalize deviation (0 = perfectly aligned, 1 = maximum deviation)
                        normalized_deviation = min(deviation_magnitude / finger_length, 1.0)
                        
                        # Apply coordination to prevent weaving - REFINED LOGIC
                        # When finger deviates from center, INCREASE movement to counteract weaving
                        coordination_factor = 1.0 + (normalized_deviation * 0.4)  # Reduce from 60% to 40% for smoother movement
                        
                        # Base servo positions from angles
                        base_servo1 = int(angle1 * 1023 / (np.pi/2))
                        base_servo2 = int(angle2 * 1023 / (np.pi/2))
                        
                        # Apply coordination to both motors
                        servo1 = int(base_servo1 * coordination_factor)
                        servo2 = int(base_servo2 * coordination_factor)
                        
                        # Additional coordination: ensure smooth movement - REFINED LOGIC
                        # When stretching, coordinate both motors for natural finger curve
                        stretch_factor = np.linalg.norm(palm_to_tip) / finger_length
                        if stretch_factor > 1.1:  # Stretching
                            # Create natural finger curve by adjusting motor coordination
                            # Base joint (MCP) should move MORE than middle joint (PIP) when stretching to prevent weave
                            servo1 = int(servo1 * 1.25)  # Reduce from 1.35 to 1.25 for smoother stretching
                            servo2 = int(servo2 * 0.75)  # Increase from 0.65 to 0.75 for better balance
                        
                        # When bending, ensure both motors work together for natural palm closure
                        elif stretch_factor < 0.9:  # Bending
                            # Both motors should work together to close finger to palm
                            # CRITICAL: Prevent stretching right by ensuring both motors close to palm
                            # Progressive bending: more aggressive for deeper bends to prevent rightward stretch
                            if stretch_factor < 0.7:  # Deep bending (close to palm)
                                # Very aggressive factors to ensure palm closure and prevent rightward stretch
                                servo1 = int(servo1 * 1.5)  # Increase significantly for deep bend
                                servo2 = int(servo2 * 1.6)  # Increase significantly for deep bend
                                print(f"  {finger_name}: Deep bend mode - aggressive palm closure")
                            else:  # Moderate bending
                                # Moderate factors for smooth bending
                                servo1 = int(servo1 * 1.2)  # Moderate increase for smooth bending
                                servo2 = int(servo2 * 1.3)  # Moderate increase for smooth bending
                                print(f"  {finger_name}: Moderate bend mode - smooth movement")
                        
                        # Special case for ring finger: ensure it stretches straight instead of curving
                        if finger_name == "Ring":
                            if stretch_factor > 1.05:  # When stretching (even slightly)
                                # Balance both motors more evenly for straight stretch
                                servo1 = int(servo1 * 0.95)  # Slightly reduce base joint
                                servo2 = int(servo2 * 1.05)  # Slightly increase middle joint
                                print(f"  {finger_name}: Ring finger straight stretch adjustment applied")
                        
                        # Safety check: prevent fingers from stretching right when they should be bending
                        # If the finger is supposed to be bending but the stretch factor suggests stretching
                        if stretch_factor < 0.9 and stretch_factor > 0.5:  # Bending range
                            # Check if the calculated angles suggest the finger should continue bending
                            angle_sum = abs(angle1) + abs(angle2)
                            if angle_sum > np.pi/3:  # Significant bend angles
                                # Force the finger to continue bending instead of stretching
                                servo1 = int(servo1 * 1.3)  # Increase base joint for continued bending
                                servo2 = int(servo2 * 1.4)  # Increase middle joint for continued bending
                                print(f"  {finger_name}: Safety override - forcing continued bending")
                        
                        print(f"  {finger_name}: angle1={angle1:.3f}, angle2={angle2:.3f}")
                        print(f"  {finger_name}: deviation={normalized_deviation:.3f}, coordination={coordination_factor:.3f}")
                        print(f"  {finger_name}: stretch_factor={stretch_factor:.3f}")
                        print(f"  {finger_name}: base_servo1={base_servo1}, base_servo2={base_servo2}")
                        print(f"  {finger_name}: final_servo1={servo1}, final_servo2={servo2}")
                        
                    else:
                        # Fallback without palm reference - use basic coordination
                        servo1 = int(angle1 * 1023 / (np.pi/2))
                        servo2 = int(angle2 * 1023 / (np.pi/2))
                        
                        # Simple coordination to prevent extreme movements and maintain central alignment
                        servo1 = int(servo1 * 1.1)  # Reduce from 1.2 to 1.1 for smoother movement
                        servo2 = int(servo2 * 1.2)  # Reduce from 1.3 to 1.2 for smoother movement
                        
                        print(f"  {finger_name}: angle1={angle1:.3f}, angle2={angle2:.3f}")
                        print(f"  {finger_name}: servo1={servo1}, servo2={servo2} (basic coordination)")
                    
                    # Final clipping to valid servo range
                    servo1 = np.clip(servo1, 0, 1023)
                    servo2 = np.clip(servo2, 0, 1023)
                    
                    joint_positions.extend([servo1, servo2])
                else:
                    print(f"  {finger_name}: Insufficient data, using center positions")
                    joint_positions.extend([512, 512])
        
        return joint_positions
    
    def send_motor_commands(self, joint_positions):
        """Send motor commands to the robot hand."""
        if self.simulation_mode:
            print(f"Simulation: Motor positions would be: {joint_positions}")
            return
        
        if not hasattr(self, 'port_handler') or self.port_handler is None:
            print("No port handler available")
            return
        
        try:
            # Get motor IDs from config
            motor_ids = []
            finger_names = []
            for finger in self.motor_config["fingers"]:
                motor_ids.extend([finger["motor1"]["id"], finger["motor2"]["id"]])
                finger_names.extend([f"{finger['name']}_m1", f"{finger['name']}_m2"])
            
            print(f"DEBUG: send_motor_commands - motor_ids: {motor_ids}")
            print(f"DEBUG: send_motor_commands - joint_positions: {joint_positions}")
            
            # Handle both list and dictionary formats
            if isinstance(joint_positions, list):
                # Convert list to dictionary format for compatibility
                positions_dict = {}
                for i, motor_id in enumerate(motor_ids):
                    if i < len(joint_positions):
                        positions_dict[motor_id] = joint_positions[i]
                    else:
                        positions_dict[motor_id] = 512  # Default center position
                joint_positions = positions_dict
                print(f"DEBUG: Converted list to dict: {joint_positions}")
            
            # Debug output for thumb movement
            if len(joint_positions) >= 8:
                print(f"Motor commands - Index: [{joint_positions.get(1, 'N/A')}, {joint_positions.get(2, 'N/A')}], "
                      f"Middle: [{joint_positions.get(3, 'N/A')}, {joint_positions.get(4, 'N/A')}], "
                      f"Ring: [{joint_positions.get(5, 'N/A')}, {joint_positions.get(6, 'N/A')}], "
                      f"Thumb: [{joint_positions.get(7, 'N/A')}, {joint_positions.get(8, 'N/A')}]")
            
            # Send commands to motors using the new move_servo function
            result = self.send_scs0009_commands(motor_ids, joint_positions)
            print(f"DEBUG: send_scs0009_commands result: {result}")
            
            if result:
                print(f"DEBUG: Motor commands sent successfully")
            else:
                print(f"DEBUG: Motor commands failed")
                
        except Exception as e:
            print(f"Error sending motor commands: {e}")
            import traceback
            traceback.print_exc()
    
    def send_scs0009_commands(self, motor_ids, positions):
        """Send SCS0009 commands to multiple motors using scservo_sdk."""
        try:
            # Handle both list and dictionary formats
            if isinstance(positions, list):
                if len(motor_ids) != len(positions):
                    print("Mismatch between motor IDs and positions")
                    return False
                # Convert list to dictionary for easier access
                positions_dict = {}
                for i, motor_id in enumerate(motor_ids):
                    if i < len(positions):
                        positions_dict[motor_id] = positions[i]
                    else:
                        positions_dict[motor_id] = 512  # Default center position
                positions = positions_dict
            elif isinstance(positions, dict):
                # Check if all motor IDs have positions
                missing_motors = [mid for mid in motor_ids if mid not in positions]
                if missing_motors:
                    print(f"Missing positions for motors: {missing_motors}")
                    return False
            else:
                print(f"Invalid positions format: {type(positions)}")
                return False
            
            # Simple sequential motor commands (reverted from complex synchronization)
            success_count = 0
            for motor_id in motor_ids:
                position = positions.get(motor_id)
                if position is None:
                    print(f"DEBUG: No position found for motor {motor_id}")
                    continue
                print(f"DEBUG: Sending motor {motor_id} to position {position}")
                result = self.move_servo(motor_id, position, run_time=DEFAULT_TIME, run_speed=DEFAULT_SPEED)
                print(f"DEBUG: move_servo result for motor {motor_id}: {result}")
                if result:
                    success_count += 1
                time.sleep(0.01)  # Small delay between each motor command
            
            print(f"DEBUG: Successfully sent commands to {success_count}/{len(motor_ids)} motors")
            return success_count > 0
            
        except Exception as e:
            print(f"Error sending motor commands: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def enable_torque(self, motor_ids, enable):
        """Enable or disable torque for motors using scservo_sdk."""
        if not hasattr(self, 'port_handler') or self.port_handler is None:
            return
        
        for motor_id in motor_ids:
            try:
                self.set_torque(motor_id, enable)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error enabling torque for motor {motor_id}: {e}")
    
    def scan_motors(self):
        """Scan for available motor IDs."""
        if self.simulation_mode:
            print("Simulation mode: Cannot scan motors")
            return
        
        print("Scanning for available motors...")
        found_motors = []
        
        for motor_id in range(1, 9):  # Scan IDs 1-8
            try:
                model, res, err = self.packet_handler.ping(self.port_handler, motor_id)
                if res == COMM_SUCCESS and err == 0:
                    print(f"✓ Motor {motor_id} found (model={model})")
                    found_motors.append(motor_id)
                else:
                    print(f"✗ Motor {motor_id} not found")
            except Exception as e:
                print(f"✗ Motor {motor_id} error: {e}")
        
        if found_motors:
            print(f"Found {len(found_motors)} motors: {found_motors}")
        else:
            print("No motors found!")
        
        return found_motors
    
    def test_motors(self):
        """Test motor movement with simple commands."""
        if self.simulation_mode:
            print("Simulation mode: Cannot test motors")
            return
        
        print("Testing motor movement...")
        
        # Get motor IDs from config
        motor_ids = []
        for finger in self.motor_config["fingers"]:
            motor_ids.extend([finger["motor1"]["id"], finger["motor2"]["id"]])
        
        print(f"Testing motors: {motor_ids}")
        
        # Test each motor individually
        for motor_id in motor_ids:
            try:
                print(f"Testing motor {motor_id}...")
                
                # Move to open position
                print(f"  Moving motor {motor_id} to open position ({OPEN_POS})...")
                self.move_servo(motor_id, OPEN_POS, run_time=500, run_speed=400)
                time.sleep(1.0)
                
                # Move to close position
                print(f"  Moving motor {motor_id} to close position ({CLOSE_POS})...")
                self.move_servo(motor_id, CLOSE_POS, run_time=500, run_speed=400)
                time.sleep(1.0)
                
                # Return to center
                print(f"  Moving motor {motor_id} to center position ({CENTER_POS})...")
                self.move_servo(motor_id, CENTER_POS, run_time=500, run_speed=400)
                time.sleep(1.0)
                
                print(f"✓ Motor {motor_id} test completed")
                
            except Exception as e:
                print(f"✗ Motor {motor_id} test failed: {e}")
        
        print("Motor testing completed")
    
    def run_tracking_loop(self):
        """Main tracking loop."""
        print("Starting hand tracking...")
        print("Press 'q' to quit, 'a' to toggle Advanced IK, 'c' to toggle Curl-based IK")
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera")
            return
        
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("Error: Could not read frame")
                    break
                
                # Flip frame for selfie view
                frame = cv2.flip(frame, 1)
                
                # Process hand landmarks
                finger_data, hand_landmarks_norm = self.process_hand_landmarks(frame)
                
                if finger_data and hand_landmarks_norm:
                    # Debug finger coordination (optional - can be disabled for performance)
                    if self.use_advanced_ik and self.debug_enabled:
                        self.debug_finger_coordination(finger_data)
                    
                    # Calculate inverse kinematics using selected method
                    if self.use_curl_ik:
                        print("Using Curl-based IK (Reference file approach)...")
                        joint_positions = self.curl_based_inverse_kinematics(hand_landmarks_norm)
                    elif self.use_advanced_ik:
                        print("Using Advanced IK (Mujoco-style)...")
                        joint_positions = self.advanced_inverse_kinematics(finger_data)
                    else:
                        print("Using Simple IK...")
                        joint_positions = self.simple_inverse_kinematics(finger_data)
                    
                    # Send motor commands
                    if joint_positions:
                        self.send_motor_commands(joint_positions)
                    
                    # Draw hand landmarks
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks_norm,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style()
                    )
                    
                    # Draw finger information
                    self.draw_finger_info(frame, finger_data)
                
                # Display frame
                cv2.imshow('AmazingHand Tracking', frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('a'):
                    self.use_advanced_ik = not self.use_advanced_ik
                    self.use_curl_ik = False  # Turn off curl IK when switching
                    print(f"Advanced IK: {'ON' if self.use_advanced_ik else 'OFF'}")
                elif key == ord('c'):
                    self.use_curl_ik = not self.use_curl_ik
                    self.use_advanced_ik = False  # Turn off advanced IK when switching
                    print(f"Curl-based IK: {'ON' if self.use_curl_ik else 'OFF'}")
                
                # Control frame rate
                time.sleep(1.0 / self.fps)
                
        except KeyboardInterrupt:
            print("Tracking interrupted by user")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.cleanup()
    
    def draw_finger_info(self, frame, finger_data):
        """Draw finger information on the frame."""
        y_offset = 50
        for finger_name, finger in finger_data.items():
            if 'angles' in finger and finger['angles']:
                angle_text = f"{finger_name}: {[f'{a:.2f}' for a in finger['angles']]}"
                cv2.putText(frame, angle_text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_offset += 25
    
    def debug_finger_coordination(self, finger_data):
        """Debug method to analyze finger coordination and movement patterns."""
        if not finger_data:
            return
        
        print("\n=== Finger Coordination Debug ===")
        
        for finger_name in ['finger1', 'finger2', 'finger3', 'finger4']:
            if finger_name in finger_data:
                finger = finger_data[finger_name]
                positions = finger.get('positions', [])
                angles = finger.get('angles', [])
                
                print(f"\n{finger_name.upper()}:")
                print(f"  Positions: {len(positions)} landmarks")
                print(f"  Angles: {len(angles)} calculated angles")
                
                if len(positions) >= 2:
                    # Calculate finger length and orientation
                    if len(positions) >= 4:  # 3-joint finger
                        mcp = positions[0]
                        tip = positions[3]
                        finger_length = np.linalg.norm(np.array(tip) - np.array(mcp))
                        print(f"  Finger length: {finger_length:.3f}")
                        
                        # Calculate finger direction
                        finger_direction = np.array(tip) - np.array(mcp)
                        finger_direction = finger_direction / np.linalg.norm(finger_direction)
                        print(f"  Finger direction: [{finger_direction[0]:.3f}, {finger_direction[1]:.3f}, {finger_direction[2]:.3f}]")
                        
                        # Calculate deviation from center if we have palm reference
                        if 'finger1' in finger_data and len(finger_data['finger1']['positions']) > 0:
                            palm_center = finger_data['finger1']['positions'][0]  # Approximate palm center
                            palm_to_tip = np.array(tip) - np.array(palm_center)
                            palm_to_mcp = np.array(mcp) - np.array(palm_center)
                            
                            # Project onto finger direction
                            projection_length = np.dot(palm_to_tip, finger_direction)
                            projected_point = np.array(mcp) + projection_length * finger_direction
                            
                            # Calculate deviation
                            deviation_vector = projected_point - np.array(palm_center)
                            deviation_magnitude = np.linalg.norm(deviation_vector)
                            normalized_deviation = min(deviation_magnitude / finger_length, 1.0)
                            
                            print(f"  Deviation from center: {normalized_deviation:.3f}")
                            print(f"  Coordination factor: {1.0 - (normalized_deviation * 0.3):.3f}")
                    
                    elif len(positions) >= 3:  # 2-joint finger (thumb)
                        mcp = positions[0]
                        tip = positions[2]
                        finger_length = np.linalg.norm(np.array(tip) - np.array(mcp))
                        print(f"  Thumb length: {finger_length:.3f}")
                        
                        # Calculate thumb orientation
                        thumb_vector = np.array(tip) - np.array(mcp)
                        thumb_vector = thumb_vector / np.linalg.norm(thumb_vector)
                        print(f"  Thumb direction: [{thumb_vector[0]:.3f}, {thumb_vector[1]:.3f}, {thumb_vector[2]:.3f}]")
                
                if len(angles) > 0:
                    print(f"  Angle 1: {angles[0]:.3f} rad ({np.degrees(angles[0]):.1f}°)")
                    if len(angles) > 1:
                        print(f"  Angle 2: {angles[1]:.3f} rad ({np.degrees(angles[1]):.1f}°)")
        
        print("=" * 30 + "\n")

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'hands') and self.hands:
            try:
                self.hands.close()
            except ValueError:
                # MediaPipe already closed, ignore error
                pass
        
        if hasattr(self, 'port_handler') and self.port_handler:
            self.port_handler.closePort()
        
        print("Cleanup completed")

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="AmazingHand Tracker")
    parser.add_argument("--config", default="config/r_hand.json", help="Motor configuration file")
    parser.add_argument("--port", default="COM3", help="Serial port for motors")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Serial baudrate")
    parser.add_argument("--simulation", action="store_true", help="Run in simulation mode only")
    parser.add_argument("--hand-side", default="right", choices=["left", "right"], help="Hand side to track")
    parser.add_argument("--test-motors", action="store_true", help="Test motor movement only")
    parser.add_argument("--scan-motors", action="store_true", help="Scan for available motors only")
    parser.add_argument("--advanced-ik", action="store_true", help="Use advanced IK method by default")
    parser.add_argument("--curl-ik", action="store_true", help="Use curl-based IK method by default")
    parser.add_argument("--debug", action="store_true", help="Enable debug output for finger coordination")
    
    args = parser.parse_args()
    
    # Create controller
    controller = SimpleAmazingHandController(
        config_file=args.config,
        serial_port=args.port,
        baudrate=args.baudrate,
        simulation_mode=args.simulation,
        hand_side=args.hand_side
    )
    
    # Set debug flag
    controller.debug_enabled = args.debug
    
    try:
        if args.scan_motors:
            # Just scan for motors
            controller.scan_motors()
        elif args.test_motors:
            # Test motor movement
            controller.test_motors()
        else:
            # Set running flag and start tracking
            controller.running = True
            # Set initial IK method based on command line argument
            if args.curl_ik:
                print("Starting with Curl-based IK method")
                controller.use_curl_ik = True
            elif args.advanced_ik:
                print("Starting with Advanced IK method")
                controller.use_advanced_ik = True
            controller.run_tracking_loop()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
