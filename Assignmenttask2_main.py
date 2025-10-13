import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR3
from math import pi
import spatialgeometry as geometry
import numpy as np
from roboticstoolbox import DHLink, DHRobot
import keyboard
import time
import threading
import AuboI5
import myCobot
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

class PickPlaceRobot:
    def __init__(self):
        self.env = swift.Swift()
        self.env.launch()
        
        self.r1 = UR3()
        self.r1.base = self.r1.base * SE3(0,0,0)
        self.r1.add_to_env(self.env)
        
        self.r2 = AuboI5.AuboI5()
        self.r2.base = self.r2.base * SE3(0,-1,0)
        self.r2.add_to_env(self.env)

        shelf = geometry.Mesh('.venv/At2/shelf.stl', pose=SE3(-0.45, -1.6, 0.85) * SE3.Rx(pi), color=(0.5, 0.3, 0.1, 1), scale=[0.007, 0.002, 0.008])
        self.env.add(shelf)

        self.r3 = myCobot.myCobot()
        self.r3.base = self.r2.base * SE3(0,1.9,0)
        self.r3.add_to_env(self.env)

        self.boxes = []
        self.boxes_pos = []
        self.placement_poses = []
        
        # Emergency stop control
        self.paused = False
        self.stop_monitoring = False
        self.monitor_thread = None
        
        self.setup_zones()
        self.create_boxes()
        self.generate_placement_poses()
        self.start_emergency_stop_monitor()
    
    def start_emergency_stop_monitor(self):# Start background thread to monitor keyboard
        self.monitor_thread = threading.Thread(target=self._monitor_keys, daemon=True)
        self.monitor_thread.start()
    
    def _monitor_keys(self): # Background thread function
        print("\n=== Emergency Stop System Active ===")
        print("Press ESC to PAUSE")
        print("Press SPACE to RESUME")
        print("====================================\n")
        
        while not self.stop_monitoring:
            if keyboard.is_pressed('esc'):
                if not self.paused:
                    self.paused = True
                    print("\n*** EMERGENCY STOP - PAUSED ***")
                    print("Press SPACE to resume...")
                time.sleep(0.3)  # Debounce
            
            elif keyboard.is_pressed('space'):
                if self.paused:
                    self.paused = False
                    print("*** RESUMING ***\n")
                time.sleep(0.3)  # Debounce
            
            time.sleep(0.05)  # Check every 50ms
    
    def wait_if_paused(self): # keep paused if emergency stop is activated
        while self.paused:
            time.sleep(0.1)
    
    def setup_zones(self):
        # create first box placement zones
        red_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01], pose=SE3(0, 0.4, 0.001), color=(1, 0, 0, 0.5))
        self.env.add(red_zone)
        blue_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01], pose=SE3(0, -0.4, 0.001), color=(0, 0, 1, 0.5))
        self.env.add(blue_zone)
    
    def create_boxes(self):
        # Create coloured boxes to pick and place
        for i in range(6):
            initial_pose = SE3(0.4, -0.3 + i*0.12, 0.05)
            box = geometry.Cuboid(scale=[0.1, 0.1, 0.1], pose=initial_pose, color=(i % 2, 0.0, (i+1) % 2, 1))
            # first box is blue and alternates
            self.env.add(box)
            self.boxes.append(box)
            self.boxes_pos.append(initial_pose)

        # Create shelf boxes for aesthetics
        for i in range(3):
            shelf_box = geometry.Cuboid(scale=[0.1, 0.1, 0.1], pose= SE3(0.15 - i*0.15, -1.6 , 0.08), color=(0, 0.0, 1, 1))
            self.env.add(shelf_box)
    
    def generate_placement_poses(self):
        # generate placement poses
        for i in range(6):
            #alternate between red and blue zones
            if i % 2 == 0:
                place_pose = SE3(-0.2 + i*0.1, -0.4, 0.05)
            else:
                place_pose = SE3(-0.2 + (i-1)*0.1, 0.4, 0.05)
            
            self.placement_poses.append(place_pose)
    
    def UR3_pick_place(self):
        steps = 50
        q0 = (0, -pi/2, -pi/2, -pi/2, pi/2, 0)
        home_pose = self.r1.q

        for i, pose in enumerate(self.boxes_pos):
            self.wait_if_paused()  # Check for pause
            
            pickup_pose = pose * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
            air_pose = pose * SE3(-0.1, 0.0, 0.2) * SE3.Rx(pi)
            place_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
            
            # Solve IK
            q_pickup = self.r1.ikine_LM(pickup_pose, q0=q0, joint_limits=True).q
            q_air = self.r1.ikine_LM(air_pose, q0=q0, joint_limits=True).q
            q_place = self.r1.ikine_LM(place_pose, q0=q0, joint_limits=True).q
            
            # generate trajectory
            traj_pickup = rtb.jtraj(self.r1.q, q_pickup, steps).q
            traj_air = rtb.jtraj(q_pickup, q_air, steps).q
            traj_place = rtb.jtraj(q_air, q_place, steps).q

            for step_idx in range(steps):
                self.wait_if_paused()  # Check for pause at each step
                self.r1.q = traj_pickup[step_idx]
                self.env.step(0.02)
                time.sleep(speed)
            
            print("\nMoving to box", i+1, "position:", self.r1.fkine(self.r1.q).t)
            
            for step_idx in range(steps):
                self.wait_if_paused()
                self.r1.q = traj_air[step_idx]
                self.env.step(0.02)
                ee_pose = self.r1.fkine(self.r1.q)
                self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                time.sleep(speed)

            for step_idx in range(steps):
                self.wait_if_paused()
                self.r1.q = traj_place[step_idx]
                self.env.step(0.02)
                ee_pose = self.r1.fkine(self.r1.q)
                self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                time.sleep(speed)
            
            print("Placing box", i+1, "position:", self.r1.fkine(self.r1.q).t)
        
        # return to home
        traj_home = rtb.jtraj(self.r1.q, home_pose, steps).q

        for step_idx in range(steps):
            self.wait_if_paused()
            self.r1.q = traj_home[step_idx]
            self.env.step(0.02)
            time.sleep(speed)

    def second_placement_poses(self):
        # generate shelf placement poses
        poses = []
        for i in range(6):
            if i % 2 == 0: #place on shelf
                place_pose = SE3(0.15 - i*0.075, -1.6 , 0.45)
                poses.append(place_pose)

            else:  #place on conveyor
                place_pose = SE3(-0.05 + i*0.05, 1.3 , 0.01)  # Dummy pose, change as desired
                poses.append(place_pose)
        return poses
    
    def robots_pick_place(self):
        steps = 50
        q0_L = (0, -pi/2, pi/2, 0, pi/2, 0)
        q0_R = (0, -pi/2, -pi/2, 0, pi/2, 0)
        second_place_pose = self.second_placement_poses()  

        for i, pose in enumerate(self.placement_poses):
            self.wait_if_paused()  # Check for pause
            
            if i % 2 == 0: #shelf robot movement
                pickup_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
                air_pose = second_place_pose[i] * SE3(0, 0.1, 0.09) * SE3.Rx(pi)
                place_pose = second_place_pose[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)

                q_pickup = self.r2.ikine_LM(pickup_pose, q0=q0_R, joint_limits=True).q
                q_air = self.r2.ikine_LM(air_pose, q0=q0_L, joint_limits=True).q
                q_place = self.r2.ikine_LM(place_pose, q0=q0_L, joint_limits=True).q

                traj_pickup = rtb.jtraj(self.r2.q, q_pickup, steps).q
                traj_air = rtb.jtraj(q_pickup, q_air, steps).q
                traj_place = rtb.jtraj(q_air, q_place, steps).q

                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r2.q = traj_pickup[step_idx]
                    self.env.step(0.02)
                    time.sleep(speed)

                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r2.q = traj_air[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r2.fkine(self.r2.q)
                    self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                    time.sleep(speed)
                
                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r2.q = traj_place[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r2.fkine(self.r2.q)
                    self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                    time.sleep(speed)

            else: #conveyor robot movement
                pickup_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.1)* SE3.Rx(pi)
                air_pose = second_place_pose[i] * SE3(0, -0.1, 0.2) * SE3.Rx(pi)
                place_pose = second_place_pose[i] * SE3(0.0, 0.0, 0.1) * SE3.Rx(pi)

                q_pickup = self.r3.ikine_LM(pickup_pose, q0=q0_L, joint_limits=True, mask=[1,1,1,1,1,1]).q
                q_air = self.r3.ikine_LM(air_pose, q0=q0_R, joint_limits=True, mask=[1,1,1,1,1,1]).q
                q_place = self.r3.ikine_LM(place_pose, q0=q0_R, joint_limits=True, mask=[1,1,1,1,1,1]).q

                print("Placing box", i+1, "position:", self.r3.fkine(self.r3.q).t)

                traj_pickup = rtb.jtraj(self.r3.q, q_pickup, steps).q
                traj_air = rtb.jtraj(q_pickup, q_air, steps).q
                traj_place = rtb.jtraj(q_air, q_place, steps).q

                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r3.q = traj_pickup[step_idx]
                    self.env.step(0.02)
                    time.sleep(speed)

                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r3.q = traj_air[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r3.fkine(self.r3.q)
                    self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                    time.sleep(speed)
                
                for step_idx in range(steps):
                    self.wait_if_paused()
                    self.r3.q = traj_place[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r3.fkine(self.r3.q)
                    self.boxes[i].T = ee_pose * SE3(0.0, 0.0, 0.07)
                    time.sleep(speed)
    
    def shutdown(self):# Stop monitoring thread
        self.stop_monitoring = True
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
    
    def teach_aubo(self):# teach for aubo
        
        # Create figure for sliders
        fig, ax = plt.subplots(figsize=(4, 4))
        plt.subplots_adjust(left=0.1, bottom=0.4)
        ax.axis('off')
        
        # Initial joint angles
        joint_angles = list(self.r2.q)
        
        # Create sliders for each joint
        sliders = []
        for i in range(6):
            ax_slider = plt.axes([0.25, 0.05 + i*0.05, 0.65, 0.03])
            slider = Slider(
                ax_slider, 
                f'Joint {i+1}', 
                -2*pi, 
                2*pi, 
                valinit=joint_angles[i],
                valstep=0.01
            )
            sliders.append(slider)
        
        # Update function for sliders
        def update(val):
            for i, slider in enumerate(sliders):
                joint_angles[i] = slider.val
            self.r2.q = joint_angles
            self.env.step(0.01)
        
        # Connect sliders to update function
        for slider in sliders:
            slider.on_changed(update)
        
        # Add reset button
        resetax = plt.axes([0.8, 0.35, 0.1, 0.04])
        reset_button = Button(resetax, 'Reset')
        
        def reset(event):
            for slider in sliders:
                slider.reset()
        
        reset_button.on_clicked(reset)
        
        # Add close button
        closeax = plt.axes([0.65, 0.35, 0.1, 0.04])
        close_button = Button(closeax, 'Close')
        
        def close(event):
            plt.close(fig)
        
        close_button.on_clicked(close)
        
        # Add title and instructions
        ax.text(0.5, 0.9, 'Aubo I5 Teach')
        plt.show()

# Create and run the robot
speed = 0.005
robot = PickPlaceRobot()

print("\n=== Pick and Place System Ready ===")
print("Options:")
print("  Press '1' to open Aubo teach pendant")
print("  Press '2' to open myCobot teach pendant")
print("  Press 'SPACE' to start the operation")
print("\nDuring operation:")
print("  - Press 'ESC' to PAUSE")
print("  - Press 'SPACE' to RESUME")
print("===================================\n")

# Menu system
teach_mode = True
while teach_mode:
    if keyboard.is_pressed('1'):
        print("\n--- Opening Aubo Teach Pendant ---")
        robot.teach_aubo()
        print("Aubo teach pendant closed. Ready to continue.\n")
        time.sleep(0.5)  # Debounce
    
    elif keyboard.is_pressed('2'):
        print("\n--- Opening myCobot Teach Pendant ---")

        print("myCobot teach pendant closed. Ready to continue.\n")
        time.sleep(0.5)  # Debounce
    
    elif keyboard.is_pressed('space'):
        print("Starting pick and place operation...")
        time.sleep(0.3)  # Debounce
        teach_mode = False
    
    time.sleep(0.05)

try:
    robot.UR3_pick_place()
    robot.robots_pick_place()
    print("\n=== Operation Complete ===")
except KeyboardInterrupt:
    print("\n=== Operation Interrupted ===")
finally:
    robot.shutdown()
    time.sleep(2)
