import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR3
from math import pi
import spatialgeometry as geometry
import numpy as np
import time
import threading
import tkinter as tk
from tkinter import ttk
import AuboI5
import myCobot
from scipy import linalg
from roboticstoolbox import trapezoidal

class PickPlaceRobot:
    def __init__(self):
        # Flags for emergency stop behaviour
        self.estop_active = False
        self.resume_allowed = False
        self.estop_event = threading.Event()  # NEW: Event for thread synchronization
        self.estop_event.set()  # Initially set (not stopped)

        self.env = swift.Swift()
        self.env.launch()

        # Initialise robots
        self.r1 = UR3()
        self.r1.base = self.r1.base * SE3(0,0,0)
        self.r1.add_to_env(self.env)

        self.r2 = AuboI5.AuboI5()
        self.r2.base = self.r2.base * SE3(0,-1,0)
        self.r2.add_to_env(self.env)

        self.r3 = myCobot.myCobot()
        self.r3.base = self.r3.base * SE3(0,0.8,0)
        self.r3.add_to_env(self.env)

        self.boxes = []
        self.boxes_pos = []
        self.placement_poses = []

        self.setup_zones()
        self.create_boxes()
        self.generate_placement_poses()
        
        # Create GUI control panel
        self.create_control_panel()

    # ----------------------------------------------------------
    # --- GUI CONTROL PANEL
    # ----------------------------------------------------------
    def create_control_panel(self):
        """Create a GUI window with E-Stop and control buttons."""
        self.control_window = tk.Tk()
        self.control_window.title("Robot Control Panel")
        self.control_window.geometry("400x300")
        self.control_window.configure(bg='#2b2b2b')
        
        # Status label
        self.status_label = tk.Label(
            self.control_window,
            text="System Status: RUNNING",
            font=("Arial", 16, "bold"),
            bg='#2b2b2b',
            fg='#00ff00',
            pady=20
        )
        self.status_label.pack()
        
        # E-Stop Button (Large and Red)
        self.estop_button = tk.Button(
            self.control_window,
            text="EMERGENCY STOP",
            command=self.trigger_estop,
            font=("Arial", 18, "bold"),
            bg='#ff0000',
            fg='black',
            activebackground='#cc0000',
            height=3,
            width=20,
            relief=tk.RAISED,
            bd=5
        )
        self.estop_button.pack(pady=10)
        
        # Reset Button
        self.reset_button = tk.Button(
            self.control_window,
            text="RESET",
            command=self.reset_system,
            font=("Arial", 14, "bold"),
            bg='#ffa500',
            fg='black',
            activebackground='#cc8400',
            height=2,
            width=20,
            state=tk.DISABLED
        )
        self.reset_button.pack(pady=5)
        
        # Resume Button
        self.resume_button = tk.Button(
            self.control_window,
            text="RESUME",
            command=self.resume_system,
            font=("Arial", 14, "bold"),
            bg='#00aa00',
            fg='black',
            activebackground='#008800',
            height=2,
            width=20,
            state=tk.DISABLED
        )
        self.resume_button.pack(pady=5)
        
        # Info label
        info_label = tk.Label(
            self.control_window,
            text="Press E-Stop to halt all robot motion\nThen Reset → Resume to continue",
            font=("Arial", 10),
            bg='#2b2b2b',
            fg='#aaaaaa',
            pady=10
        )
        info_label.pack()
        
        # Start GUI update loop
        self.update_gui()
    
    def trigger_estop(self):
        """Trigger emergency stop."""
        if not self.estop_active:
            self.estop_active = True
            self.resume_allowed = False
            self.estop_event.clear()  # NEW: Clear the event (blocks waiting threads)
            print("EMERGENCY STOP triggered!")
            self.status_label.config(text="Status: E-STOP ACTIVE", fg='#ff0000')
            self.estop_button.config(state=tk.DISABLED)
            self.reset_button.config(state=tk.NORMAL)
            self.resume_button.config(state=tk.DISABLED)
    
    def reset_system(self):
        """Reset the system after E-Stop."""
        if self.estop_active:
            self.resume_allowed = True
            print("System reset — ready to resume.")
            self.status_label.config(text="Status: READY TO RESUME", fg='#ffa500')
            self.reset_button.config(state=tk.DISABLED)
            self.resume_button.config(state=tk.NORMAL)
    
    def resume_system(self):
        """Resume operation after reset."""
        if self.estop_active and self.resume_allowed:
            self.estop_active = False
            self.resume_allowed = False
            self.estop_event.set()  # NEW: Set the event (unblocks waiting threads)
            print("System resumed.")
            self.status_label.config(text="Status: RUNNING", fg='#00ff00')
            self.estop_button.config(state=tk.NORMAL)
            self.reset_button.config(state=tk.DISABLED)
            self.resume_button.config(state=tk.DISABLED)
    
    def update_gui(self):
        """Update GUI elements periodically."""
        try:
            self.control_window.update()
            self.control_window.after(50, self.update_gui)
        except tk.TclError:
            pass  # Window was closed

    def check_estop(self):
        """Pause motion safely if E-Stop is active - NO BUSY LOOP."""
        if not self.estop_event.is_set():
            print("Motion paused due to E-Stop. Awaiting reset and resume...")
            self.estop_event.wait()  # NEW: Blocks efficiently until event is set
            print("Motion resuming...")

    # ----------------------------------------------------------
    # --- ENVIRONMENT SETUP
    # ----------------------------------------------------------
    def setup_zones(self):
        red_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01],
                                   pose=SE3(0, 0.4, 0.001),
                                   color=(1, 0, 0, 0.5))
        blue_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01],
                                    pose=SE3(0, -0.4, 0.001),
                                    color=(0, 0, 1, 0.5))
        self.env.add(red_zone)
        self.env.add(blue_zone)

    def create_boxes(self):
        for i in range(6):
            initial_pose = SE3(0.4, -0.3 + i*0.12, 0.05)
            box = geometry.Cuboid(scale=[0.1, 0.1, 0.1],
                                  pose=initial_pose,
                                  color=(i % 2, 0.0, (i+1) % 2, 1))
            self.env.add(box)
            self.boxes.append(box)
            self.boxes_pos.append(initial_pose)

        for i in range(3):
            shelf_box = geometry.Cuboid(scale=[0.1, 0.1, 0.1],
                                        pose=SE3(0.15 - i*0.15, -1.6 , 0.08),
                                        color=(0, 0.0, 1, 1))
            self.env.add(shelf_box)

    def generate_placement_poses(self):
        for i in range(6):
            if i % 2 == 0:  
                place_pose = SE3(-0.2 + i*0.1, -0.4, 0.05)
            else:
                place_pose = SE3(-0.2 + (i-1)*0.1, 0.4, 0.05)
            self.placement_poses.append(place_pose)

    # ----------------------------------------------------------
    # --- ROBOT 1: UR3 PICK & PLACE
    # ----------------------------------------------------------
    def UR3_pick_place(self):
        steps = 50
        q0 = (0, -pi/2, -pi/2, -pi/2, pi/2, 0)
        home_pose = self.r1.q

        for i, pose in enumerate(self.boxes_pos):
            self.check_estop()
            pickup_pose = pose * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
            air_pose = pose * SE3(-0.1, 0.0, 0.2) * SE3.Rx(pi)
            place_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)

            q_pickup = self.r1.ikine_LM(pickup_pose, q0=q0, joint_limits=True).q
            q_air = self.r1.ikine_LM(air_pose, q0=q0, joint_limits=True).q
            q_place = self.r1.ikine_LM(place_pose, q0=q0, joint_limits=True).q

            traj_pickup = rtb.jtraj(self.r1.q, q_pickup, steps).q
            traj_air = rtb.jtraj(q_pickup, q_air, steps).q
            traj_place = rtb.jtraj(q_air, q_place, steps).q

            for step_idx in range(steps):
                self.check_estop()
                self.r1.q = traj_pickup[step_idx]
                self.env.step(0.02)
                time.sleep(speed)

            for step_idx in range(steps):
                self.check_estop()
                self.r1.q = traj_air[step_idx]
                self.env.step(0.02)
                ee_pose = self.r1.fkine(self.r1.q)
                self.boxes[i].T = ee_pose * SE3(0,0,0.07)
                time.sleep(speed)

            for step_idx in range(steps):
                self.check_estop()
                self.r1.q = traj_place[step_idx]
                self.env.step(0.02)
                ee_pose = self.r1.fkine(self.r1.q)
                self.boxes[i].T = ee_pose * SE3(0,0,0.07)
                time.sleep(speed)

        traj_home = rtb.jtraj(self.r1.q, home_pose, steps).q
        for step_idx in range(steps):
            self.check_estop()
            self.r1.q = traj_home[step_idx]
            self.env.step(0.02)
            time.sleep(speed)

    # ----------------------------------------------------------
    # --- ROBOT 2: AUBO PICK & PLACE
    # ----------------------------------------------------------
    def second_placement_poses(self):
        shelf_poses = []
        for i in range(6):
            if i % 2 == 0:
                place_pose = SE3(0.15 - i*0.075, -1.6 , 0.45)
            else:
                place_pose = SE3(0, 0 , i)
            shelf_poses.append(place_pose)
        return shelf_poses

    def aubo_pick_place(self):
        steps = 50
        q0 = (0, -pi/2, pi/2, 0, pi/2, 0)
        shelf_pose = self.second_placement_poses()

        for i, pose in enumerate(self.placement_poses):
            self.check_estop()
            if i % 2 == 0:
                pickup_pose = self.placement_poses[i] * SE3(0,0,0.07) * SE3.Rx(pi)
                air_pose = shelf_pose[i] * SE3(0,0.1,0.09) * SE3.Rx(pi)
                place_pose = shelf_pose[i] * SE3(0,0,0.07) * SE3.Rx(pi)

                q_pickup = self.r2.ikine_LM(pickup_pose, q0=q0, joint_limits=True).q
                q_air = self.r2.ikine_LM(air_pose, q0=q0, joint_limits=True).q
                q_place = self.r2.ikine_LM(place_pose, q0=q0, joint_limits=True).q

                traj_pickup = rtb.jtraj(self.r2.q, q_pickup, steps).q
                traj_air = rtb.jtraj(q_pickup, q_air, steps).q
                traj_place = rtb.jtraj(q_air, q_place, steps).q

                for step_idx in range(steps):
                    self.check_estop()
                    self.r2.q = traj_pickup[step_idx]
                    self.env.step(0.02)
                    time.sleep(speed)

                for step_idx in range(steps):
                    self.check_estop()
                    self.r2.q = traj_air[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r2.fkine(self.r2.q)
                    self.boxes[i].T = ee_pose * SE3(0,0,0.07)
                    time.sleep(speed)

                for step_idx in range(steps):
                    self.check_estop()
                    self.r2.q = traj_place[step_idx]
                    self.env.step(0.02)
                    ee_pose = self.r2.fkine(self.r2.q)
                    self.boxes[i].T = ee_pose * SE3(0,0,0.07)
                    time.sleep(speed)
            else:
                print("conveyor")

    # ----------------------------------------------------------
    # --- RMRC HELPER FUNCTION (Lab 6 Method)
    # ----------------------------------------------------------
    def rmrc_motion(self, robot, T_start, T_end, steps=50, delta_t=0.05):
        """
        Perform RMRC motion from T_start to T_end using Lab 6 velocity control method
        
        This implements the same approach as Lab 6 Question 3:
        1. Create Cartesian trajectory using trapezoidal velocity profile
        2. Solve IK for initial position
        3. Loop through trajectory calculating velocities
        4. Use Jacobian inverse to get joint velocities
        5. Integrate to get joint positions
        
        Args:
            robot: Robot object
            T_start: Starting SE3 transform
            T_end: Ending SE3 transform
            steps: Number of discrete time steps
            delta_t: Time step duration
            
        Returns:
            q_matrix: Joint angle trajectory (steps x n_joints)
            success: Boolean
        """
        
        # Step 1: Extract start and end positions (like Lab 6 x1 and x2)
        x_start = T_start.t  # Translation vector [x, y, z]
        x_end = T_end.t

        R_constant = T_end.R  # Keep rotation constant for simplicity
        
        # Step 2: Create trajectory using trapezoidal velocity profile (Lab 6 line 241)
        s = trapezoidal(0, 1, steps).q
        
        # Step 3: Generate Cartesian waypoints (Lab 6 lines 244-246)
        x_trajectory = np.zeros([3, steps])
        for i in range(steps):
            x_trajectory[:, i] = x_start * (1 - s[i]) + s[i] * x_end
        
        # Step 4: Initialize joint trajectory matrix (Lab 6 line 248)
        q_matrix = np.zeros([steps, robot.n])
        
        # Step 5: Solve IK for initial position (Lab 6 line 250)
        q0 = robot.q if robot.q is not None else [0, -pi/2, pi/2, 0, pi/2, 0]
        result = robot.ikine_LM(T_start, q0=q0, joint_limits=True)
        
        if not result.success:
            print(f"  Warning: Initial IK failed for RMRC")
            return None, False
        
        q_matrix[0, :] = result.q
        
        # Step 6: RMRC Loop (Lab 6 lines 252-258)
        for i in range(steps - 1):
            # Calculate Cartesian velocity (Lab 6 line 253)
            xdot = (x_trajectory[:, i+1] - x_trajectory[:, i]) / delta_t
            
            # Get Jacobian at current state (Lab 6 line 254)
            J = robot.jacob0(q_matrix[i, :])
            
            # Use only position rows (x, y, z) - Lab 6 line 255
            J_pos = J[:3, :]

            manipulability = np.sqrt(np.abs(linalg.det(J_pos @ J_pos.T)))

            if manipulability < 0.01:
                lambda_sq = 0.01
                q_dot = J_pos.T @ linalg.inv(J_pos @ J_pos.T + lambda_sq * np.eye(3)) @ xdot
            else:
                # Solve for joint velocities: q_dot = J^-1 * xdot (Lab 6 line 256)
                q_dot = linalg.pinv(J_pos) @ xdot
            
            if not np.all(np.isfinite(q_dot)):
                return None, False
            
            # Update joint angles using Euler integration (Lab 6 line 257)
            q_matrix[i+1, :] = q_matrix[i, :] + delta_t * q_dot
            
#            max_join_step = 0.1
#            joint_change = np.abs(q_matrix[i+1, :] - q_matrix[i, :])

#            if np.any(joint_change > max_join_step):
#                return None, False


            T_desired = SE3.Rt(R_constant, x_trajectory[:, i+1])
            result = robot.ikine_LM(T_desired, q0=q_matrix[i+1, :], joint_limits=True)

            if result.success:
                q_matrix[i+1, :] = result.q

        
        return q_matrix, True

    # ----------------------------------------------------------
    # --- ROBOT 3: myCobot PICK & PLACE WITH RMRC
    # ----------------------------------------------------------
    def myCobot_pick_place(self):
        """
        myCobot pick and place using RMRC with proper waypoints
        """
        steps = 100
        delta_t = 0.03
        
        base_x = -0.4
        base_y = 1.0
        base_z = 0.15
        spacing = 0.135
        
        for i, pose in enumerate(self.placement_poses):
            self.check_estop()
            
            if i % 2 == 1:
                print(f"\n--- Box {i}: Multi-waypoint RMRC ---")
                
                # CORRECT WAYPOINTS
                waypoints = [
                    # 0: Current position
                    self.r3.fkine(self.r3.q),
                    
                    # 1: Safe height above pickup (approach from above)
                    self.placement_poses[i] * SE3(0, 0, 0.4) * SE3.Ry(pi) * SE3.Rz(pi),
                    
                    # 2: Lower to pickup height
                    self.placement_poses[i] * SE3(0, 0, 0.07) * SE3.Ry(pi) * SE3.Rz(pi),
                    
                    # 3: Lift up with box
                    self.placement_poses[i] * SE3(0, 0, 0.4) * SE3.Ry(pi) * SE3.Rz(pi),
                    
                    # 4: High above place position (safe travel height)
                    SE3(base_x + i * spacing, base_y, base_z + 0.4) * SE3.Ry(pi),
                    
                    # 5: Lower to place position
                    SE3(base_x + i * spacing, base_y, base_z) * SE3.Ry(pi)
                ]
                    
                # Execute motion through all waypoints
                for wp_idx in range(len(waypoints) - 1):
                    print(f"  Moving: Waypoint {wp_idx} → {wp_idx + 1}")
                    
                    q_traj, success = self.rmrc_motion(
                        self.r3, 
                        waypoints[wp_idx], 
                        waypoints[wp_idx + 1], 
                        steps, 
                        delta_t
                    )
                    
                    if not success:
                        print(f"  ERROR at waypoint {wp_idx + 1}")
                        break
                    
                    # Execute trajectory
                    for step_idx in range(steps):
                        self.check_estop()
#                        self.check_obstacle(self.r3, "myCobot")  # ADD THIS
                        self.r3.q = q_traj[step_idx]
                        self.env.step(0.01)
                        
                        # Attach box AFTER pickup (waypoint 2 completed)
                        if wp_idx >= 2:  # FIXED: Was >= 3
                            ee_pose = self.r3.fkine(self.r3.q)
                            self.boxes[i].T = ee_pose * SE3(0, 0, 0.02)
                        
                        time.sleep(0.005)
                
                print(f"  ✓ Box {i} complete")
            else:
                print(f"Box {i}: Skipped (blue zone)")

# ----------------------------------------------------------
# --- MAIN EXECUTION
# ----------------------------------------------------------
if __name__ == "__main__":
    speed = 0.01
    robot = PickPlaceRobot()

    # Run the simulation tasks in a separate thread
    def run_simulation():
        robot.UR3_pick_place()
        robot.aubo_pick_place()
        robot.myCobot_pick_place()
        robot.env.hold()
    
    sim_thread = threading.Thread(target=run_simulation, daemon=True)
    sim_thread.start()
    
    # Keep GUI running (this is the main thread)
    try:
        robot.control_window.mainloop()
    except KeyboardInterrupt:
        print("\nShutting down...")