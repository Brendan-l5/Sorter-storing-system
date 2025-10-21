import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support import UR3, RectangularPrism, line_plane_intersection
from math import pi
import spatialgeometry as geometry
import numpy as np
import keyboard
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import AuboI5
import myCobot
from itertools import combinations
from spatialgeometry import Sphere
from spatialmath.base import transl
import serial # For Arduino communication

#================================================ SETUP PICK AND PLACE ROBOT ENVIRONMENT ==================================================
class PickPlaceRobot:
    def __init__(self):
        self.env = swift.Swift()
        self.env.launch()

        try: #check if arduino is connected or not
            self.arduino = serial.Serial('COM10', 9600, timeout=1)  # COM port depends on which port the arduino is plugged into
            time.sleep(2)  # Allow time for Arduino reset
            print("Connected to Arduino")
        except Exception as e:
            self.arduino = None
            print("Could not connect to Arduino: {e}") # CLOSE ARDUINO IDE BEFORE STARTOING PROGRAM
        
        self.r1 = UR3()
        self.r1.base = self.r1.base * SE3(0,0,0)
        self.r1.add_to_env(self.env)
        
        self.r2 = AuboI5.AuboI5()
        self.r2.base = self.r2.base * SE3(0,-1,0)
        self.r2.add_to_env(self.env)

        self.r3 = myCobot.myCobot()
        self.r3.base = self.r2.base * SE3(0,1.9,0)
        self.r3.add_to_env(self.env)

        shelf = geometry.Mesh('.venv/At2/shelf.stl', pose=SE3(-0.45, -1.6, 0.48) * SE3.Rx(pi), color=(0.5, 0.3, 0.1, 1), scale=[0.007, 0.002, 0.008])
        self.env.add(shelf)

        fence_data = [(SE3(-0.7, 1.9, 0), [0.02, 0.03, 0.03]),(SE3(-0.3, -1.5, 0), [0.03, 0.03, 0.03]),(SE3(0.9, -0.1, 0) * SE3.Rz(pi / 2), [0.035, 0.03, 0.03]),(SE3(-2.1, -0.5, 0) * SE3.Rz(pi / 2), [0.025, 0.03, 0.03])]

        for pose, scale in fence_data:
            fence = geometry.Mesh('.venv/At2/fence.stl', pose=pose, color=(0.5, 0.5, 0.5, 1), scale=scale)
            self.env.add(fence)

        table = geometry.Mesh('.venv/At2/Table.stl', pose=SE3(-1, 2, 0.5) * SE3.Rx(pi), color=(0.5, 0.3, 0.1, 1), scale=[0.015, 0.02, 0.02])
        self.env.add(table)

        fire_extinguisher = geometry.Mesh('.venv/At2/Fire_extinguisher.stl', pose=SE3(-5, -5, 0), color=(0.5, 0, 0, 1), scale=[0.06, 0.06, 0.06])
        self.env.add(fire_extinguisher)

        sign_post = geometry.Cylinder(radius=0.02,length=1.2, pose=SE3(0.35, 1.68, 0.6), color=(0.2, 0.2, 0.2, 1))
        self.env.add(sign_post)

        warning_sign = geometry.Mesh('.venv/At2/Warning_sign.stl', pose=SE3(0.6, 1.7, 1) * SE3.Rx(pi / 2) * SE3.Ry(pi), color=(0.5, 0.5, 0, 1), scale=[0.003, 0.003, 0.003])
        self.env.add(warning_sign)

        button = geometry.Cylinder(radius=0.05, length=0.04, pose=SE3(-0.5, 2.4, 0.52), color=(1.0, 0, 0, 1))
        self.env.add(button)

        button_casing = geometry.Cuboid(scale=[0.12, 0.12, 0.04], pose=SE3(-0.5, 2.4, 0.51), color=(0.1, 0.1, 0.1, 1))
        self.env.add(button_casing)

        person = geometry.Mesh('.venv/At2/person.stl', pose=SE3(-0.4, 2.9, 0), color=(0.7, 0.5, 0.5, 1), scale=[0.08, 0.08, 0.08])
        self.env.add(person)

        truck = geometry.Mesh('.venv/At2/Truck.stl', pose=SE3(-3, 1.4, 0) * SE3.Rz(pi), color=(0.4, 0.4, 0.4, 1), scale=[0.04, 0.02, 0.04])
        self.env.add(truck)

        conveyor = geometry.Mesh('.venv/At2/conveyor.stl', pose=SE3(-0.8, 1.3, 0), color=(0.3, 0.3, 0.3, 1), scale=[0.008, 0.0025, 0.005])
        self.env.add(conveyor)
        
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
        self.setup_collision_detection()  # Initialize collision detection

        #emergency stop initialise
        self.paused = False
        self.ready_to_resume = False
        self.stop_monitoring = False
        self.monitor_thread = None

        self.start_emergency_stop_monitor()
    
    def setup_collision_detection(self):#setup collision detection obstacles
        self.collision_detector = CollisionDetector(self.env)
        
        # Add shelf as obstacle (main collision concern)
        shelf_lwh = [0.4, 0.3, 0.5]  # length, width, height
        shelf_pose = SE3(-0.45, -1.6, 0.48)

        floor_lwh = [3.0, 3.0, 0.01] # add floor as obstacle
        floor_pose = SE3(0, 0, -0.005)
        self.collision_detector.add_obstacle(shelf_lwh, shelf_pose)
    
    def setup_zones(self):
        red_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01], pose=SE3(0, 0.4, 0.001), color=(1, 0, 0, 0.5))
        self.env.add(red_zone)
        blue_zone = geometry.Cuboid(scale=[0.7, 0.3, 0.01], pose=SE3(0, -0.4, 0.001), color=(0, 0, 1, 0.5))
        self.env.add(blue_zone)
    
    def create_boxes(self):
        for i in range(6):
            initial_pose = SE3(0.4, -0.3 + i*0.12, 0.05)
            box = geometry.Cuboid(scale=[0.1, 0.1, 0.1], pose=initial_pose, color=(i % 2, 0.0, (i+1) % 2, 1))
            self.env.add(box)
            self.boxes.append(box)
            self.boxes_pos.append(initial_pose)

        for i in range(3):
            shelf_box = geometry.Cuboid(scale=[0.1, 0.1, 0.1], pose= SE3(0.15 - i*0.15, -1.6 , 0.1), color=(0, 0.0, 1, 1))
            self.env.add(shelf_box)

#================================================= EMERGENCY STOP MONITORING ============================================================
    
    def start_emergency_stop_monitor(self):
        self.monitor_thread = threading.Thread(target=self._monitor_arduino, daemon=True)
        self.monitor_thread.start()
    
    def _monitor_arduino(self):
        print("\nEmergency Stop System Active")
        
        while not self.stop_monitoring:
            if self.arduino and self.arduino.in_waiting > 0:
                msg = self.arduino.readline().decode('utf-8').strip()
                if msg == "PAUSE" and not self.paused:
                    self.paused = True
                    self.ready_to_resume = False
                    print("\n PAUSED")
                elif msg == "READY" and self.paused and self.ready_to_resume == False:
                    self.ready_to_resume = True
                    print("\n READY")
                elif msg == "RESUME" and self.paused and self.ready_to_resume:
                        self.paused = False
                        self.ready_to_resume = False
                        print("\n RESUMED")
            time.sleep(0.05)
    
    def wait_if_paused(self):
        while self.paused:
            time.sleep(0.1)

#================================================= PICK AND PLACE OPERATIONS =============================================================

    def move_robot_safe(self, robot, q_start, q_goal, box_idx=None):#move robot with collision detection
        success, q_matrix = self.collision_detector.find_collision_free_path(
            robot, q_start, q_goal, max_attempts=30
        )
        
        if not success:
            print("collision detected")
        
        for q in q_matrix:
            self.wait_if_paused()
            robot.q = q
            
            if box_idx is not None and box_idx < len(self.boxes):
                ee_pose = robot.fkine(robot.q)
                self.boxes[box_idx].T = ee_pose * SE3(0.0, 0.0, 0.07)
            
            self.env.step(0.02)
            time.sleep(speed)
            
        return success
    
    def generate_placement_poses(self):
        for i in range(6):
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
            self.wait_if_paused()
            
            pickup_pose = pose * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
            air_pose = pose * SE3(-0.1, 0.0, 0.2) * SE3.Rx(pi)
            place_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
            
            q_pickup = self.r1.ikine_LM(pickup_pose, q0=q0, joint_limits=True).q
            q_air = self.r1.ikine_LM(air_pose, q0=q0, joint_limits=True).q
            q_place = self.r1.ikine_LM(place_pose, q0=q0, joint_limits=True).q
            # Move to pickup
            self.move_robot_safe(self.r1, self.r1.q, q_pickup)
            
            # Lift box
            self.move_robot_safe(self.r1, q_pickup, q_air, box_idx=i)
            
            # Place box
            self.move_robot_safe(self.r1, q_air, q_place, box_idx=i)
        
        # Return home
        self.move_robot_safe(self.r1, self.r1.q, home_pose)

    def second_placement_poses(self):
        poses = []
        for i in range(6):
            if i % 2 == 0:
                place_pose = SE3(0.15 - i*0.075, -1.6 , 0.5)
                poses.append(place_pose)
            else:
                place_pose = SE3(0.2 + -i*0.07, 1.3 , 0.4)
                poses.append(place_pose)
        return poses
    
    def robots_pick_place(self):
        
        steps = 50
        q0_L = (0, -pi/2, pi/2, 0, pi/2, 0)
        q0_R = (0, -pi/2, -pi/2, 0, pi/2, 0)
        second_place_pose = self.second_placement_poses()

        for i, pose in enumerate(self.placement_poses):
            self.wait_if_paused()
            
            if i % 2 == 0: #aubo movement
                pickup_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)
                air_pose = second_place_pose[i] * SE3(0, 0.1, 0.09) * SE3.Rx(pi)
                place_pose = second_place_pose[i] * SE3(0.0, 0.0, 0.07) * SE3.Rx(pi)

                q_pickup = self.r2.ikine_LM(pickup_pose, q0=q0_R, joint_limits=True).q
                q_air = self.r2.ikine_LM(air_pose, q0=q0_L, joint_limits=True).q
                q_place = self.r2.ikine_LM(place_pose, q0=q0_L, joint_limits=True).q

                self.move_robot_safe(self.r2, self.r2.q, q_pickup)
                
                self.move_robot_safe(self.r2, q_pickup, q_air, box_idx=i)
                
                self.move_robot_safe(self.r2, q_air, q_place, box_idx=i)

            else:  # myCobot movement
                
                pickup_pose = self.placement_poses[i] * SE3(0.0, 0.0, 0.1)* SE3.Rx(pi)
                air_pose = second_place_pose[i] * SE3(0, -0.1, 0.2) * SE3.Rx(pi)
                place_pose = second_place_pose[i] * SE3(0.0, 0.0, 0.1) * SE3.Rx(pi)

                q_pickup = self.r3.ikine_LM(pickup_pose, q0=q0_L, joint_limits=True, mask=[1,1,1,1,1,1]).q
                q_air = self.r3.ikine_LM(air_pose, q0=q0_R, joint_limits=True, mask=[1,1,1,1,1,1]).q
                q_place = self.r3.ikine_LM(place_pose, q0=q0_R, joint_limits=True, mask=[1,1,1,1,1,1]).q

                self.move_robot_safe(self.r3, self.r3.q, q_pickup)
                
                self.move_robot_safe(self.r3, q_pickup, q_air, box_idx=i)
                
                self.move_robot_safe(self.r3, q_air, q_place, box_idx=i)
    
    def shutdown(self):
        self.stop_monitoring = True
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        if self.arduino:
            self.arduino.close()

# ====================================================== TEACH PENDANT FUNCTION ==========================================================
    
    def create_teach_pendant(self, robot, robot_name):
        plt.ioff()
        fig = plt.figure(figsize=(5, 4))
        updating = {'flag': False}
        
        # Joint Control 
        ax_joint = fig.add_subplot(121)
        ax_joint.axis('off')
        ax_joint.text(0.5, 0.95, f'{robot_name} - Joint Control')
        
        joint_angles = list(robot.q)
        joint_sliders = []
        
        for i in range(6):
            ax_slider = plt.axes([0.1, 0.75 - i*0.1, 0.35, 0.03])
            slider = Slider(ax_slider, f'J{i+1}', -2*pi, 2*pi, valinit=joint_angles[i], valstep=0.01)
            joint_sliders.append(slider)
        
        def update_joints(val):
            if updating['flag']:
                return
            updating['flag'] = True
            
            for i, slider in enumerate(joint_sliders):
                joint_angles[i] = slider.val
            robot.q = joint_angles
            self.env.step(0.01)
            
            current_pose = robot.fkine(robot.q)
            for i, slider in enumerate(cart_sliders):
                slider.set_val(current_pose.t[i])
            
            updating['flag'] = False
        
        for slider in joint_sliders:
            slider.on_changed(update_joints)
        
        # Cartesian Control 
        ax_cart = fig.add_subplot(122)
        ax_cart.axis('off')
        ax_cart.text(0.5, 0.95, f'{robot_name} - Cartesian Control')
        
        current_pose = robot.fkine(robot.q)
        cart_pos = [current_pose.t[0], current_pose.t[1], current_pose.t[2]]
        cart_sliders = []
        
        for i, label in enumerate(['X (m)', 'Y (m)', 'Z (m)']):
            ax_slider = plt.axes([0.6, 0.75 - i*0.1, 0.35, 0.03])
            slider = Slider(ax_slider, label, -1.5, 1.5, valinit=cart_pos[i], valstep=0.01)
            cart_sliders.append(slider)
        
        def update_cartesian(val):
            if updating['flag']:
                return
            updating['flag'] = True
            
            current_pose = robot.fkine(robot.q)
            target_pose = SE3(cart_sliders[0].val, cart_sliders[1].val, cart_sliders[2].val)
            sol = robot.ikine_LM(target_pose, q0=robot.q, joint_limits=True)
            
            if sol.success:
                robot.q = sol.q
                self.env.step(0.01)
                
                for i, slider in enumerate(joint_sliders):
                    slider.set_val(robot.q[i])
            else:
                actual_pose = robot.fkine(robot.q)
                for i, slider in enumerate(cart_sliders):
                    slider.set_val(actual_pose.t[i])
                print(f"no sol found")
            
            updating['flag'] = False
        
        for slider in cart_sliders:
            slider.on_changed(update_cartesian)
        
        # Buttons
        reset_ax = plt.axes([0.35, 0.05, 0.1, 0.04])
        reset_btn = Button(reset_ax, 'Reset')
        
        def reset_all(event):
            updating['flag'] = True
            
            for i, slider in enumerate(joint_sliders):
                slider.reset()
            
            robot.q = [slider.val for slider in joint_sliders]
            self.env.step(0.01)
            
            current_pose = robot.fkine(robot.q)
            for i, slider in enumerate(cart_sliders):
                slider.set_val(current_pose.t[i])
            
            updating['flag'] = False
        
        reset_btn.on_clicked(reset_all)
        
        close_ax = plt.axes([0.55, 0.05, 0.1, 0.04])
        close_btn = Button(close_ax, 'Close')
        close_btn.on_clicked(lambda event: plt.close(fig))
        
        plt.show()

    def teach_aubo(self):
        self.create_teach_pendant(self.r2, 'Aubo I5')

    def teach_cobot(self):
        self.create_teach_pendant(self.r3, 'myCobot')

# ===================================================== COLLISION DETECTION CLASS =============================================================
class CollisionDetector:
    def __init__(self, env):
        self.env = env
        self.obstacles = []  # List of (vertices, faces, face_normals) tuples
        self.collision_spheres = []  # Visual indicators of collisions
        
    def add_obstacle(self, lwh, pose):#make obstacle lwh = [length, width, height] pose = center of object
        center = pose.t
        vertices, faces, face_normals = RectangularPrism(
            lwh[0], lwh[1], lwh[2], center=center
        ).get_data()
        self.obstacles.append((vertices, faces, face_normals))
        print(f"Added obstacle at {center} with size {lwh}")
        
    def is_point_inside_triangle(self, intersect_p, triangle_verts):#check if point is inside triangle
        u = triangle_verts[1, :] - triangle_verts[0, :]
        v = triangle_verts[2, :] - triangle_verts[0, :]
        
        uu = np.dot(u, u)
        uv = np.dot(u, v)
        vv = np.dot(v, v)
        
        w = intersect_p - triangle_verts[0, :]
        wu = np.dot(w, u)
        wv = np.dot(w, v)
        
        D = uv * uv - uu * vv
        
        s = (uv * wv - vv * wu) / D
        if s < 0.0 or s > 1.0:
            return False
            
        t = (uv * wu - uu * wv) / D
        if t < 0.0 or (s + t) > 1.0:
            return False
            
        return True
        
    def check_collision(self, robot, q_matrix, visualize=False):#check if path has collisions
        collision_found = False
        collision_configs = []
        
        for q_idx, q in enumerate(q_matrix):# Get transform of every joint
            tr = robot.fkine_all(q).A
            
            # Check each link against each obstacle
            for i in range(np.size(tr, 2) - 1):
                for vertices, faces, face_normals in self.obstacles:
                    for j, face in enumerate(faces):
                        vert_on_plane = vertices[face][0]
                        intersect_p, check = line_plane_intersection(face_normals[j], vert_on_plane, tr[i][:3, 3], tr[i+1][:3, 3])
                        if check == 1:
                            triangle_list = np.array(
                                list(combinations(face, 3)), dtype=int
                            )
                            for triangle in triangle_list:
                                if self.is_point_inside_triangle(
                                    intersect_p, vertices[triangle]
                                ):
                                    collision_found = True
                                    collision_configs.append(q_idx)
                                    
                                    if visualize:
                                        sphere = Sphere(radius=0.03, color=[1.0, 0.0, 0.0, 1.0])
                                        sphere.T = transl(intersect_p[0], intersect_p[1], intersect_p[2])
                                        self.env.add(sphere)
                                        self.collision_spheres.append(sphere)
                                    
                                    return collision_found, list(set(collision_configs))
        return collision_found, list(set(collision_configs))
    
    def clear_collision_spheres(self):
        self.collision_spheres.clear()
    
    def find_collision_free_path(self, robot, q_start, q_goal, max_attempts=50):#generate collision avoiding path
        
        # First check direct path
        q_direct = self.interpolate_path(q_start, q_goal, max_step=np.deg2rad(5))
        collision, _ = self.check_collision(robot, q_direct, visualize=False)
        
        if not collision:
            return True, q_direct
        
        print("Collision detected")
        
        # Use RRT approach to find path
        q_waypoints = [q_start, q_goal]
        checked_till = 0
        q_matrix = []
        
        for attempt in range(max_attempts):
            for i in range(checked_till, len(q_waypoints) - 1):
                q_segment = self.interpolate_path(
                    q_waypoints[i], 
                    q_waypoints[i+1],
                    max_step=np.deg2rad(10)
                )
                
                collision, _ = self.check_collision(robot, q_segment, visualize=False)
                
                if not collision:
                    q_matrix.extend(q_segment)
                    checked_till = i + 1
                    
                    # Try to connect to goal
                    q_to_goal = self.interpolate_path(
                        q_matrix[-1],
                        q_goal,
                        max_step=np.deg2rad(10)
                    )
                    collision_goal, _ = self.check_collision(
                        robot, q_to_goal, visualize=False
                    )
                    
                    if not collision_goal:
                        q_matrix.extend(q_to_goal)
                        return True, q_matrix
                    break
                else:
                    # Generate random collision-free waypoint
                    q_rand = None
                    for _ in range(20):
                        q_test = self.random_config(robot)
                        collision_test, _ = self.check_collision(
                            robot, [q_test], visualize=False
                        )
                        if not collision_test:
                            q_rand = q_test
                            break
                    
                    if q_rand is not None:
                        q_waypoints = (
                            q_waypoints[:i+1] + 
                            [q_rand] + 
                            q_waypoints[i+1:]
                        )
                    break
        
        print(f"no collision free path found")
        return False, q_direct
    
    def interpolate_path(self, q1, q2, max_step=np.deg2rad(1)):
        steps = 2
        while np.any(max_step < np.abs(np.diff(
            rtb.jtraj(q1, q2, steps).q, axis=0
        ))):
            steps += 1
        return rtb.jtraj(q1, q2, steps).q
    
    def random_config(self, robot):#generate random joint position to change paths
        q_rand = []
        for joint in robot.links:
            q_min, q_max = joint.qlim
            q_rand.append(np.random.uniform(q_min, q_max))
        return q_rand
    
# ==================== MAIN EXECUTION ====================
speed = 0.005
robot = PickPlaceRobot()
print("  1 - Open Aubo teach pendant")
print("  2 - Open myCobot teach pendant")
print("  SPACE - Start operation")

# Menu system
teach_mode = True
while teach_mode:
    if keyboard.is_pressed('1'):
        print("\n Aubo teach opening")
        robot.teach_aubo()
        print("Aubo teach closed.\n")
        time.sleep(0.5)
    
    elif keyboard.is_pressed('2'):
        print("\nCobot teach opening")
        robot.teach_cobot()
        print("myCobot teach closed.\n")
        time.sleep(0.5)
    
    elif keyboard.is_pressed('space'):
        print("\nstarting pick and place\n")
        time.sleep(0.3)
        teach_mode = False
    
    time.sleep(0.05)

try:
    robot.UR3_pick_place()
    robot.robots_pick_place()
    print("operation complete")
except KeyboardInterrupt:
    print("\nOperation Interrupted")
finally:
    robot.shutdown()
    time.sleep(2)
