##  @file
#   @brief UR3 Robot defined by standard DH parameters with 3D model
#   @author Ho Minh Quang Ngo
#   @date Jul 20, 2023

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
from ir_support import CylindricalDHRobotPlot

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class AuboI5(DHRobot3D):
    def __init__(self):
        """
            UR3 Robot by DHRobot3D class

            Example usage:
            >>> from ir-support import UR3
            >>> import swift

            >>> r = UR3()
            >>> q = [0,-pi/2,pi/4,0,0,0]r
            >>> r.q = q
            >>> q_goal = [r.q[i]-pi/4 for i in range(r.n)]
            >>> env = swift.Swift()
            >>> env.launch(realtime= True)
            >>> r.add_to_env(env)
            >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
            >>> for q in qtraj:r
            >>>    r.q = q
            >>>    env.step(0.02)
        """
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_aubo',
                            link1 = 'shoulder_aubo',
                            link2 = 'upperarm_aubo',
                            link3 = 'forearm_aubo',
                            link4 = 'wrist1_aubo',
                            link5 = 'wrist2_aubo',
                            link6 = 'wrist3_aubo')

        # A joint config and the 3D object transforms to match that config
        qtest = [0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(0,0,0),
                            spb.transl(0,0,0),
                            spb.transl(0,0,0),
                            spb.transl(0,0,0),
                            spb.transl(0,0,0),
                            spb.transl(0,0,0),
                            spb.transl(0,0,0)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'UR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        a = [0,  0.408, 0.376, 0, 0, 0]
        d = [0.1215, 0, 0, 0.1225, 0.108, 0.094] 
    
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        theta = [pi, pi, 0, pi/2, 0, 0]
        
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=theta[i], qlim= qlim[i])
            links.append(link)
        return links

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        
        env = swift.Swift()
        env.launch(realtime= True)

        self.q = self._qtest
        self.base = SE3(0,0,0)
        self.add_to_env(env)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.05)
            # fig.step(0.01)
        time.sleep(3)
        #env.hold()

    # -----------------------------------------------------------------------------------#
    def teach(self):
        """
        Interactive teach pendant to manually control each joint
        Use sliders to move individual joints and see which 3D models move
        """
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
        
        env = swift.Swift()
        env.launch(realtime=True)
        
        # cyl_viz = CylindricalDHRobotPlot(self, cylinder_radius=0.05, color="#3478f62e")
        # robot_with_cyl = cyl_viz.create_cylinders()
        # env.add(robot_with_cyl)   

        self.q = self._qtest
        self.base = SE3(0,0,0)
        self.add_to_env(env)
        
        # Create figure for sliders
        fig, ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.25, bottom=0.35)
        
        # Initial joint angles
        joint_angles = list(self.q)
        
        # Create sliders for each joint
        sliders = []
        slider_axes = []
        
        for i in range(6):
            ax_slider = plt.axes([0.25, 0.05 + i*0.04, 0.65, 0.03])
            slider = Slider(
                ax_slider, 
                f'Joint {i+1}', 
                -2*pi, 
                2*pi, 
                valinit=joint_angles[i],
                valstep=0.01
            )
            sliders.append(slider)
            slider_axes.append(ax_slider)
        
        # Update function for sliders
        def update(val):
            for i, slider in enumerate(sliders):
                joint_angles[i] = slider.val
            self.q = joint_angles
            env.step(0.01)
        
        # Connect sliders to update function
        for slider in sliders:
            slider.on_changed(update)
        
        # Add reset button
        from matplotlib.widgets import Button
        resetax = plt.axes([0.8, 0.3, 0.1, 0.04])
        button = Button(resetax, 'Reset', color='lightgoldenrodyellow', hovercolor='0.975')
        
        def reset(event):
            for i, slider in enumerate(sliders):
                slider.reset()
        
        button.on_clicked(reset)
        
        # Add preset positions
        preset_ax = plt.axes([0.1, 0.3, 0.1, 0.04])
        preset_button = Button(preset_ax, 'Home', color='lightblue', hovercolor='0.975')
        
        def go_home(event):
            home_pos = [0, -pi/2, 0, 0, 0, 0]
            for i, slider in enumerate(sliders):
                slider.set_val(home_pos[i])
        
        preset_button.on_clicked(go_home)
        
        # Add instructions
        ax.text(0.5, 0.9, 'Use sliders to move individual joints', 
                ha='center', va='center', transform=ax.transAxes, fontsize=14, weight='bold')
        ax.text(0.5, 0.8, 'Watch which 3D models move to identify joint-model mappings', 
                ha='center', va='center', transform=ax.transAxes, fontsize=10)
        ax.axis('off')
        
        plt.show()
        env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = AuboI5()
    r.teach()