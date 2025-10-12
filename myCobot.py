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

#-----------------------------------------------------------------------------------#
class myCobot(DHRobot3D):
    def __init__(self):
        """
            myCobot Robot by DHRobot3D class

            Example usage:
            >>> from Assignment_2 import myCobot
            >>> from ir_support import swift

            >>> r = myCobot()
            >>> q = [0,-pi/2,pi/4,0,0,0]
            >>> r.q = q
            >>> q_goal = [r.q[i]-pi/4 for i in range(r.n)]
            >>> env = swift.Swift()
            >>> env.launch(realtime= True)
            >>> r.add_to_env(env)
            >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
            >>> for q in qtraj:
            >>>    r.q = q
            >>>    env.step(0.02)
        """
        # DH links
        links = self._create_DH() 

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'mycobot_base',
                            link1 = 'mycobot_base_elbow',
                            link2 = 'mycobot_longarm',
                            link3 = 'mycobot_longarm2',
                            link4 = 'mycobot_elbow2',
                            link5 = 'mycobot_end_effector',
                            link6 = 'mycobot_end_effector')

        # A joint config and the 3D object transforms to match that config
        qtest = [0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(-0.09,0.07,0),
                            spb.transl(-0.0920,0.07,0),
                            spb.transl(0.21,0.07,0.122)@ spb.troty(-pi/2),

#                            spb.transl(0.07,0.09,0) @ spb.trotz(-pi/2),
#                            spb.transl(0.07,0.09,0) @ spb.trotz(-pi/2),
                            spb.transl(0.21,0.07,0.122)@ spb.troty(-pi/2),
                            spb.transl(0.21,0.07,0.122) @ spb.troty(-pi/2),
                            spb.transl(0.21,0.07,0.122) @ spb.troty(-pi/2),
                            spb.transl(0.21,0.07,0.122) @ spb.troty(-pi/2)  
                        ]


        '''
        qtest_transforms = [
            spb.transl(-0.09, 0.07, 0),  # link0 (base)
    
            # link1: d=0.21, α=-90°, offset=0
            spb.transl(-0.09, 0.07, 0.21) @ spb.trotx(pi/2),
    
            # link2: a=0.25, offset=+90°
            spb.transl(0.25, 0, 0) @ spb.trotz(pi/2),
            
            # link3: a=0.25
            spb.transl(0.25, 0, 0),
            
            # link4: d=0.109, α=-90°, offset=+90°
            spb.transl(0, 0, 0.109) @ spb.trotx(-pi/2) @ spb.trotz(pi/2),
            
            # link5: d=0.107, α=-90°, offset=+180°
            spb.transl(0, 0, 0.107) @ spb.trotx(-pi/2) @ spb.trotz(pi),
            
            # link6: d=0.076
            spb.transl(0, 0, 0.076)
        ]
        '''

        # 👇 Add this block to print transforms when the robot initializes
        for i, T in enumerate(qtest_transforms):
            print(f"Link {i} transform:\n{T}\n")

        current_path = os.path.abspath(os.path.dirname(__file__))
        print(current_path)
        current_path = current_path +"/myCobot3D"
        print(current_path)

        super().__init__(links, link3D_names, name = 'myCobot', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest

#-----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
#        a = [0, -250, -250, 0, 0, 0]
 #       d = [210, 0, 0, 109.5, 107, 76.2] 


        a = [0, -0.250, -0.250, 0, 0, 0]
        d = [0.210, 0, 0, 0.109, 0.107, 0.076] 


        alpha = [-pi/2, 0, 0, -pi/2, -pi/2, 0]
        theta = [0, pi/2, 0, pi/2, pi, 0]
            
        qlim = [[-pi, pi],
                [-3*pi/2, pi/2],              # Joint 2: -270° to +90°
                [-5*pi/6, 5*pi/6],            # Joint 3: -150° to +150°
                [-13*pi/9, 4*pi/9],           # Joint 4: -260° to +80°
                [-14*pi/15, 14*pi/15],        # Joint 5: -168° to +168°
                [-29*pi/30, 29*pi/30]         # Joint 6: -174° to +174°
        ]
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(
                d=d[i], 
                a=a[i], 
                alpha=alpha[i], 
                offset=theta[i], 
                qlim= qlim[i]
                )
            links.append(link)
        return links

#-----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """

        env = swift.Swift()
        env.launch(realtime = True)

        self.q = self._qtest
        self.base = SE3(0,0,0)
        self.add_to_env(env)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q

        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.05)
        time.sleep(3)

    #-----------------------------------------------------------------------------------#
    def teach(self):
        """
        Interactive teach pendant to manuaully move and control the robot's joints
        Using sliders to move the joint angles
        """
        
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        env = swift.Swift()
        env.launch(realtime=True)

        #cyl_viz = CylindricalDHRobotPlot(self, cylinder_radius=0.05, color="#3478f62e")
        #robot_with_cyl = cyl_viz.create_cylinders()
        #env.add(robot_with_cyl)   

        self.q = self._qtest
        self.base = SE3(0,0,0)
        self.add_to_env(env)

        # Create the figure and the axes for the sliders
        fig, ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.25, bottom=0.7)

        #Initial Joint Angles
        joint_angles = list(self.q)

        # Create sliders for each joint
        sliders = []
        sliders_axes = []

        for i in range(5):
            print(f"This is I: {i}")
            ax_slider = plt.axes([0.25, 0.05 + i*0.1, 0.65, 0.03])
            slider = Slider(
                ax_slider,
                f'joint {i+1}',
                -2*pi,
                2*pi,
                valinit=joint_angles[i],
                valstep=0.01
            )
            sliders.append(slider)
            sliders_axes.append(ax_slider)

        #update slider function
        def update(val):
            print("################################")
            print(f"Joint Angles-1: {joint_angles}")
            for i, slider in enumerate(sliders):
                    joint_angles[i] = slider.val
            self.q = joint_angles
            print(f"Joint Angles-2: {joint_angles}")

            env.step(0.01)

            # Get cylinder transforms and print them
#            print(f"Cylinder Transforms: {robot_with_cyl}")

        #connecting the sliders to the update function
        for slider in sliders:
            slider.on_changed(update)

        #Add a reset button
        from matplotlib.widgets import Button
        resetax = plt.axes([0.8, 0.3, 0.1, 0.04])
        button = Button(resetax, 'Reset', color='lightgoldenrodyellow', hovercolor='0.975')

        def reset(event):
            #home_pos = [0, -pi/2, 0, 0, 0, 0]
 
            for i, slider in enumerate(sliders):
                slider.reset()
                print(f"Slider {i} reset to {slider.val}")

        button.on_clicked(reset)

        #initial draw
        preset_ax = plt.axes([0.1, 0.3, 0.1, 0.04])
        preset_button = Button(preset_ax, 'Preset', color='lightblue', hovercolor='0.975')

        def go_home(event):
            home_pos = [0, -pi/2, 0, 0, 0, 0]
            for i, slider in enumerate(sliders):
                slider.set_val(home_pos[i])

        preset_button.on_clicked(go_home)

        #instructions to be added
        ax.text(0.5, 0.9, 'Use sliders to move individual joints',
                    ha = 'center', va = 'center', transform = ax.transAxes, fontsize = 12, weight = 'bold')
        ax.text(0.5, 0.8, 'Watch which joint is moving in the Swift window', 
                    ha = 'center', va = 'center', transform = ax.transAxes, fontsize = 10)
        ax.axis('off')

        plt.show()
        env.hold()

#-----------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = myCobot()
    r.q = [0, -pi/2, 0, 0, 0, 0]
    print(r.fkine(r.q))
    r.teach()
