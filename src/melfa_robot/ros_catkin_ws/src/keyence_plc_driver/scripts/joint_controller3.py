#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
import time
import numpy as np
import math
import PyKDL as kdl

class RobotArmKDL:
    def __init__(self):
        # Joint limits in radians
        self.joint_limits = np.array([
            [math.radians(-240), math.radians(240)],  # j1
            [math.radians(-120), math.radians(120)],  # j2
            [math.radians(0),    math.radians(161)],  # j3
            [math.radians(-200), math.radians(200)],  # j4
            [math.radians(-120), math.radians(120)]   # j5
        ])

        # Build KDL chain (only up to joint 5, no fixed end-effector segment)
        self.chain = self._create_kdl_chain()

        # Create joint limit arrays for 5 joints only
        q_min = kdl.JntArray(5)
        q_max = kdl.JntArray(5)
        for i in range(5):
            q_min[i] = self.joint_limits[i][0]
            q_max[i] = self.joint_limits[i][1]

        # Create solvers
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_pos_solver = kdl.ChainIkSolverPos_NR_JL(
            self.chain,
            q_min,
            q_max,
            self.fk_solver,
            self.ik_vel_solver,
            maxiter=100,
            eps=1e-6
        )
        
        # End effector offset (applied after joint 5 in the Z direction)
        self.end_effector_offset = 0.12485  # meters
        


    def _create_kdl_chain(self):

        chain = kdl.Chain()

        # Segment 0: Joint 1 at base, RotZ
        # After rotating, Joint 2 starts at [0, 0, 0] (no offset from Joint 1)
        # But Joint 2's location after A0*A1 rotation would be at [0,0,0] before A1 translation
        # So Frame should move us to where Joint 2 is BEFORE it rotates
        # Joint 2 is at the base, so Frame is identity
        chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Vector(0.0, 0.0, 0.0))
        ))

        # Segment 1: Joint 2, RotY  
        # After Joint 2 rotates, we translate [0, 0, 350] in its local frame
        # This translation brings us to Joint 3's location
        chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0.0, 0.0, 0.350))
        ))

        # Segment 2: Joint 3, RotY
        # After Joint 3 rotates, we translate [0, 0, 310] in its local frame
        chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0.0, 0.0, 0.310))
        ))

        # Segment 3: Joint 4, RotZ
        # After Joint 4 rotates, we translate [-50, 0, 50] in its local frame
        chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),
            kdl.Frame(kdl.Vector(-0.050, 0.0, 0.050))
        ))

        # Segment 4: Joint 5, RotY
        # After Joint 5 rotates, we translate [0, 0, 285.15] in its local frame
        chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0.0, 0.0, 0.28515))
        ))

        return chain

    
    def forward_kinematics(self, joint_angles_rad):
        """Calculate FK using manual A-matrices (bypassing KDL for now)
        
        Args:
            joint_angles_rad: List of 5 joint angles in radians
            
        Returns:
            [x, y, z] in mm
        """
        # Convert to degrees for A-matrix functions
        joint_angles_deg = [math.degrees(a) for a in joint_angles_rad]
        
        # Use manual FK calculation
        result = self.manual_fk(joint_angles_deg, verbose=False)
        
        return result
    
    def inverse_kinematics(self, target_pos_mm, initial_guess_rad):
        """Solve IK using Jacobian-based numerical method
        
        Args:
            target_pos_mm: [x, y, z] in mm (desired end-effector position)
            initial_guess_rad: Initial joint angles in radians (5 values)
            
        Returns:
            (joint_angles_rad, success) - returns 5 joint angles
        """
        # Use iterative Jacobian-based IK
        max_iterations = 50
        tolerance = 1.0  # mm
        
        # Start with initial guess
        q = np.array(initial_guess_rad)
        target = np.array(target_pos_mm)
        
        for iteration in range(max_iterations):
            # Compute current position
            q_deg = [math.degrees(angle) for angle in q]
            current_pos = np.array(self.manual_fk(q_deg, verbose=False))
            
            # Compute error
            error = target - current_pos
            error_magnitude = np.linalg.norm(error)
            
            # Check if we've converged
            if error_magnitude < tolerance:
                # Verify joint limits
                if self.check_joint_limits(q):
                    return q.tolist(), True
                else:
                    return q.tolist(), False
            
            # Compute Jacobian
            J = self.compute_jacobian_numerical(q)
            
            # Compute pseudo-inverse
            try:
                J_pinv = np.linalg.pinv(J)
            except:
                return initial_guess_rad, False
            
            # Update joint angles using damped least squares
            damping = 0.01
            delta_q = J_pinv @ error
            
            # Limit step size to avoid large jumps
            max_step = 0.1  # radians
            step_size = np.linalg.norm(delta_q)
            if step_size > max_step:
                delta_q = delta_q * (max_step / step_size)
            
            q = q + delta_q
            
            # Clip to joint limits
            for i in range(5):
                q[i] = np.clip(q[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        # Failed to converge
        return initial_guess_rad, False
    
    def compute_jacobian_numerical(self, joint_angles_rad):
        """Compute Jacobian matrix numerically using finite differences
        
        Args:
            joint_angles_rad: numpy array of 5 joint angles in radians
            
        Returns:
            3x5 Jacobian matrix
        """
        epsilon = 1e-6  # Small perturbation
        J = np.zeros((3, 5))
        
        # Convert to degrees for FK
        q_deg = [math.degrees(angle) for angle in joint_angles_rad]
        
        # Current position
        pos_0 = np.array(self.manual_fk(q_deg, verbose=False))
        
        # Compute each column of Jacobian
        for i in range(5):
            # Perturb joint i
            q_perturbed = joint_angles_rad.copy()
            q_perturbed[i] += epsilon
            
            q_perturbed_deg = [math.degrees(angle) for angle in q_perturbed]
            pos_perturbed = np.array(self.manual_fk(q_perturbed_deg, verbose=False))
            
            # Finite difference
            J[:, i] = (pos_perturbed - pos_0) / epsilon
        
        return J
    
    def check_joint_limits(self, joint_angles_rad):
        """Check if within limits"""
        for i in range(5):
            if joint_angles_rad[i] < self.joint_limits[i][0] or \
               joint_angles_rad[i] > self.joint_limits[i][1]:
                return False
        return True
    
    def manual_fk(self, joint_angles_deg, verbose=True):
        """Manual FK calculation using A-matrices for verification"""
        t1, t2, t3, t4, t5 = joint_angles_deg
        
        # A0
        t1_rad = math.radians(t1)
        A0 = np.array([
            [math.cos(t1_rad), -math.sin(t1_rad), 0.0, 0.0],
            [math.sin(t1_rad),  math.cos(t1_rad), 0.0, 0.0],
            [0.0,               0.0,               1.0, 0.0],
            [0.0,               0.0,               0.0, 1.0]
        ])
        
        # A1
        t2_rad = math.radians(t2)
        A1 = np.array([
            [math.cos(t2_rad),  0.0, math.sin(t2_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t2_rad), 0.0, math.cos(t2_rad), 350.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # A2
        t3_rad = math.radians(t3)
        A2 = np.array([
            [math.cos(t3_rad),  0.0, math.sin(t3_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t3_rad), 0.0, math.cos(t3_rad), 310.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # A3
        t4_rad = math.radians(t4)
        A3 = np.array([
            [math.cos(t4_rad), -math.sin(t4_rad), 0.0, -50.0],
            [math.sin(t4_rad),  math.cos(t4_rad), 0.0, 0.0],
            [0.0,               0.0,              1.0, 50.0],
            [0.0,               0.0,              0.0, 1.0]
        ])
        
        # A4
        t5_rad = math.radians(t5)
        A4 = np.array([
            [math.cos(t5_rad),  0.0, math.sin(t5_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t5_rad), 0.0, math.cos(t5_rad), 285.15],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # Compute total transformation
        T = A0 @ A1 @ A2 @ A3 @ A4
        
        if verbose:
            # Log joint 5 position (at origin of frame 5, before end-effector offset)
            j5_pos = T @ np.array([0.0, 0.0, 0.0, 1.0])
            rospy.loginfo(f"Manual Joint 5 pos: X={j5_pos[0]:.1f}, Y={j5_pos[1]:.1f}, Z={j5_pos[2]:.1f}")
            
            # Log the Z-axis of frame 5 (3rd column of rotation matrix)
            z_axis_manual = T[:3, 2]
            rospy.loginfo(f"Manual Z-axis: [{z_axis_manual[0]:.3f}, {z_axis_manual[1]:.3f}, {z_axis_manual[2]:.3f}]")
        
        # End effector position
        pos_homogeneous = T @ np.array([0.0, 0.0, 124.85, 1.0])
        return pos_homogeneous[:3].tolist()


class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm KDL Controller")
        self.root.geometry("700x600")

        # Initialize KDL solver
        try:
            self.kdl_solver = RobotArmKDL()
        except Exception as e:
            rospy.logerr(f"Failed to initialize KDL solver: {e}")
            raise
        
        # Joint names
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Joint limits in radians
        self.joint_limits_rad = [
            [math.radians(-240), math.radians(240)],
            [math.radians(-120), math.radians(120)],
            [math.radians(0), math.radians(161)],
            [math.radians(-200), math.radians(200)],
            [math.radians(-120), math.radians(120)],
            [None, None]
        ]
        
        # State variables
        self.joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        self.control_mode = "JOINT"
        
        # Movement control
        self.last_movement_time = 0
        self.movement_delay = 0.05
        
        # Cartesian control
        self.current_cartesian = [0.0, 0.0, 0.0]
        self.cartesian_update_pending = False
        self.last_ik_time = 0
        self.ik_cooldown = 0.05
        self.last_cartesian_input_time = 0
        self.cartesian_control_timeout = 0.5
        self.fk_calculated = False  # Track if FK has been calculated at least once
        
        # ROS Publishers & Subscribers
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.setup_gui()
        
        rospy.loginfo("Robot KDL Controller started")

    def setup_gui(self):
        """Build GUI"""
        mode_frame = tk.Frame(self.root, padx=10, pady=10)
        mode_frame.pack()
        
        tk.Label(mode_frame, text="Control Mode:", font=("Arial", 14, "bold")).pack(side=tk.LEFT, padx=5)
        self.mode_label = tk.Label(mode_frame, text="JOINT", font=("Arial", 14, "bold"), fg="green")
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)
        
        joint_frame = tk.LabelFrame(self.root, text="Joint Angles (rad)", padx=10, pady=10)
        joint_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.joint_labels = {}
        for i, name in enumerate(self.joint_names):
            row_frame = tk.Frame(joint_frame)
            row_frame.pack(fill='x', pady=2)
            
            tk.Label(row_frame, text=name, font=("Arial", 11, "bold"), width=10, anchor='w').pack(side=tk.LEFT)
            self.joint_labels[name] = tk.Label(row_frame, text=f"{self.joint_positions[i]:.4f}", 
                                               font=("Arial", 11), width=12, anchor='e')
            self.joint_labels[name].pack(side=tk.LEFT)
        
        cartesian_frame = tk.LabelFrame(self.root, text="Cartesian Position (mm)", padx=10, pady=10)
        cartesian_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.cartesian_labels = {}
        for axis in ['X', 'Y', 'Z']:
            row_frame = tk.Frame(cartesian_frame)
            row_frame.pack(fill='x', pady=2)
            
            tk.Label(row_frame, text=axis, font=("Arial", 11, "bold"), width=10, anchor='w').pack(side=tk.LEFT)
            self.cartesian_labels[axis] = tk.Label(row_frame, text="0.00", 
                                                    font=("Arial", 11), width=12, anchor='e')
            self.cartesian_labels[axis].pack(side=tk.LEFT)
        
        self.selected_label = tk.Label(self.root, text="", font=("Arial", 13, "bold"), fg="blue")
        self.selected_label.pack(pady=10)
        
        self.status_label = tk.Label(self.root, text="Ready (KDL)", font=("Arial", 10), fg="gray", anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=5)

    def joint_states_callback(self, msg):
        """Updates joint positions and calculates FK"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        # Always update Cartesian display from FK unless we're actively controlling in Cartesian mode
        current_time = time.time()
        
        # Update Cartesian position from FK in these cases:
        # 1. Not currently processing a cartesian update
        # 2. AND (in JOINT mode OR enough time has passed since last cartesian input)
        should_update_fk = (
            not self.cartesian_update_pending and 
            (self.control_mode == "JOINT" or 
             current_time - self.last_cartesian_input_time > self.cartesian_control_timeout)
        )
        
        if should_update_fk:
            try:
                joint_angles_rad = self.joint_positions[:5]
                self.current_cartesian = self.kdl_solver.forward_kinematics(joint_angles_rad)
                if not self.fk_calculated:
                    self.fk_calculated = True
            except Exception as e:
                rospy.logwarn(f"FK calculation failed: {e}")
                import traceback
                rospy.logwarn(traceback.format_exc())

    def joy_callback(self, msg):
        """Handles PS4 controller"""
        button_cross = msg.buttons[0]
        button_circle = msg.buttons[1]
        button_triangle = msg.buttons[2]
        button_square = msg.buttons[3]
        button_l1 = msg.buttons[4]
        button_r1 = msg.buttons[5]
        
        current_time = time.time()
        
        # Switch mode
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_l1 == 1 or button_r1 == 1:
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                
                # When switching to Cartesian mode, initialize with current position
                if self.control_mode == "CARTESIAN":
                    try:
                        joint_angles_rad = self.joint_positions[:5]
                        self.current_cartesian = self.kdl_solver.forward_kinematics(joint_angles_rad)
                        self.status_label.config(text="Switched to Cartesian mode", fg="green")
                    except Exception as e:
                        rospy.logwarn(f"FK calculation failed on mode switch: {e}")
            
            if button_triangle == 1:
                self.current_joint_index = (self.current_joint_index - 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
            elif button_cross == 1:
                self.current_joint_index = (self.current_joint_index + 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time

        moved = False
        
        if self.control_mode == "JOINT":
            if current_time - self.last_movement_time < self.movement_delay:
                return
                
            step = 0.01
            if button_circle == 1:
                new_position = self.joint_positions[self.current_joint_index] + step
                limits = self.joint_limits_rad[self.current_joint_index]
                if limits[0] is not None and limits[1] is not None:
                    if new_position <= limits[1]:
                        self.joint_positions[self.current_joint_index] = new_position
                        moved = True
                    else:
                        self.status_label.config(text=f"{self.joint_names[self.current_joint_index]} at upper limit", fg="orange")
                else:
                    self.joint_positions[self.current_joint_index] = new_position
                    moved = True
                self.last_movement_time = current_time
                
            elif button_square == 1:
                new_position = self.joint_positions[self.current_joint_index] - step
                limits = self.joint_limits_rad[self.current_joint_index]
                if limits[0] is not None and limits[1] is not None:
                    if new_position >= limits[0]:
                        self.joint_positions[self.current_joint_index] = new_position
                        moved = True
                    else:
                        self.status_label.config(text=f"{self.joint_names[self.current_joint_index]} at lower limit", fg="orange")
                else:
                    self.joint_positions[self.current_joint_index] = new_position
                    moved = True
                self.last_movement_time = current_time
                
            if moved:
                self.send_command()
                
        else:  # CARTESIAN
            if current_time - self.last_ik_time < self.ik_cooldown:
                return
            
            step = 0.3
            if button_circle == 1:
                self.current_cartesian[self.current_joint_index] += step
                moved = True
                self.last_cartesian_input_time = current_time
            elif button_square == 1:
                self.current_cartesian[self.current_joint_index] -= step
                moved = True
                self.last_cartesian_input_time = current_time
            
            if moved:
                self.last_ik_time = current_time
                self.cartesian_update_pending = True
                
                current_angles_rad = self.joint_positions[:5]
                new_angles_rad, success = self.kdl_solver.inverse_kinematics(
                    self.current_cartesian,
                    current_angles_rad
                )
                
                if success and self.kdl_solver.check_joint_limits(new_angles_rad):
                    for i in range(5):
                        self.joint_positions[i] = new_angles_rad[i]
                    
                    reached_pos = self.kdl_solver.forward_kinematics(new_angles_rad)
                    error = np.linalg.norm(np.array(self.current_cartesian) - np.array(reached_pos))
                    
                    self.status_label.config(text=f"KDL IK OK ({error:.1f}mm)", fg="green")
                    self.send_command()
                else:
                    if button_circle == 1:
                        self.current_cartesian[self.current_joint_index] -= step
                    elif button_square == 1:
                        self.current_cartesian[self.current_joint_index] += step
                    
                    if not success:
                        self.status_label.config(text=f"KDL IK Failed", fg="red")
                    else:
                        self.status_label.config(text=f"Joint limit reached!", fg="orange")
                
                self.cartesian_update_pending = False

    def send_command(self):
        """Publish command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.1)
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    def update_gui(self):
        """Update GUI"""
        for i, name in enumerate(self.joint_names):
            self.joint_labels[name].config(text=f"{self.joint_positions[i]:.3f}")
        
        # Always update Cartesian display - ensure we have valid data
        try:
            for i, axis in enumerate(['X', 'Y', 'Z']):
                value = self.current_cartesian[i]
                self.cartesian_labels[axis].config(text=f"{value:.1f}")
        except (IndexError, TypeError) as e:
            rospy.logwarn_once(f"Cartesian display update failed: {e}")
        
        if self.control_mode == "JOINT":
            selected = self.joint_names[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected}")
            
            for name, label in self.joint_labels.items():
                label.config(fg="blue" if name == selected else "black")
            for label in self.cartesian_labels.values():
                label.config(fg="black")
        else:
            axes = ['X', 'Y', 'Z']
            selected = axes[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected} axis")
            
            for axis, label in self.cartesian_labels.items():
                label.config(fg="blue" if axis == selected else "black")
            for label in self.joint_labels.values():
                label.config(fg="black")
        
        self.root.after(150, self.update_gui)


def main():
    rospy.init_node("robot_kdl_controller")
    root = tk.Tk()
    gui = RobotControllerGUI(root)
    gui.update_gui()
    root.mainloop()

if __name__ == "__main__":
    main()
