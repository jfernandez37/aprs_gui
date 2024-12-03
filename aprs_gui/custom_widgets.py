import PyKDL
import subprocess
import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from typing import Optional
from copy import copy
from math import sin, cos, atan2, pi
from time import time
from ament_index_python.packages import get_package_share_directory
import re
import threading
from rclpy import spin_once
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from difflib import SequenceMatcher
from functools import partial
from time import sleep
from plansys2_msgs.msg import Plan

from aprs_gui.canvas_tooltips import CanvasTooltip

from aprs_interfaces.msg import Tray, SlotInfo
from aprs_interfaces.srv import Pick, Place, MoveToNamedPose, PneumaticGripperControl, GeneratePlan
from aprs_interfaces.action import ExecutePlan


class LiveImage(ctk.CTkLabel):
    def __init__(self, frame):
        super().__init__(frame, text="Image not found", fg_color="#C2C2C2", height=400, width=400, font=("UbuntuMono",50))
        self.current_image: Optional[ctk.CTkImage] = None
        self.update_image()

    def update_image(self):
        if self.current_image is not None:
            self.configure(text="", image=self.current_image, fg_color="transparent")
        else:
            self.configure(text="Image not found", fg_color="#C2C2C2")
        self.after(50, self.update_image)

class TrayCanvas(tk.Canvas):
    tray_corners_ = {Tray.SMALL_GEAR_TRAY: [
        (-0.08, 0.08),
        (0.08, 0.08),
        (0.08, -0.08),
        (-0.08, -0.08)
    ],
    Tray.MEDIUM_GEAR_TRAY: [
        (-0.098, 0.098),
        (0.098, 0.098),
        (0.098, -0.098),
        (-0.098, -0.098)
    ],
    Tray.LARGE_GEAR_TRAY: [
        (-0.105, 0.113),
        (0.105, 0.113),
        (0.105, -0.025),
        (-0.105, -0.025),
    ],
    Tray.M2L1_KIT_TRAY: [
        (-0.108, 0.043), # Top left corner
        (0.108, 0.043), # Top right corner
        (0.108, -0.054), # Bottom right corner
        (0.019, -0.128), # Very bottom right corner
        (-0.019, -0.128), # Very bottom left corner
        (-0.108, -0.054), # Bottom left corner
    ],
    Tray.S2L2_KIT_TRAY: [
        (-0.105, 0.113), # Top left corner
        (0.105, 0.113), # Top right corner
        (0.105, 0.0), # Middle right coner
        (0.067, -0.08), # Bottom right corner
        (-0.067,  -0.08), # Bottom left corner
        (-0.105, 0.0), # Middle left coner
    ],
    -1: [
        (-0.02, 0.02),
        (0.02, 0.02),
        (0.02, -0.02),
        (-0.02, -0.02)
    ]}

    gear_radii_ = {SlotInfo.SMALL: 0.032,
                   SlotInfo.MEDIUM: 0.04,
                   SlotInfo.LARGE: 0.05}
    
    tray_colors_ = {
        Tray.SMALL_GEAR_TRAY: "#3b3a3a",
        Tray.MEDIUM_GEAR_TRAY: "#3b3a3a",
        Tray.LARGE_GEAR_TRAY: "#3b3a3a",
        Tray.M2L1_KIT_TRAY: "#3b3a3a",
        Tray.S2L2_KIT_TRAY: "#3b3a3a"
    }

    fiducial_square_measurements = (0.04, 0.04)
    
    def __init__(self, frame):
        super().__init__(frame, height=150, width=150, bd = 0, highlightthickness=0)
        self.conversion_factor = None
        self.all_trays: Optional[list[Tray]] = None
        self.width: Optional[int] = None
        self.side_canvas = False
        self.global_conversion_factor = None
        self.update_canvas()
    
    def update_canvas(self, once = False):
        self.delete("all")
        if self.side_canvas:
            try:
                self.configure(height=150, width=int(self.width * 3 / 8))
            except:
                self.configure(height=150, width=150)
        else:
            self.configure(height=400, width=self.width)
        if self.global_conversion_factor is not None and self.all_trays is not None:
            if self.side_canvas:
                self.conversion_factor = self.global_conversion_factor * 3 / 8
            else:
                self.conversion_factor = copy(self.global_conversion_factor)
            for tray in self.all_trays:
                self.draw_tray(tray)
        if not once:
            self.after(500, self.update_canvas)
    
    def round_shape(self, points: list[float], radius: float = 0.03):
        rounded_points = []
        for i in range(len(points)):
            p_1 = points[i]
            p_2 = points[(i+1)%len(points)]
            angle = atan2((p_1[1] - p_2[1]), (p_2[0] - p_1[0]))
            rounded_points.append((p_1[0], p_1[1]))
            for _ in range(2):
                rounded_points.append((p_1[0] + radius * cos(angle), p_1[1] - radius * sin(angle)))

            for _ in range(2):
                rounded_points.append((p_2[0] - radius * cos(angle), p_2[1] + radius * sin(angle)))

        return rounded_points
    
    def translate_points(self, center: tuple[float, float], points):
        return [(p[0] + center[0], p[1] + center[1]) for p in points]
    
    def generate_tray_points(self, center: tuple[float, float], identifier: int, angle: float, round: bool = True):
        corners = TrayCanvas.tray_corners_[identifier]
        if round:
            corners = self.round_shape(corners)
        rotated_points = self.rotate_shape(corners, angle)
        # flipped_points = [(x, -y) for x, y in rotated_points]
        translated_points = self.translate_points(center, rotated_points)
        return translated_points

    def get_canvas_points(self, points: list[tuple[float, float]]) -> list[float]:
        canvas_points = []
        for x, y in points:
            canvas_points.append(x*self.conversion_factor)
            canvas_points.append(y*self.conversion_factor)
        return canvas_points

    def draw_tray(self, tray: Tray):
        if tray.identifier not in TrayCanvas.tray_corners_.keys():
            return

        # Gets the center point for the tray
        tray_x = tray.tray_pose.pose.position.x
        tray_y = tray.tray_pose.pose.position.y
        tray_rotation = self.get_tray_angle(tray.tray_pose.pose.orientation)
        
        points = self.generate_tray_points((tray_x, tray_y), tray.identifier, tray_rotation)
        canvas_points = self.get_canvas_points(points)
        tray_polygon = self.create_polygon(canvas_points, fill=TrayCanvas.tray_colors_[tray.identifier], smooth=True, outline="black", splinesteps=100)
        tooltip = CanvasTooltip(self, tray_polygon, text=tray.name)

        fiducial_points = self.generate_tray_points((tray_x, tray_y), -1, tray_rotation, False)
        canvas_points = self.get_canvas_points(fiducial_points)
        fiducial_polygon = self.create_polygon(canvas_points, fill="white", smooth=False, outline="black", splinesteps=100)
        tooltip = CanvasTooltip(self, fiducial_polygon, text=str(tray.name))
        
        for slot in tray.slots:
            slot: SlotInfo
            if not slot.occupied:
                continue

            slot_tray_rotation = tray_rotation - pi
            
            x_offset = slot.slot_pose.pose.position.x
            y_offset = -slot.slot_pose.pose.position.y

            new_x = x_offset * cos(slot_tray_rotation) - y_offset * sin(slot_tray_rotation)
            new_y = x_offset * sin(slot_tray_rotation) + y_offset * cos(slot_tray_rotation)

            translated_x = new_x + tray_x
            translated_y = new_y + tray_y

            canvas_x = translated_x * self.conversion_factor
            canvas_y = translated_y * self.conversion_factor

            self.draw_gear(canvas_x, canvas_y, TrayCanvas.gear_radii_[slot.size] * self.conversion_factor, slot.name)

    
    def draw_gear(self, c_x, c_y, radius, slot_name):
        gear_drawing = self.create_oval(c_x - radius, c_y - radius, c_x + radius, c_y + radius, fill="#40bd42")
        tooltip = CanvasTooltip(self, gear_drawing, text=slot_name)

    def get_tray_angle(self, q):
        R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        return R.GetRPY()[-1] + pi

    def rotate_shape(self, points: list[tuple[float, float]], angle: float):
        rotated_points = []
        for x,y in points:
            new_x = x * cos(angle) - y * sin(angle)
            new_y = x * sin(angle) + y * cos(angle)
            rotated_points.append((new_x, new_y))
        return rotated_points

class RobotStatusFrame(ctk.CTkFrame):
    def __init__(self, frame, robot_name: str):
        super().__init__(frame, width = 200, height=950, fg_color="#EBEBEB")
        self.grid_rowconfigure((0, 8), weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.robot_ = robot_name
        self.most_recent_time: Optional[float] = None
        self.threshold = 3.0

        ctk.CTkLabel(self, text=f"{self.robot_.capitalize()} status:").grid(column = 0, row = 1)
        self.robot_status_label = ctk.CTkLabel(self, text="Not Connected", text_color="red")
        self.robot_status_label.grid(column = 0, row = 2)

        ctk.CTkLabel(self, text=f"Active controllers:").grid(column = 0, row = 3)
        self.active_controllers_label = ctk.CTkLabel(self, text="No controllers active", text_color="red")
        self.active_controllers_label.grid(column = 0, row = 4)

        ctk.CTkLabel(self, text=f"Inactive controllers:").grid(column = 0, row = 5)
        self.inactive_controllers_label = ctk.CTkLabel(self, text="No controllers inactive", text_color="green")
        self.inactive_controllers_label.grid(column = 0, row = 6)
        ctk.CTkLabel(self, text=f" " * 40).grid(column = 0, row = 7) # Spacing so the window doesn't jump during update
    
    def escape_ansi(self, line): # https://stackoverflow.com/questions/14693701/how-can-i-remove-the-ansi-escape-sequences-from-a-string-in-python
        ansi_escape = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
        return ansi_escape.sub('', line)
    
    def update_status(self):
        if self.most_recent_time is not None:
            if time() - self.most_recent_time <= self.threshold:
                self.robot_status_label.configure(text="Connected", text_color="green")
            else:
                self.robot_status_label.configure(text="Not Connected", text_color="red")
                return

            try:
                result = subprocess.run(f"ros2 control list_controllers -c /{self.robot_}/controller_manager", timeout=0.1, stdout=subprocess.PIPE, shell=True)
                controllers_output = result.stdout.decode("utf-8")
            except subprocess.TimeoutExpired:
                return
            
            if "waiting" not in controllers_output:
                active = []
                inactive = []
                for line in controllers_output.split("\n"):
                    l = self.escape_ansi(line)
                    if "inactive" in l:
                        inactive.append(l.split(" ")[0])
                    else:
                        active.append(l.split(" ")[0])
                

                if len(active) > 0:
                    self.active_controllers_label.configure(text="\n".join(active), text_color = "green")
                else:
                    self.active_controllers_label.configure(text="No controllers active", text_color="red")
                
                if len(inactive) > 0:
                    self.inactive_controllers_label.configure(text="\n".join(inactive), text_color = "red")
                else:
                    self.inactive_controllers_label.configure(text="No controllers active", text_color="green")
            else: 
                print(f"Not working for {self.robot_}")

class ServicesFrame(ctk.CTkFrame):
    robots_ = ["fanuc", "motoman"]
    def __init__(self, frame, node: Node):
        super().__init__(frame, width = 1200, height=950, fg_color="#EBEBEB")
        self.grid_rowconfigure((0), weight=1)
        self.grid_columnconfigure((0, 1), weight=1)

        self.node = node

        self.named_positions = {robot: self.get_named_positions(robot) for robot in ServicesFrame.robots_}

        self.service_clients = {robot: {"move_to_named_pose": self.node.create_client(MoveToNamedPose, f"/{robot}/move_to_named_pose"),
                              "pick_from_slot": self.node.create_client(Pick, f"/{robot}/pick_from_slot"),
                              "place_in_slot": self.node.create_client(Place, f"/{robot}/place_in_slot"),
                              "actuate_gripper": self.node.create_client(PneumaticGripperControl, f"/{robot}/actuate_gripper")}
                              for robot in ServicesFrame.robots_}

        self.robot_selection_frame = ctk.CTkFrame(self, width = 300, height=900, fg_color="#EBEBEB")
        self.selected_robot = ctk.StringVar(value=ServicesFrame.robots_[0])
        self.add_robot_selection_widgets()

        self.occupied_slots = {robot: [] for robot in ServicesFrame.robots_}
        self.unoccupied_slots = {robot: [] for robot in ServicesFrame.robots_}
        
        self.service_notebook = ttk.Notebook(self)

        self.move_to_named_pose_frame = ctk.CTkFrame(self.service_notebook, width = 700, height=900, fg_color="#EBEBEB")
        self.move_to_named_pose_frame.pack(fill='both', expand=True)
        self.move_to_named_pose_frame.pack_propagate(0)
        self.selected_named_pose = ctk.StringVar()
        self.service_notebook.add(self.move_to_named_pose_frame, text="Move to Named Pose")
        self.add_move_to_named_pose_widgets_to_frame()

        self.pick_frame = ctk.CTkFrame(self.service_notebook, width = 700, height=900, fg_color="#EBEBEB")
        self.pick_frame.pack(fill='both', expand=True)
        self.pick_frame.pack_propagate(0)
        self.pick_frame_selection = ctk.StringVar()
        self.service_notebook.add(self.pick_frame, text="Pick")
        self.add_pick_widgets_to_frame()

        self.place_frame = ctk.CTkFrame(self.service_notebook, width = 700, height=900, fg_color="#EBEBEB")
        self.place_frame.pack(fill='both', expand=True)
        self.place_frame.pack_propagate(0)
        self.place_frame_selection = ctk.StringVar()
        self.service_notebook.add(self.place_frame, text="Place")
        self.add_place_widgets_to_frame()

        self.gripper_frame = ctk.CTkFrame(self.service_notebook, width = 700, height=900, fg_color="#EBEBEB")
        self.gripper_frame.pack(fill='both', expand=True)
        self.gripper_frame.pack_propagate(0)
        self.service_notebook.add(self.gripper_frame, text="Actuate_gripper")
        self.add_gripper_widgets_to_frame()

        self.service_notebook.grid(column=0, row=0, padx=20)

        self.robot_selection_frame.grid(column=1, row=0, padx=20)

        self.held_gear: Optional[str] = None

        self.available_matching_slots: list[str] = []

    def add_robot_selection_widgets(self):
        ctk.CTkLabel(self.robot_selection_frame, text="Select the robot for the service:").pack(pady=25)

        radio_buttons = []
        for robot in ServicesFrame.robots_:
            radio_buttons.append(ctk.CTkRadioButton(self.robot_selection_frame, text=robot, variable=self.selected_robot, value=robot, command=self.reload_services_frames))
            radio_buttons[-1].pack(pady=5)

    def reload_services_frames(self):
        for frame in [self.move_to_named_pose_frame, self.pick_frame, self.place_frame]:
            for widget in frame.winfo_children():
                widget.pack_forget()
        
        self.add_move_to_named_pose_widgets_to_frame()
        self.add_pick_widgets_to_frame()
        self.add_place_widgets_to_frame()
    
    def reload_pick_and_place(self):
        for frame in [self.pick_frame, self.place_frame]:
            for widget in frame.winfo_children():
                widget.pack_forget()
        
        self.add_pick_widgets_to_frame()
        self.add_place_widgets_to_frame()

    # ==============================================================
    #                     Move to named pose
    # ==============================================================
    
    def add_move_to_named_pose_widgets_to_frame(self):
        if len(self.named_positions[self.selected_robot.get()]) == 0:
            ctk.CTkLabel(self.move_to_named_pose_frame, text="No named poses found for " + self.selected_robot.get()).pack(pady=10)
        else:
            self.selected_named_pose.set(self.named_positions[self.selected_robot.get()][0])
            ctk.CTkLabel(self.move_to_named_pose_frame, text="Select the pose to move to:").pack(pady=10)

            ctk.CTkOptionMenu(self.move_to_named_pose_frame, variable=self.selected_named_pose, values = self.named_positions[self.selected_robot.get()]).pack(pady=20)

            ctk.CTkButton(self.move_to_named_pose_frame, text="Call Service", command=self.call_move_to_named_pose_service_thread).pack(pady=10)

    def call_move_to_named_pose_service_thread(self):
        thread = threading.Thread(target=self.call_move_to_named_pose_service_)
        thread.start()
    
    def call_move_to_named_pose_service_(self):
        move_to_named_pose_request = MoveToNamedPose.Request()
        move_to_named_pose_request.name = self.selected_named_pose.get()

        future = self.service_clients[self.selected_robot.get()]["move_to_named_pose"].call_async(move_to_named_pose_request)
        future.add_done_callback(self.move_to_names_pose_response_cb)
    
    def move_to_names_pose_response_cb(self, future: Future):
        try:
            response = future.result()
            if not response.success:
                self.node.get_logger().warn(f"Failed to move to {self.selected_named_pose.get()} with {self.selected_robot.get()}")
        except Exception as e:
            self.node.get_logger().error("Pick service call ailed: %r" % (e,))
    
    # ==============================================================
    #                            Pick
    # ==============================================================
    
    def add_pick_widgets_to_frame(self):
        if len(self.occupied_slots[self.selected_robot.get()]) == 0:
            ctk.CTkLabel(self.pick_frame, text="No occupied slots found for " + self.selected_robot.get()).pack(pady=10)
        elif self.held_gear is not None:
            ctk.CTkLabel(self.pick_frame, text= self.selected_robot.get() + " is currently holding a gear. Please place it to pick another.").pack(pady=10)
        else:
            self.pick_frame_selection.set("")
            ctk.CTkLabel(self.pick_frame, text="Select the frame for picking:").pack(pady=10)

            self.pick_frame_menu = ctk.CTkComboBox(self.pick_frame, variable=self.pick_frame_selection, values=self.occupied_slots[self.selected_robot.get()])
            self.pick_frame_menu.pack(pady=10)

            ctk.CTkButton(self.pick_frame, text="Call Service", command=self.call_pick_service_thread).pack(pady=10)

            self.pick_frame_selection.trace_add('write', partial(self.only_show_matching_frames, self.pick_frame_selection, self.pick_frame_menu, self.occupied_slots[self.selected_robot.get()]))

    def call_pick_service_thread(self):
        thread = threading.Thread(target=self.call_pick_service)
        thread.start()
    
    def call_pick_service(self):
        pick_request = Pick.Request()
        pick_request.frame_name = self.pick_frame_selection.get()

        future = self.service_clients[self.selected_robot.get()]["pick_from_slot"].call_async(pick_request)
        future.add_done_callback(self.pick_response_cb)
    
    def pick_response_cb(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.occupied_slots[self.selected_robot.get()].remove(self.pick_frame_selection.get())
                self.unoccupied_slots[self.selected_robot.get()].append(self.pick_frame_selection.get())

                if "small" in self.pick_frame_selection.get() or "sg" in self.pick_frame_selection.get():
                    self.held_gear = ["small", "sg"]
                elif "medium" in self.pick_frame_selection.get() or "mg" in self.pick_frame_selection.get():
                    self.held_gear = ["medium", "mg"]
                else:
                    self.held_gear = ["large", "lg"]
                self.reload_pick_and_place()
            else:
                self.node.get_logger().warn(f"Failed to pick frame {self.pick_frame_selection.get()} with {self.selected_robot.get()}")
        except Exception as e:
            self.node.get_logger().error("Pick service call ailed: %r" % (e,))
    # ==============================================================
    #                            Place
    # ==============================================================
    
    def add_place_widgets_to_frame(self):
        if len(self.unoccupied_slots[self.selected_robot.get()]) == 0:
            ctk.CTkLabel(self.place_frame, text="No unoccupied slots found for " + self.selected_robot.get()).pack(pady=10)
        elif self.held_gear is None:
            ctk.CTkLabel(self.place_frame, text="A gear must be picked by " + self.selected_robot.get() + " to place. Please pick a gear").pack(pady=10)
        else:
            # self.available_matching_slots = self.unoccupied_slots[self.selected_robot.get()]
            self.available_matching_slots = []
            for slot in self.unoccupied_slots[self.selected_robot()]:
                for i in self.held_gear:
                    if i in slot:
                        self.available_matching_slots.append(i)
            self.place_frame_selection.set("")
            ctk.CTkLabel(self.place_frame, text="Select the frame for placing:").pack(pady=10)

            self.place_frame_menu = ctk.CTkComboBox(self.place_frame, variable=self.place_frame_selection, values=self.available_matching_slots)
            self.place_frame_menu.pack(pady=10)

            ctk.CTkButton(self.place_frame, text="Call Service", command=self.call_place_service_thread).pack(pady=10)

            self.place_frame_selection.trace_add('write', partial(self.only_show_matching_frames, self.place_frame_selection, self.place_frame_menu, self.unoccupied_slots[self.selected_robot.get()]))

    def call_place_service_thread(self):
        thread = threading.Thread(target=self.call_place_service)
        thread.start()
    
    def call_place_service(self):
        place_request = Place.Request()
        place_request.frame_name = self.place_frame_selection.get()

        future = self.service_clients[self.selected_robot.get()]["place_in_slot"].call_async(place_request)
        future.add_done_callback(self.place_response_cb)
    
    def place_response_cb(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.occupied_slots[self.selected_robot.get()].append(self.place_frame_selection.get())
                self.unoccupied_slots[self.selected_robot.get()].remove(self.place_frame_selection.get())

                self.held_gear = None
                self.reload_pick_and_place()
            else:
                self.node.get_logger().warn(f"Failed to place frame {self.place_frame_selection.get()} with {self.selected_robot.get()}")
        except Exception as e:
            self.node.get_logger().error("Place service call failed: %r" % (e,))
            
    # ==============================================================
    #                          Gripper
    # ==============================================================
    
    def add_gripper_widgets_to_frame(self):

        ctk.CTkButton(self.gripper_frame, text="Open gripper", command=partial(self.acuate_gripper_service, False)).pack(pady=25)
        ctk.CTkButton(self.gripper_frame, text="Close gripper", command=partial(self.acuate_gripper_service, True)).pack(pady=25)
    
    def acuate_gripper_service(self, enable: bool):
        gripper_request = PneumaticGripperControl.Request()
        gripper_request.enable = enable

        future = self.service_clients[self.selected_robot.get()]["actuate_gripper"].call_async(gripper_request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.node.get_logger().warn(f"Unable to {'close' if enable else 'open'} {self.selected_robot.get()} gripper")
                return

    def get_named_positions(self, robot_name: str):
        moveit_package = get_package_share_directory(f'{robot_name}_moveit_config')
        srdf_file_path = moveit_package + f"/config/{robot_name}.srdf"
        found_named_positions = []
        with open(srdf_file_path, "+r") as f:
            for line in f:
                if "group_state" in line and " name" in line:
                    found_name = ""
                    inside_quotes = False
                    for c in line[line.find(" name"):]:
                        if c == '"':
                            if inside_quotes == False:
                                inside_quotes = True
                                continue
                            else:
                                break
                        if inside_quotes:
                            found_name += c
                    found_named_positions.append(found_name)
        return found_named_positions
    
    def only_show_matching_frames(self, frame_var, frame_menu, frames, _, __, ___):
        pass
        selection = frame_var.get()

        if selection in frames:
            frame_menu.configure(values = list(set(frames)))
        else:
            options = []
            for topic in frames:
                if selection.lower() in topic.lower():
                    options.append(topic)
                else:
                    for i in range(len(topic)-len(selection)):
                        if SequenceMatcher(None, selection.lower(), topic[i:i+len(selection)].lower()).ratio() > 0.8:
                            options.append(topic)
            frame_menu.configure(values = list(set(options)))

class PDDLFrame(ctk.CTkFrame):
    def __init__(self, frame, node: Node):
        super().__init__(frame, width = 1200, height=950, fg_color="#EBEBEB")

        self.node = node
        self.service_clients = {"generate_plan": self.node.create_client(GeneratePlan, f"/generate_pddl_plan")}
        
        self.execute_plan_client = ActionClient(self.node, ExecutePlan, "/execute_pddl_plan")

        self.generate_plan_button = ctk.CTkButton(self, text="Generate Plan", command=self.call_generate_plan_service, state=tk.NORMAL)
        self.generate_plan_button.pack()

        self.plan_scrollable_frame = ctk.CTkScrollableFrame(self, width=900, height=450)
        self.plan_scrollable_frame.pack()
        self.scrollable_label = ctk.CTkLabel(self.plan_scrollable_frame, text="")
        self.scrollable_label.pack()

        self.execute_plan_button = ctk.CTkButton(self, text="Execute Plan", command=self.call_execute_plan_action, state=tk.DISABLED)
        self.execute_plan_button.pack()

        self.plan: Optional[Plan] = None

    def call_generate_plan_service(self):
        generate_plan_request = GeneratePlan.Request()

        future = self.service_clients["generate_plan"].call_async(generate_plan_request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.node.get_logger().warn(f"Unable to generate plan (timeout)")
                return
        if future.result().success:
            self.plan = future.result().plan
            self.generate_plan_button.configure(state=tk.DISABLED)
            self.scrollable_label.configure(text="\n".join([item.action for item in future.result().plan.items]))
            self.execute_plan_button.configure(state=tk.NORMAL)
        else:
            self.node.get_logger().warn(f"Unable to generate plan (service request=False)")
            return
    
    def call_execute_plan_action(self):
        self.execute_plan_client.wait_for_server()
        self.node.get_logger().info("Executing plan")

        goal = ExecutePlan.Goal()
        goal.plan = self.plan

        try:
            future = self.execute_plan_client.send_goal_async(goal)
            
            while not self.node.goal_finished:
                spin_once(self.node)
                sleep(0.1)
            
            self.execute_plan_button.configure(state=tk.DISABLED)
            self.scrollable_label.configure(text="")
            self.generate_plan_button.configure(state=tk.NORMAL)
        except:
            self.node.get_logger().info("Could not execute plan")