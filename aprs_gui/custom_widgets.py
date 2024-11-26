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
from rclpy.node import Node
from difflib import SequenceMatcher
import yaml
from functools import partial

from aprs_interfaces.msg import Tray, SlotInfo
from aprs_interfaces.srv import LocateTrays, Pick, Place, MoveToNamedPose

from geometry_msgs.msg import Transform

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
    
    # gear_offsets_ = {Tray.SMALL_GEAR_TRAY: {
    #                     'slot_1': (-0.045, 0.045),
    #                     'slot_2': (0.045, 0.045),
    #                     'slot_3': (-0.045, -0.045),
    #                     'slot_4': (0.045, -0.045),
    #                 },
    #                 Tray.MEDIUM_GEAR_TRAY: {
    #                     'slot_1': (-0.050, 0.050),
    #                     'slot_2': (0.050, 0.050),
    #                     'slot_3': (-0.050, -0.050),
    #                     'slot_4': (0.050, -0.050),
    #                 },
    #                 Tray.LARGE_GEAR_TRAY: {
    #                     'slot_1': (-0.052, -0.06),
    #                     'slot_2': (0.052, -0.06),
    #                 },
    #                 Tray.M2L1_KIT_TRAY: {
    #                     'lg_1': (0.0, -0.075),
    #                     'mg_1': (-0.065, 0.0),
    #                     'mg_2': (0.065, 0.0),
    #                 },
    #                 Tray.S2L2_KIT_TRAY: {
    #                     'lg_1': (-0.052, -0.060),
    #                     'lg_2': (0.052, -0.060),
    #                     'sg_1': (-0.045, 0.045),
    #                     'sg_2': (0.045, 0.045),
    #                 }}
    
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
        self.update_canvas()
    
    def update_canvas(self):
        self.delete("all")
        # if self.side_canvas:
        #     try:
        #         self.configure(height=150, width=self.width * 3 / 8)
        #     except:
        #         self.configure(height=150, width=150)
        # else:
        self.configure(height=400, width=self.width)
        if self.conversion_factor is not None and self.all_trays is not None:
            # if self.side_canvas:
            #     self.conversion_factor = self.global_conversion_factor * 3 / 8
            for tray in self.all_trays:
                self.draw_tray(tray)
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
        self.create_polygon(canvas_points, fill=TrayCanvas.tray_colors_[tray.identifier], smooth=True, outline="black", splinesteps=100)

        fiducial_points = self.generate_tray_points((tray_x, tray_y), -1, tray_rotation, False)
        canvas_points = self.get_canvas_points(fiducial_points)
        self.create_polygon(canvas_points, fill="white", smooth=False, outline="black", splinesteps=100)
        
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

            self.draw_circle(canvas_x, canvas_y, TrayCanvas.gear_radii_[slot.size] * self.conversion_factor)
    
    def draw_circle(self, c_x, c_y, radius):
        self.create_oval(c_x - radius, c_y - radius, c_x + radius, c_y + radius, fill="#40bd42")

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
                              "place_in_slot": self.node.create_client(Place, f"/{robot}/place_in_slot")}
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

        self.service_notebook.grid(column=0, row=0, padx=20)

        self.robot_selection_frame.grid(column=1, row=0, padx=20)

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

            ctk.CTkButton(self.move_to_named_pose_frame, text="Call Service", command=self.call_move_to_named_pose_service_).pack(pady=10)

    def call_move_to_named_pose_service_(self):
        move_to_named_pose_request = MoveToNamedPose.Request()
        move_to_named_pose_request.name = self.selected_named_pose.get()

        future = self.service_clients[self.selected_robot.get()]["move_to_named_pose"].call_async(move_to_named_pose_request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 15.0:
                self.node.get_logger().warn(f"Unable to move {self.selected_robot.get()} to desired pose")
                return
    
    # ==============================================================
    #                            Pick
    # ==============================================================
    
    def add_pick_widgets_to_frame(self):
        if len(self.occupied_slots[self.selected_robot.get()]) == 0:
            ctk.CTkLabel(self.pick_frame, text="No occupied slots found for " + self.selected_robot.get()).pack(pady=10)
        else:
            self.pick_frame_selection.set("")
            ctk.CTkLabel(self.pick_frame, text="Select the frame for picking:").pack(pady=10)

            self.pick_frame_menu = ctk.CTkComboBox(self.pick_frame, variable=self.pick_frame_selection, values=self.occupied_slots[self.selected_robot.get()])
            self.pick_frame_menu.pack(pady=10)

            ctk.CTkButton(self.pick_frame, text="Call Service", command=self.call_pick_service).pack(pady=10)

            self.pick_frame_selection.trace_add('write', partial(self.only_show_matching_frames, self.pick_frame_selection, self.pick_frame_menu, self.occupied_slots[self.selected_robot.get()]))

    def call_pick_service(self):
        pick_request = Pick.Request()
        pick_request.frame_name = self.pick_frame_selection.get()

        future = self.service_clients[self.selected_robot.get()]["pick_from_slot"].call_async(pick_request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 15.0:
                self.node.get_logger().warn(f"Unable to pick frame {self.pick_frame_selection} with {self.selected_robot.get()}")
                return
    
    # ==============================================================
    #                            Place
    # ==============================================================
    
    def add_place_widgets_to_frame(self):
        if len(self.unoccupied_slots[self.selected_robot.get()]) == 0:
            ctk.CTkLabel(self.place_frame, text="No unoccupied slots found for " + self.selected_robot.get()).pack(pady=10)
        else:
            self.place_frame_selection.set("")
            ctk.CTkLabel(self.place_frame, text="Select the frame for placing:").pack(pady=10)

            self.place_frame_menu = ctk.CTkComboBox(self.place_frame, variable=self.place_frame_selection, values=self.unoccupied_slots[self.selected_robot.get()])
            self.place_frame_menu.pack(pady=10)

            ctk.CTkButton(self.place_frame, text="Call Service", command=self.call_place_service).pack(pady=10)

            self.place_frame_selection.trace_add('write', partial(self.only_show_matching_frames, self.place_frame_selection, self.place_frame_menu, self.unoccupied_slots[self.selected_robot.get()]))

    def call_place_service(self):
        place_request = Place.Request()
        place_request.frame_name = self.place_frame_selection.get()

        future = self.service_clients[self.selected_robot.get()]["place_in_slot"].call_async(place_request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 15.0:
                self.node.get_logger().warn(f"Unable to place frame {self.place_frame_selection} with {self.selected_robot.get()}")
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
            frame_menu.configure(values = frames)
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