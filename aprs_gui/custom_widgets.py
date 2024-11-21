import PyKDL
import subprocess
import customtkinter as ctk
import tkinter as tk
from typing import Optional
from copy import copy
from math import sin, cos, atan2
from time import time
import re

from aprs_interfaces.msg import Tray, SlotInfo

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
        self.after(500, self.update_image)

class TrayCanvas(tk.Canvas):
    tray_points_ = {Tray.SMALL_GEAR_TRAY: [
        (-0.08, -0.08),
        (-0.08, 0.08),
        (0.08, 0.08),
        (0.08, -0.08)
    ],
    Tray.MEDIUM_GEAR_TRAY: [
        (-0.098, -0.098),
        (-0.098, 0.098),
        (0.098, 0.098),
        (0.098, -0.098)
    ],
    Tray.LARGE_GEAR_TRAY: [
        (-0.105, -0.113),
        (0.105, -0.113),
        (0.105, 0.025),
        (-0.105, 0.025),
    ],
    Tray.M2L1_KIT_TRAY: [
        (-0.108, -0.062), # Top left corner
        (0.108, -0.062), # Top right corner
        (0.108, 0.05225), # Bottom right corner
        (0.019, 0.128), # Very bottom right corner
        (-0.019, 0.128), # Very bottom left corner
        (-0.108, 0.05225), # Bottom left corner
    ],
    Tray.S2L2_KIT_TRAY: [
        (-0.105, -0.113), # Top left corner
        (0.105, -0.113), # Top right corner
        (0.105, 0.0), # Middle right coner
        (0.06638, 0.08), # Bottom right corner
        (-0.06638,  0.08), # Bottom left corner
        (-0.105, 0.0), # Middle left coner
    ],
    -1: [
        (-0.02, -0.02),
        (-0.02, 0.02),
        (0.02, 0.02),
        (0.02, -0.02)
    ]}

    gear_radii_ = {SlotInfo.SMALL: 0.032,
                   SlotInfo.MEDIUM: 0.04,
                   SlotInfo.LARGE: 0.05}
    
    gear_offsets_ = {Tray.SMALL_GEAR_TRAY: {
                        'slot_1': (-0.045, 0.045),
                        'slot_2': (0.045, 0.045),
                        'slot_3': (-0.045, -0.045),
                        'slot_4': (0.045, -0.045),
                    },
                    Tray.MEDIUM_GEAR_TRAY: {
                        'slot_1': (-0.050, 0.050),
                        'slot_2': (0.050, 0.050),
                        'slot_3': (-0.050, -0.050),
                        'slot_4': (0.050, -0.050),
                    },
                    Tray.LARGE_GEAR_TRAY: {
                        'slot_1': (-0.052, -0.06),
                        'slot_2': (0.052, -0.06),
                    },
                    Tray.M2L1_KIT_TRAY: {
                        'lg_1': (0.0, -0.075),
                        'mg_1': (-0.065, 0.0),
                        'mg_2': (0.065, 0.0),
                    },
                    Tray.S2L2_KIT_TRAY: {
                        'lg_1': (-0.052, -0.060),
                        'lg_2': (0.052, -0.060),
                        'sg_1': (-0.045, 0.045),
                        'sg_2': (0.045, 0.045),
                    }}
    
    tray_colors_ = {
        Tray.SMALL_GEAR_TRAY: "red",
        Tray.MEDIUM_GEAR_TRAY: "green",
        Tray.LARGE_GEAR_TRAY: "blue",
        Tray.M2L1_KIT_TRAY: "black",
        Tray.S2L2_KIT_TRAY: "yellow"
    }

    fiducial_square_measurements = (0.04, 0.04)
    
    def __init__(self, frame):
        super().__init__(frame, height=150, width=150, bd = 0, highlightthickness=0)
        self.global_conversion_factor: Optional[float] = 684.6970215679562
        self.conversion_factor = 684.6970215679562
        self.trays_info_recieved = False
        self.tray_tranforms: Optional[list[Transform]] = None
        self.all_trays: Optional[list[Tray]] = None
        self.width: Optional[int] = None
        self.side_canvas = False
    
    def update_canvas(self):
        self.delete("all")
        if self.side_canvas:
            try:
                self.configure(height=150, width=self.width * 3 / 8)
            except:
                self.configure(height=150, width=150)
        else:
            self.configure(height=400, width=400)
        if self.conversion_factor is not None and self.trays_info_recieved:
            if self.side_canvas:
                self.conversion_factor = self.global_conversion_factor * 3 / 8
            else:
                self.conversion_factor = copy(self.global_conversion_factor)
            for tray in self.all_trays:
                self.draw_tray(tray)
    
    def round_shape(self, points: list[float], radius: float = 0.03):
        rounded_points = []
        for i in range(len(points)):
            p_1 = points[i]
            p_2 = points[(i+1)%len(points)]
            angle = atan2((p_1[1] - p_2[1]), (p_2[0] - p_1[0]))
            rounded_points.append(p_1[0])
            rounded_points.append(p_1[1])
            for _ in range(2):
                rounded_points.append(p_1[0] + radius * cos(angle))
                rounded_points.append(p_1[1] - radius * sin(angle))

            for _ in range(2):
                rounded_points.append(p_2[0] - radius * cos(angle))
                rounded_points.append(p_2[1] + radius * sin(angle))

        return rounded_points
    
    def generate_tray_points(self, center: tuple[float, float], identifier: int, angle: float, round: bool = True):
        points=TrayCanvas.tray_points_[identifier]
        if round:
            points = self.round_shape(points)
        else:
            p = []
            for point in points:
                p.extend(list(point))
            points = p
        points = [points[i] + center[i%2] for i in range(len(points))]
        points = [p * self.conversion_factor for p in points]
        self.rotate_shape((center[0] * self.conversion_factor, center[1] * self.conversion_factor), points, angle)
        return points

    def draw_tray(self, tray: Tray):
        if tray.identifier not in TrayCanvas.tray_points_.keys():
            return

        # Gets the center point for the tray
        c_x = tray.transform_stamped.transform.translation.x
        c_y = tray.transform_stamped.transform.translation.y
        
        points = self.generate_tray_points((c_x, c_y), tray.identifier, self.get_tray_angle(tray.transform_stamped.transform.rotation))
        self.create_polygon(points, fill=TrayCanvas.tray_colors_[tray.identifier], smooth=True)

        fiducial_points = self.generate_tray_points((c_x, c_y), -1, self.get_tray_angle(tray.transform_stamped.transform.rotation), False)
        self.create_polygon(fiducial_points, fill="white", smooth=False)

        for slot in tray.slots:
            if slot.occupied:
                x_coord = (c_x + TrayCanvas.gear_offsets_[tray.identifier][slot.name][0]) * self.conversion_factor
                y_coord = (c_y + TrayCanvas.gear_offsets_[tray.identifier][slot.name][1]) * self.conversion_factor
                slot_coords = [x_coord, y_coord]
                self.rotate_shape((c_x * self.conversion_factor, c_y * self.conversion_factor), slot_coords, self.get_tray_angle(tray.transform_stamped.transform.rotation))
                self.draw_circle(slot_coords[0], slot_coords[1], TrayCanvas.gear_radii_[slot.size] * self.conversion_factor)
    
    def draw_circle(self, c_x, c_y, radius):
        self.create_oval(c_x - radius, c_y - radius, c_x + radius, c_y + radius, fill="yellow")

    def get_tray_angle(self, q):
        R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        return R.GetRPY()[-1]

    def rotate_shape(self, tray_center: tuple[int, int], points, angle: float):
        for i in range(0,len(points),2):
            original_x = points[i] - tray_center[0]
            original_y = points[i+1] - tray_center[1]
            points[i] = int((original_x * cos(angle) + original_y * sin(angle))) + tray_center[0]
            points[i+1] = int((-1 * original_x * sin(angle) + original_y * cos(angle))) + tray_center[1]

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