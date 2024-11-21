import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk, END
import tkinter as tk

from PIL import Image
import PyKDL
from time import time
from math import sin, cos, atan, pi
from functools import partial
from copy import copy
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from typing import Optional
from numpy import ndarray
from aprs_interfaces.msg import Trays, Tray, SlotInfo
from aprs_vision.slot_offsets import SlotOffsets
from geometry_msgs.msg import Transform, Quaternion

from aprs_interfaces.srv import LocateTrays

def deg_to_rad(deg: float):
        return deg * pi / 180

class GuiClass(Node):
    vision_systems_ = ["fanuc_vision", "motoman_vision", "teach_table_vision", "fanuc_conveyor", "motoman_conveyor"]
    service_headers_ = ["/fanuc/table_vision", "/motoman/table_vision", "/teach/table_vision", "/fanuc/conveyor_vision", "/motoman/conveyor_vision"]
    def __init__(self):
        super().__init__("test_gui")
        
        self.main_wind = ctk.CTk()
        ctk.set_appearance_mode("light")
        self.main_wind.geometry("1300x1000")
        # self.main_wind.resizable(False, False)

        self.img_max_height = 400
        self.main_wind.grid_rowconfigure([i for i in range(8)], weight=1)
        self.main_wind.grid_columnconfigure((0,1,2), weight=1)

        self.bridge = CvBridge()

        # Locate Trays widgets
        self.locate_trays_frame = ctk.CTkFrame(self.main_wind, 200, 800, fg_color="#EBEBEB")
        self.locate_trays_vars = {vision_system: ctk.StringVar(value="0") for vision_system in GuiClass.vision_systems_}
        self.locate_trays_cbs_: dict[str: ctk.CTkCheckBox] = {}
        self.locate_trays_label = ctk.CTkLabel(self.locate_trays_frame, text="Select the vision systems to locate trays for")
        self.locate_trays_label.pack(pady=50)
        row = 1
        for vision_system in GuiClass.vision_systems_:
            self.locate_trays_cbs_[vision_system] = ctk.CTkCheckBox(self.locate_trays_frame,text=vision_system, variable=self.locate_trays_vars[vision_system], onvalue="1", offvalue="0", height=1, width=20)
            self.locate_trays_cbs_[vision_system].pack(pady=30, ipadx=15, expand=True)
            row += 1
        self.locate_trays_button = ctk.CTkButton(self.locate_trays_frame, text="Locate trays", command=self.locate_trays)
        self.locate_trays_button.pack(pady=50)
        self.locate_trays_frame.grid(column = 0, row = 2, rowspan=6, padx=20)


        # Visualization menu widgets
        vision_selection_label = ctk.CTkLabel(self.main_wind, text="Select the vision system for the live view")
        vision_selection_label.grid(column = 1, row = 0, pady=1, sticky="ew")
        self.vision_selection = ctk.StringVar(value=GuiClass.vision_systems_[0])
        self.vision_selection_menu = ctk.CTkOptionMenu(self.main_wind, variable=self.vision_selection, values=GuiClass.vision_systems_)
        self.vision_selection_menu.grid(column = 1, row = 1, pady = 1, sticky="ew")

        self.center_frame = ctk.CTkFrame(self.main_wind, 600, 900, fg_color="#EBEBEB")
        self.live_image_label = LiveImage(self.center_frame)
        self.live_image_label.pack(pady=1, padx=20)
        self.center_frame.grid(column = 1, row = 2, rowspan=6, padx=20)

        # Subcanvas frame
        self.visualization_frame = ctk.CTkFrame(self.main_wind, 400, 900, fg_color="#EBEBEB")
        self.visualization_frame.grid(column = 2, row = 2, rowspan=8, padx=20)

        self.visualization_canvases = {vision_system: TrayCanvas(self.main_wind) for vision_system in GuiClass.vision_systems_}
        for vision_system in GuiClass.vision_systems_:
            self.visualization_canvases[vision_system].bind('<Button-1>', partial(self.vis_clicked, vision_system))
        self.visualization_labels: list[ctk.CTkLabel] = []
        
        # Subscribers and clients
        self.image_subs = {}
        self.tray_subs = {}
        self.locate_clients = {}
        for i in range(len(GuiClass.vision_systems_)):
            self.image_subs[GuiClass.vision_systems_[i]] = self.create_subscription(
                ImageMsg, 
                f'{GuiClass.service_headers_[i]}/raw_image',
                partial(self.image_cb, GuiClass.vision_systems_[i]),
                qos_profile_default
            )

            self.tray_subs[GuiClass.vision_systems_[i]] = self.create_subscription(
                Trays,
                f'{GuiClass.service_headers_[i]}/trays_info',
                partial(self.trays_cb_, GuiClass.vision_systems_[i]),
                qos_profile_default
            )

            self.locate_clients[GuiClass.vision_systems_[i]] = self.create_client(
                LocateTrays,
                f'{GuiClass.service_headers_[i]}/trays_info'
            )
        self.most_recent_imgs: dict[str: Optional[ctk.CTkImage]] = {vision_system: None for vision_system in GuiClass.vision_systems_}
        
        self.show_all_canvases(1,1,1)
        self.vision_selection.trace_add("write", self.show_all_canvases)

    def image_cb(self, vision_system: str, msg: ImageMsg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        width = int(cv_image.shape[1] * self.img_max_height / cv_image.shape[0])
        
        self.most_recent_imgs[vision_system] = ctk.CTkImage(Image.fromarray(cv_image), size=(width, 400))
        if vision_system == self.vision_selection.get():
            self.live_image_label.current_image = self.live_image_label[vision_system]

        height_in_inches = cv_image.shape[0] / 30
        height_in_meters = height_in_inches * 0.0254
        self.visualization_canvases[vision_system].conversion_factor = 400 / height_in_meters
        self.visualization_canvases[vision_system].width = width


    def trays_cb_(self, vision_system: str, msg: Trays):
        self.get_logger().info(vision_system)
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        self.visualization_canvases[vision_system].trays_info_recieved = True
        self.visualization_canvases[vision_system].all_trays = all_trays
        self.visualization_canvases[vision_system].update_canvas()
    
    def locate_trays(self):
        c = 0
        for vision_system in GuiClass.vision_systems_:
            if self.locate_trays_vars[vision_system].get() == "1":
                request = LocateTrays.Request()
                future = self.locate_clients[vision_system].call_async(request)

                start = time()
                success = True
                while not future.done():
                    pass
                    if time()-start >= 1.0:
                        self.get_logger().warn(f"Unable to locate trays for {vision_system}.\n")
                        success = False
                        break
                if not success:
                    continue
                self.locate_trays_cbs_[vision_system].grid_forget()
                c+=1
        if c == len(GuiClass.vision_systems_):
            self.locate_trays_label.grid_forget()
            self.locate_trays_button.grid_forget()
    
    def show_all_canvases(self, _, __, ___):
        self.live_image_label.current_image = self.most_recent_imgs[self.vision_selection.get()]
        for canvas in self.visualization_canvases.values():
            canvas.grid_forget()
            canvas.pack_forget()

        for label in self.visualization_labels:
            label.grid_forget()
            label.pack_forget()
        self.visualization_labels.clear()
        
        self.visualization_canvases[self.vision_selection.get()].side_canvas = False
        self.visualization_canvases[self.vision_selection.get()].update_canvas()
        self.visualization_canvases[self.vision_selection.get()].pack(in_=self.center_frame, pady=20, padx=20)
        self.visualization_labels.append(ctk.CTkLabel(self.center_frame, text=self.vision_selection.get()))
        self.visualization_labels[-1].pack(pady=5, padx=20)

        current_row = 0
        for vision_system in GuiClass.vision_systems_:
            if vision_system != self.vision_selection.get():
                self.visualization_canvases[vision_system].side_canvas = True
                self.visualization_canvases[vision_system].update_canvas()
                self.visualization_canvases[vision_system].grid(in_=self.visualization_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1
                self.visualization_labels.append(ctk.CTkLabel(self.visualization_frame, text=vision_system))
                self.visualization_labels[-1].grid(in_=self.visualization_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1

    def vis_clicked(self, vision_system, event):
        self.vision_selection.set(vision_system)

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
        -0.08, -0.08,
        -0.08, 0.08,
        0.08, 0.08,
        0.08, -0.08
    ],
    Tray.MEDIUM_GEAR_TRAY: [
        -0.098, -0.098,
        -0.098, 0.098,
        0.098, 0.098,
        0.098, -0.098
    ],
    Tray.LARGE_GEAR_TRAY: [
        -0.105, -0.24,
        -0.105, 0.113,
        0.105, 0.113,
        0.105, -0.24
    ],
    Tray.M2L1_KIT_TRAY: [
        -0.108, -0.062, # Top left corner
        0.108, -0.062, # Top right corner
        0.108, 0.05225, # Bottom right corner
        0.019, 0.128, # Very bottom right corner
        -0.019, 0.128, # Very bottom left corner
        -0.108, 0.05225, # Bottom left corner
    ],
    Tray.S2L2_KIT_TRAY: [
        -0.105, -0.113, # Top left corner
        0.105, -0.113, # Top right corner
        0.105, 0.0, # Middle right coner
        0.06638, 0.08, # Bottom right corner
        -0.06638,  0.08, # Bottom left corner
        -0.105, 0.0, # Middle left coner
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
                        'slot_2': (0.052, 0.06),
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
    
    def __init__(self, frame):
        super().__init__(frame, height=150, width=200, bd = 0, highlightthickness=0)
        self.global_conversion_factor: Optional[float] = 684.6970215679562
        self.conversion_factor = 684.6970215679562
        self.trays_info_recieved = False
        self.tray_tranforms: Optional[list[Transform]] = None
        self.all_trays: Optional[list[Tray]] = None
        self.width: Optional[int] = None
        self.side_canvas = False
    
    def update_canvas(self):
        self.delete("all")
        if self.conversion_factor is not None and self.trays_info_recieved:
            if self.side_canvas:
                self.conversion_factor = self.global_conversion_factor * 3 / 8
                try:
                    self.configure(height=150, width=self.width * 3 / 8)
                except:
                    self.configure(height=150, width=150)
            else:
                self.conversion_factor = copy(self.global_conversion_factor)
                # self.configure(height=400, width=self.width)
                self.configure(height=400, width=400)
            for tray in self.all_trays:
                self.draw_tray(tray)
    
    def round_shape(self, points: list[float], radius: float):
        rounded_points = []
        for i in range(0, len(points), 2):
            p_1 = (points[i], points[i+1])
            p_2 = (points[(i+2)%len(points)], points[(i+3)%len(points)])
            try:
                angle = atan((p_2[1] - p_1[1])/(p_2[0] - p_1[0]))
            except:
                angle = pi/2
            rounded_points.append(p_1[0])
            rounded_points.append(p_1[1])
            for _ in range(2):
                if p_1[0] < p_2[0]:
                    rounded_points.append(p_1[0] + abs(radius * cos(angle)))
                else:
                    rounded_points.append(p_1[0] - abs(radius * cos(angle)))
                if p_1[1] < p_2[1]:
                    rounded_points.append(p_1[1] + abs(radius * sin(angle)))
                else:
                    rounded_points.append(p_1[1] - abs(radius * sin(angle)))

            for _ in range(2):
                if p_2[0] < p_1[0]:
                    rounded_points.append(p_2[0] + abs(radius * cos(angle)))
                else:
                    rounded_points.append(p_2[0] - abs(radius * cos(angle)))
                if p_2[1] < p_1[1]:
                    rounded_points.append(p_2[1] + abs(radius * sin(angle)))
                else:
                    rounded_points.append(p_2[1] - abs(radius * sin(angle)))

        return rounded_points
    
    def draw_tray(self, tray: Tray):
        c_x = int(tray.transform_stamped.transform.translation.x * self.conversion_factor)
        c_y = int(tray.transform_stamped.transform.translation.y * self.conversion_factor)
        if tray.identifier not in TrayCanvas.tray_points_.keys():
            return
        points = self.round_shape(TrayCanvas.tray_points_[tray.identifier], 0.03)
        points = [points[i] * self.conversion_factor + (c_x, c_y)[i%2] for i in range(len(points))]
        self.rotate_shape((c_x, c_y), points, self.get_tray_angle(tray.transform_stamped.transform.rotation))
        self.create_polygon(points, fill=TrayCanvas.tray_colors_[tray.identifier], smooth=True)
        for slot in tray.slots:
            if slot.occupied:
                x_coord = c_x + TrayCanvas.gear_offsets_[tray.identifier][slot.name][0] * self.conversion_factor
                y_coord = c_y + TrayCanvas.gear_offsets_[tray.identifier][slot.name][1] * self.conversion_factor
                slot_coords = [x_coord, y_coord]
                self.rotate_shape((c_x, c_y), slot_coords, self.get_tray_angle(tray.transform_stamped.transform.rotation))
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
    
    
    
