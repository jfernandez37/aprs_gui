import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk, END
import tkinter as tk

from PIL import Image
import PyKDL
from time import time
from math import sin, cos, pi
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from typing import Optional
from numpy import ndarray
from aprs_interfaces.msg import Trays, Tray, SlotInfo
from aprs_vision.slot_offsets import SlotOffsets
from geometry_msgs.msg import Transform, Quaternion

from aprs_interfaces.srv import LocateTrays

# class TrayVisualParameters:
#     def __init__(self, center_x, center_y, rotation, )

def deg_to_rad(deg: float):
        return deg * pi / 180
class GuiClass(Node):
    def __init__(self):
        super().__init__("test_gui")
        
        self.main_wind = ctk.CTk()
        ctk.set_appearance_mode("light")
        self.main_wind.geometry("800x1000")
        self.main_wind.resizable(False, False)

        self.img_max_height = 400
        # self.main_wind.grid_rowconfigure((0,1), weight=1)
        # self.main_wind.grid_columnconfigure(0, weight=1)

        self.bridge = CvBridge()

        self.fanuc_live_image_label = LiveImage(self.main_wind)
        # self.fanuc_live_image_label.grid(column = 0, row = 0, pady=20, padx=20, sticky="ew")
        self.fanuc_live_image_label.pack(pady = 20)

        self.fanuc_visualization_canvas = TrayCanvas(self.main_wind)
        # self.dummy.grid(column = 0, row = 1, pady=20, padx=20, sticky="ew")
        self.fanuc_visualization_canvas.pack(pady=20)

        self.fanuc_locate_trays_button = ctk.CTkButton(self.main_wind, text="Locate trays", command=self.fanuc_table_locate_trays)
        self.fanuc_locate_trays_button.pack()
        
    
        self.fanuc_image_subscriber = self.create_subscription(
            ImageMsg, 
            '/fanuc/table_vision/raw_image',
            self.fanuc_image_cb,
            qos_profile_default)
        
        self.fanuc_table_trays_info_sub = self.create_subscription(
            Trays,
            "/fanuc/table_vision/trays_info",
            self.fanuc_table_trays_cb_,
            qos_profile_default
        )

        self.locate_fanuc_trays_client = self.create_client(
            LocateTrays,
            "/fanuc/table_vision/locate_trays"
        )

    def fanuc_image_cb(self, msg: ImageMsg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        width = int(cv_image.shape[1] * self.img_max_height / cv_image.shape[0])
        # self.fanuc_live_image_label.current_image = ctk.CTkImage(Image.fromarray(cv_image), size=(cv_image.shape[1], cv_image.shape[0]))
        self.fanuc_live_image_label.current_image = ctk.CTkImage(Image.fromarray(cv_image), size=(width, 400))
        height_in_inches = cv_image.shape[0] / 30
        height_in_meters = height_in_inches * 0.0254
        self.fanuc_visualization_canvas.conversion_factor = 400 / height_in_meters
        self.get_logger().info(str(self.fanuc_visualization_canvas.conversion_factor))
        self.fanuc_visualization_canvas.width = width


    def fanuc_table_trays_cb_(self, msg: Trays):
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        self.fanuc_visualization_canvas.trays_info_recieved = True
        self.fanuc_visualization_canvas.all_trays = all_trays
        self.fanuc_visualization_canvas.update_canvas()
    
    def fanuc_table_locate_trays(self):
        request = LocateTrays.Request()
        future = self.locate_fanuc_trays_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to locate Fanuc trays. Be sure that the Fanuc is not blocking the vision system\n")
                return


class LiveImage(ctk.CTkLabel):
    def __init__(self, frame):
        super().__init__(frame, text="Image not found", fg_color="#C2C2C2", height=400, width=400, font=("UbuntuMono",50))
        self.current_image: Optional[ctk.CTkImage] = None
        self.update_image()

    def update_image(self):
        if self.current_image is not None:
            self.configure(text="", image=self.current_image, fg_color="transparent")
        self.after(100, self.update_image)

class TrayCanvas(tk.Canvas):
    tray_points_ = {Tray.SMALL_GEAR_TRAY: [
        0.08 - 0.03, -0.08,
        0.08 - 0.03, -0.08,
        -0.08 + 0.03, -0.08,
        -0.08 + 0.03, -0.08,
        -0.08, -0.08,
        -0.08, -0.08 + 0.03,
        -0.08, -0.08 + 0.03,
        -0.08, 0.08 - 0.03,
        -0.08, 0.08 - 0.03,
        -0.08, 0.08, 
        -0.08 + 0.03, 0.08,
        -0.08 + 0.03, 0.08, 
        0.08 - 0.03, 0.08, 
        0.08 - 0.03, 0.08,
        0.08, 0.08, 
        0.08, 0.08 - 0.03,
        0.08, 0.08 - 0.03,
        0.08, -0.08 + 0.03,
        0.08, -0.08 + 0.03,
        0.08, -0.08
    ],
    Tray.MEDIUM_GEAR_TRAY: [
        0.098 - 0.03, -0.098,
        0.098 - 0.03, -0.098,
        -0.098 + 0.03, -0.098,
        -0.098 + 0.03, -0.098,
        -0.098, -0.098,
        -0.098, -0.098 + 0.03,
        -0.098, -0.098 + 0.03,
        -0.098, 0.098 - 0.03,
        -0.098, 0.098 - 0.03,
        -0.098, 0.098, 
        -0.098 + 0.03, 0.098,
        -0.098 + 0.03, 0.098, 
        0.098 - 0.03, 0.098, 
        0.098 - 0.03, 0.098,
        0.098, 0.098, 
        0.098, 0.098 - 0.03,
        0.098, 0.098 - 0.03,
        0.098, -0.098 + 0.03,
        0.098, -0.098 + 0.03,
        0.098, -0.098
    ],
    Tray.LARGE_GEAR_TRAY: [
        0.105 - 0.03, -0.24,
        0.105 - 0.03, -0.24,
        -0.105 + 0.03, -0.24,
        -0.105 + 0.03, -0.24,
        -0.105, -0.24,
        -0.105, -0.24 + 0.03,
        -0.105, -0.24 + 0.03,
        -0.105, 0.113 - 0.03,
        -0.105, 0.113 - 0.03,
        -0.105, 0.113, 
        -0.105 + 0.03, 0.113,
        -0.105 + 0.03, 0.113, 
        0.105 - 0.03, 0.113, 
        0.105 - 0.03, 0.113,
        0.105, 0.113, 
        0.105, 0.113 - 0.03,
        0.105, 0.113 - 0.03,
        0.105, -0.24 + 0.03,
        0.105, -0.24 + 0.03,
        0.105, -0.24
    ],
    Tray.M2L1_KIT_TRAY: [
        -0.108, -0.062 + 0.03,
        -0.108, -0.062 + 0.03,
        -0.108, -0.062, # Top left corner
        -0.108 + 0.03, -0.062,
        -0.108 + 0.03, -0.062,
        0.108 - 0.03, -0.062, 
        0.108 - 0.03, -0.062, 
        0.108, -0.062, # Top right corner
        0.108, -0.062 + 0.03,
        0.108, -0.062 + 0.03,
        0.108, 0.05225 - 0.03,
        0.108, 0.05225 - 0.03,
        0.108, 0.05225, # Bottom right corner
        0.108 - 0.03 * sin(deg_to_rad(50)), 0.05225 + 0.03 * cos(deg_to_rad(50)),
        0.108 - 0.03 * sin(deg_to_rad(50)), 0.05225 + 0.03 * cos(deg_to_rad(50)),
        0.019 + 0.03 * sin(deg_to_rad(50)), 0.128 - 0.03 * cos(deg_to_rad(50)),
        0.019 + 0.03 * sin(deg_to_rad(50)), 0.128 - 0.03 * cos(deg_to_rad(50)),
        0.019, 0.128, # Very bottom right corner
        0.019 - 0.03, 0.128,
        0.019 - 0.03, 0.128,
        -0.019 + 0.03, 0.128,
        -0.019 + 0.03, 0.128,
        -0.019, 0.128, # Very bottom left corner
        -0.019 - 0.03 * sin(deg_to_rad(50)), 0.128 - 0.03 * cos(deg_to_rad(50)),
        -0.019 - 0.03 * sin(deg_to_rad(50)), 0.128 - 0.03 * cos(deg_to_rad(50)),
        -0.108 + 0.03 * sin(deg_to_rad(50)), 0.05225 +  0.03 * cos(deg_to_rad(50)),
        -0.108 + 0.03 * sin(deg_to_rad(50)), 0.05225 +  0.03 * cos(deg_to_rad(50)),
        -0.108, 0.05225, # Bottom left corner
        -0.108, 0.05225 - 0.03,
        -0.108, 0.05225 - 0.03,
    ],
    Tray.S2L2_KIT_TRAY: [
        -0.105, -0.113 + 0.03,
        -0.105, -0.113 + 0.03,
        -0.105, -0.113, # Top left corner
        -0.105 + 0.03, -0.113,
        -0.105 + 0.03, -0.113,
        0.105 - 0.03, -0.113,
        0.105 - 0.03, -0.113,
        0.105, -0.113, # Top right corner
        0.105, -0.113 + 0.03,
        0.105, -0.113 + 0.03,
        0.105, -0.03,
        0.105, -0.03,
        0.105, 0.0, # Middle right coner
        0.105 - 0.012827119, 0.0271186441,
        0.105 - 0.012827119, 0.0271186441,
        0.06638 + 0.012827119, 0.08 - 0.0271186441,
        0.06638 + 0.012827119, 0.08 - 0.0271186441,
        0.06638, 0.08, # Bottom right corner
        0.06638 - 0.03, 0.08,
        0.06638 - 0.03, 0.08,
        -0.06638 + 0.03, 0.08,
        -0.06638 + 0.03, 0.08,
        -0.06638,  0.08, # Bottom left corner
        -0.06638 - 0.012827119, 0.08 - 0.0271186441,
        -0.06638 - 0.012827119, 0.08 - 0.0271186441,
        -0.105 + 0.012827119, 0.0271186441,
        -0.105 + 0.012827119, 0.0271186441,
        -0.105, 0.0, # Middle left coner
        -0.105, - 0.03,
        -0.105, - 0.03
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
    
    def __init__(self, frame):
        super().__init__(frame, height=400, width=400, bd = 0, highlightthickness=0)
        self.conversion_factor: Optional[float] = 684.6970215679562
        self.trays_info_recieved = False
        self.tray_tranforms: Optional[list[Transform]] = None
        self.all_trays: Optional[list[Tray]] = None
        self.width: Optional[int] = None
    
    def update_canvas(self):
        self.delete("all")
        if self.conversion_factor is not None and self.trays_info_recieved:
            self.configure(width=self.width)
            for tray in self.all_trays:
                self.draw_tray(tray)
    
    def draw_tray(self, tray: Tray):
        c_x = int(tray.transform_stamped.transform.translation.x * self.conversion_factor)
        c_y = int(tray.transform_stamped.transform.translation.y * self.conversion_factor)
        if tray.identifier not in TrayCanvas.tray_points_.keys():
            return
        points = [TrayCanvas.tray_points_[tray.identifier][i] * self.conversion_factor + (c_x, c_y)[i%2] for i in range(len(TrayCanvas.tray_points_[tray.identifier]))]
        self.rotate_shape((c_x, c_y), points, self.get_tray_angle(tray.transform_stamped.transform.rotation))
        self.create_polygon(points, fill="#FF0000", smooth=True)
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
    
    
    
