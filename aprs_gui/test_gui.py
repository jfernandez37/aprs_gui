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
from aprs_interfaces.msg import Trays, Tray
from aprs_vision.slot_offsets import SlotOffsets
from geometry_msgs.msg import Transform, Quaternion

from aprs_interfaces.srv import LocateTrays

# class TrayVisualParameters:
#     def __init__(self, center_x, center_y, rotation, )

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
    tray_sizes_ = {13: ("red", (10, 10)),
                   14: ("green", (20, 10)),
                   15: ("blue", (20, 20)),
                   16: ("black", (25, 10, 15)),
                   17: ("yellow", (25, 13, 15))}
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

        points = self.get_points(tray.identifier, (c_x, c_y), self.get_tray_angle(tray.transform_stamped.transform.rotation))
        self.create_polygon(points, fill="#FF0000")
    
    def get_points(self, identifier: int, tray_center: tuple[int, int], rotation_angle: float):
        print(rotation_angle)
        match(identifier):
            case 13:
                x_size = 0.16 * self.conversion_factor
                y_size = 0.16 * self.conversion_factor
                points = [tray_center[0] - x_size / 2, tray_center[0] - y_size / 2, 
                          tray_center[0] - x_size / 2, tray_center[0] + y_size / 2, 
                          tray_center[0] + x_size / 2, tray_center[0] + y_size / 2, 
                          tray_center[0] + x_size / 2, tray_center[0] - y_size / 2]
            case 14:
                x_size = 0.186 * self.conversion_factor
                y_size = 0.186 * self.conversion_factor
                points = [tray_center[0] - x_size / 2, tray_center[0] - y_size / 2, 
                          tray_center[0] - x_size / 2, tray_center[0] + y_size / 2, 
                          tray_center[0] + x_size / 2, tray_center[0] + y_size / 2, 
                          tray_center[0] + x_size / 2, tray_center[0] - y_size / 2]
            case 15:
                x_size = 0.21 * self.conversion_factor
                above_center = 0.113 * self.conversion_factor
                below_center = 0.024 * self.conversion_factor
                points = [tray_center[0] - x_size / 2, tray_center[0] - below_center, 
                          tray_center[0] - x_size / 2, tray_center[0] + above_center, 
                          tray_center[0] + x_size / 2, tray_center[0] + above_center, 
                          tray_center[0] + x_size / 2, tray_center[0] - below_center]
            case 16:
                points = [tray_center[0] - 0.108 * self.conversion_factor, tray_center[1] - 0.062 * self.conversion_factor, # Top left corner
                          tray_center[0] + 0.108 * self.conversion_factor, tray_center[1] - 0.062 * self.conversion_factor, # Top right corner
                          tray_center[0] + 0.108 * self.conversion_factor, tray_center[1] + 0.05225 * self.conversion_factor, # Bottom right corner
                          tray_center[0] + 0.019 * self.conversion_factor, tray_center[1] + 0.128 * self.conversion_factor, # Very bottom right corner
                          tray_center[0] - 0.019 * self.conversion_factor, tray_center[1] + 0.128 * self.conversion_factor, # Very bottom left corner
                          tray_center[0] - 0.108 * self.conversion_factor, tray_center[1] + 0.05225 * self.conversion_factor, # Bottom right corner
                          ]
            case 17:
                points = [tray_center[0] - 0.105 * self.conversion_factor, tray_center[1] - 0.113 * self.conversion_factor, # Top left corner
                          tray_center[0] + 0.105 * self.conversion_factor, tray_center[1] - 0.113 * self.conversion_factor, # Top right corner
                          tray_center[0] + 0.105 * self.conversion_factor, tray_center[1], # Middle right coner
                          tray_center[0] + 0.06638 * self.conversion_factor, tray_center[1] + 0.08 * self.conversion_factor, # Bottom right corner
                          tray_center[0] - 0.06638 * self.conversion_factor, tray_center[1] + 0.08 * self.conversion_factor, # Bottom left corner
                          tray_center[0] - 0.105 * self.conversion_factor, tray_center[1], # Middle left coner
                          ]
            case _:
                points = []
    
        self.rotate_shape(tray_center, points, rotation_angle)
        return points
    
    def draw_circle(self, c_x, c_y, radius):
        self.create_oval(c_x - radius, c_y - radius, c_x + radius, c_y + radius, fill="red")

    def get_tray_angle(self, q):
        R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        return R.GetRPY()[-1]

    def rotate_shape(self, tray_center: tuple[int, int], points, angle: float):
        for i in range(0,len(points),2):
            original_x = points[i] - tray_center[0]
            original_y = points[i+1] - tray_center[1]
            points[i] = int((original_x * cos(angle) + original_y * sin(angle))) + tray_center[0]
            points[i+1] = int((-1 * original_x * sin(angle) + original_y * cos(angle))) + tray_center[1]
    
