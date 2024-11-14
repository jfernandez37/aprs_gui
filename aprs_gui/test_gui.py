import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk, END
import tkinter as tk

from PIL import Image
from time import time
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from typing import Optional
from numpy import ndarray
from aprs_interfaces.msg import Trays, Tray
from aprs_vision.slot_offsets import SlotOffsets
from geometry_msgs.msg import Transform

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
    def __init__(self, frame):
        super().__init__(frame, height=400, width=400, bd = 0, highlightthickness=0)
        self.conversion_factor: Optional[float] = None
        self.trays_info_recieved = False
        self.tray_tranforms: Optional[list[Transform]] = None
        self.all_trays: Optional[list[Tray]] = None
        self.width: Optional[int] = None
    
    def update_canvas(self):
        self.delete("all")
        if self.conversion_factor is not None and self.trays_info_recieved:
            self.configure(width=self.width)
            for tray in self.all_trays:
                c_x = tray.transform_stamped.transform.translation.x * self.conversion_factor
                c_y = tray.transform_stamped.transform.translation.y * self.conversion_factor
                self.draw_circle(c_x, c_y, 10)
    
    def draw_circle(self, c_x, c_y, radius):
        self.create_oval(c_x - radius, c_y - radius, c_x + radius, c_y + radius, fill="red")


# Utilities
# def rotate_shape(self, center_x: int, center_y: int, points, rotation: float):
#     angle = self.degs_to_rads(rotation)
#     for i in range(0,len(points),2):
#         original_x = points[i] - center_x
#         original_y = points[i+1] - center_y
#         points[i] = int((original_x * cos(angle) + original_y * sin(angle))) + center_x
#         points[i+1] = int((-1 * original_x * sin(angle) + original_y * cos(angle))) + center_y