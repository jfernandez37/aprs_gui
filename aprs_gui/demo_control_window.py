import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk
import tkinter as tk

from PIL import Image
import os
import cv2
from time import time
from math import pi
from functools import partial
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from typing import Optional
from ament_index_python.packages import get_package_share_directory

from aprs_interfaces.msg import Trays, Tray
from aprs_interfaces.srv import LocateTrays
from aprs_vision.stream_handler import StreamHandler

from aprs_gui.custom_widgets import LiveImage, TrayCanvas, RobotStatusFrame, ServicesFrame, PDDLFrame


class GuiClass(Node):
    vision_systems_ = ["fanuc_vision", "motoman_vision", "teach_table_vision", "fanuc_conveyor", "motoman_conveyor"]
    tray_service_topics_ = ["/fanuc/table_trays_info", "/motoman/table_trays_info", "/teach/table_trays_info", "/fanuc/conveyor_trays_info", "/motoman/conveyor_trays_info"]
    robots_ = ["fanuc", "motoman"]
    tray_topics_ = ["move_to_named_pose", "pick_from_slot", "place_in_slot"]
    locate_trays_services_ = ["/fanuc/locate_trays_on_table", "/motoman/locate_trays_on_table", "/teach/locate_trays_on_table","/fanuc/locate_trays_on_conveyor","/motoman/locate_trays_on_conveyor"]
    vision_video_streams_ = {"fanuc_vision": "http://192.168.1.104/mjpg/video.mjpg",
                            "fanuc_conveyor": "http://192.168.1.108/mjpg/video.mjpg",
                            "motoman_vision": "http://192.168.1.110/mjpg/video.mjpg",
                            "motoman_conveyor": "http://192.168.1.107/mjpg/video.mjpg",
                            "teach_table_vision": "http://192.168.1.105/mjpg/video.mjpg"}
    def __init__(self):
        super().__init__("test_gui")
        
        self.main_wind = ctk.CTk()
        ctk.set_appearance_mode("light")
        self.main_wind.geometry("1500x1000")
        self.main_wind.resizable(False, False)

        self.img_max_height = 400
        self.main_wind.grid_rowconfigure([i for i in range(5)], weight=1)
        self.main_wind.grid_columnconfigure((1,2,3), weight=1)

        self.bridge = CvBridge()
        
        # Stream handlers
        share_path = get_package_share_directory('aprs_vision')
        calibration_filepaths = {"fanuc_vision": os.path.join(share_path, 'config', 'fanuc_table_calibration.npz'),
                                 "motoman_vision" : os.path.join(share_path, 'config', 'motoman_table_calibration.npz'),
                                 "fanuc_conveyor": os.path.join(share_path, 'config', 'fanuc_conveyor_calibration.npz'),
                                 "motoman_conveyor": os.path.join(share_path, 'config', 'motoman_conveyor_calibration.npz'),
                                 "teach_table_vision": os.path.join(share_path, 'config', 'teach_table_calibration.npz')}

        self.stream_handlers = {vision_system: StreamHandler(GuiClass.vision_video_streams_[vision_system], calibration_filepaths[vision_system]) for vision_system in GuiClass.vision_systems_}
        self.most_recent_imgs: dict[str: Optional[ctk.CTkImage]] = {vision_system: None for vision_system in GuiClass.vision_systems_}
        
        # Subscribers and clients
        self.tray_subs = {}
        self.locate_clients = {}

        for i in range(len(GuiClass.vision_systems_)):
            self.tray_subs[GuiClass.vision_systems_[i]] = self.create_subscription(
                Trays,
                GuiClass.tray_service_topics_[i],
                partial(self.trays_cb_, GuiClass.vision_systems_[i]),
                qos_profile_default
            )

            self.locate_clients[GuiClass.vision_systems_[i]] = self.create_client(
                LocateTrays,
                GuiClass.locate_trays_services_[i]
            )

        self.joint_states_subs = {}
        for robot in GuiClass.robots_:
                self.joint_states_subs[robot] = self.create_subscription(
                JointState,
                f"/{robot}/joint_states",
                partial(self.joint_state_cb, robot),
                10
            )
        
        self.occupied_slots = {robot: [] for robot in GuiClass.robots_}
        self.unoccupied_slots = {robot: [] for robot in GuiClass.robots_}

        s = ttk.Style()
        s.theme_use('clam')
        s.configure('TNotebook', font='Arial Bold')

        self.notebook = ttk.Notebook(self.main_wind)

        self.visualization_frame = ctk.CTkFrame(self.notebook, width = 1200, height=950, fg_color="#EBEBEB")
        self.visualization_frame.pack(fill='both', expand=True)
        self.visualization_frame.grid_rowconfigure([i for i in range(8)], weight=1)
        self.visualization_frame.grid_columnconfigure((0, 1, 2), weight=1)
        self.notebook.add(self.visualization_frame, text="Visualization")
        self.add_visualization_widgets_to_frame()

        self.services_frame = ServicesFrame(self.notebook, self)
        self.services_frame.pack(fill='both', expand=True)
        self.notebook.add(self.services_frame, text="Services")

        self.pddl_frame = PDDLFrame(self.notebook, self)
        self.pddl_frame.pack(fill='both', expand=True)
        self.notebook.add(self.pddl_frame, text="PDDL")

        self.notebook.grid(pady=10,column=2, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

        self.status_frames: dict[str, RobotStatusFrame] = {}
        self.status_frames["fanuc"] = RobotStatusFrame(self.main_wind, "fanuc")
        self.status_frames["fanuc"].grid(pady=10,column=1, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

        self.status_frames["motoman"] = RobotStatusFrame(self.main_wind, "motoman")
        self.status_frames["motoman"].grid(pady=10,column=3, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

        self.update_imgs()

    def update_imgs(self):
        for vision_system in GuiClass.vision_systems_:
            cv_image = self.stream_handlers[vision_system].read_frame()
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            width = int(cv_image.shape[1] * self.img_max_height / cv_image.shape[0])
            self.most_recent_imgs[vision_system] = ctk.CTkImage(Image.fromarray(cv_image), size=(width, self.img_max_height))
            
            if vision_system == self.vision_selection.get():
                self.live_image_label.current_image = self.most_recent_imgs[vision_system]

            height_in_inches = cv_image.shape[0] / 30
            height_in_meters = height_in_inches * 0.0254
            self.visualization_canvases[vision_system].global_conversion_factor = 400 / height_in_meters
            self.visualization_canvases[vision_system].width = width
        

        self.main_wind.after(50, self.update_imgs)

    def add_visualization_widgets_to_frame(self):
        # Locate Trays widgets
        self.locate_trays_frame = ctk.CTkFrame(self.visualization_frame, 200, 800, fg_color="#EBEBEB")
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
        vision_selection_label = ctk.CTkLabel(self.visualization_frame, text="Select the vision system for the live view")
        vision_selection_label.grid(column = 1, row = 0, pady=1, sticky="ew")
        self.vision_selection = ctk.StringVar(value=GuiClass.vision_systems_[0])
        self.vision_selection_menu = ctk.CTkOptionMenu(self.visualization_frame, variable=self.vision_selection, values=GuiClass.vision_systems_)
        self.vision_selection_menu.grid(column = 1, row = 1, pady = 1, sticky="ew")

        self.center_visualization_frame = ctk.CTkFrame(self.visualization_frame, 600, 900, fg_color="#EBEBEB")
        self.live_image_label = LiveImage(self.center_visualization_frame)
        self.live_image_label.pack(pady=1, padx=20)
        self.center_visualization_frame.grid(column = 1, row = 2, rowspan=6, padx=20)

        # Subcanvas frame
        self.subcanvas_frame = ctk.CTkFrame(self.visualization_frame, 400, 900, fg_color="#EBEBEB")
        self.subcanvas_frame.pack_propagate(0)
        self.subcanvas_frame.grid(column = 2, row = 2, rowspan=8, padx=20)

        self.visualization_canvases = {vision_system: TrayCanvas(self.main_wind) for vision_system in GuiClass.vision_systems_}
        for vision_system in GuiClass.vision_systems_:
            self.visualization_canvases[vision_system].bind('<Button-1>', partial(self.vis_clicked, vision_system))
        self.visualization_labels: list[ctk.CTkLabel] = []

        self.show_all_canvases(1,1,1)
        self.vision_selection.trace_add("write", self.show_all_canvases)

    def trays_cb_(self, vision_system: str, msg: Trays):
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        self.visualization_canvases[vision_system].trays_info_recieved = True
        self.visualization_canvases[vision_system].all_trays = all_trays

        for robot in GuiClass.robots_:
            if robot in vision_system:
                for tray in all_trays:
                    for slot in tray.slots:
                        if slot.occupied:
                            self.occupied_slots[robot].append(slot.name)
                        else:
                            self.unoccupied_slots[robot].append(slot.name)
                occupied_refresh = len(self.occupied_slots[robot]) > 0 and len(self.services_frame.occupied_slots[robot]) == 0
                unoccupied_refresh = len(self.unoccupied_slots[robot]) > 0 and len(self.services_frame.unoccupied_slots[robot]) == 0
                self.occupied_slots[robot] = list(set(self.occupied_slots[robot]))
                self.unoccupied_slots[robot] = list(set(self.unoccupied_slots[robot]))
                self.services_frame.occupied_slots = self.occupied_slots
                self.services_frame.unoccupied_slots = self.unoccupied_slots

                if occupied_refresh or unoccupied_refresh:
                    self.services_frame.reload_services_frames()
    
    def joint_state_cb(self, robot: str, msg: JointState):
        self.status_frames[robot].most_recent_time = time()
    
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
                    if time()-start >= 3.0:
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
        self.visualization_canvases[self.vision_selection.get()].update_canvas(True)
        self.visualization_canvases[self.vision_selection.get()].pack(in_=self.center_visualization_frame, pady=20, padx=20)
        self.visualization_labels.append(ctk.CTkLabel(self.center_visualization_frame, text=self.vision_selection.get()))
        self.visualization_labels[-1].pack(pady=5, padx=20)

        current_row = 0
        for vision_system in GuiClass.vision_systems_:
            if vision_system != self.vision_selection.get():
                self.visualization_canvases[vision_system].side_canvas = True
                self.visualization_canvases[vision_system].update_canvas(True)
                self.visualization_canvases[vision_system].grid(in_=self.subcanvas_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1
                self.visualization_labels.append(ctk.CTkLabel(self.subcanvas_frame, text=vision_system))
                self.visualization_labels[-1].grid(in_=self.subcanvas_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1

    def vis_clicked(self, vision_system, event):
        self.vision_selection.set(vision_system)