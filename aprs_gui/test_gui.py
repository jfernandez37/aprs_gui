import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk
import tkinter as tk

from PIL import Image
import yaml
from time import time
from math import pi
from functools import partial
from copy import copy
from ament_index_python.packages import get_package_share_directory
from difflib import SequenceMatcher
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from typing import Optional
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from aprs_interfaces.msg import Trays, Tray
from aprs_interfaces.srv import LocateTrays, Pick, Place, MoveToNamedPose

from aprs_gui.custom_widgets import LiveImage, TrayCanvas, RobotStatusFrame

def deg_to_rad(deg: float):
        return deg * pi / 180

class GuiClass(Node):
    vision_systems_ = ["fanuc_vision", "motoman_vision", "teach_table_vision", "fanuc_conveyor", "motoman_conveyor"]
    service_headers_ = ["/fanuc/table_vision", "/motoman/table_vision", "/teach/table_vision", "/fanuc/conveyor_vision", "/motoman/conveyor_vision"]
    robots_ = ["fanuc", "motoman"]
    service_types_ = ["move_to_named_pose", "pick_from_slot", "place_in_slot"]
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

        self.joint_states_subs = {}
        for robot in GuiClass.robots_:
                self.joint_states_subs[robot] = self.create_subscription(
                JointState,
                f"/{robot}/joint_states",
                partial(self.joint_state_cb, robot),
                10
            )
        
        self.service_clients = {robot: {"move_to_named_pose": self.create_client(MoveToNamedPose, f"/{robot}/move_to_named_pose"),
                              "pick_from_slot": self.create_client(Pick, f"/{robot}/pick_from_slot"),
                              "place_in_slot": self.create_client(Place, f"/{robot}/place_in_slot")}
                              for robot in GuiClass.robots_}

        self.named_positions = {robot: self.get_named_positions(robot) for robot in GuiClass.robots_}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        try:
            self.frames_list = list(frames_dict.keys())
        except:
            self.frames_list = []

        frames_timer = self.create_timer(0.5, self.get_current_frames)
        status_timer = self.create_timer(1.0, self.update_robot_statuses)

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

        self.services_frame = ctk.CTkFrame(self.notebook, width = 1200, height=950, fg_color="#EBEBEB")
        self.services_frame.pack(fill='both', expand=True)
        self.services_frame.grid_rowconfigure([i for i in range(8)], weight=1)
        self.services_frame.grid_columnconfigure((0,1,2), weight=1)
        self.notebook.add(self.services_frame, text="Services")
        self.add_services_widgets_to_frame()

        self.notebook.grid(pady=10,column=2, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

        self.status_frames: dict[str, RobotStatusFrame] = {}
        self.status_frames["fanuc"] = RobotStatusFrame(self.main_wind, "fanuc")
        self.status_frames["fanuc"].grid(pady=10,column=1, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

        self.status_frames["motoman"] = RobotStatusFrame(self.main_wind, "motoman")
        self.status_frames["motoman"].grid(pady=10,column=3, row=2, sticky=tk.E+tk.W+tk.N+tk.S)

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

        self.most_recent_imgs: dict[str: Optional[ctk.CTkImage]] = {vision_system: None for vision_system in GuiClass.vision_systems_}
        self.center_visualization_frame = ctk.CTkFrame(self.visualization_frame, 600, 900, fg_color="#EBEBEB")
        self.live_image_label = LiveImage(self.center_visualization_frame)
        self.live_image_label.pack(pady=1, padx=20)
        self.center_visualization_frame.grid(column = 1, row = 2, rowspan=6, padx=20)

        # Subcanvas frame
        self.subcanvas_frame = ctk.CTkFrame(self.visualization_frame, 400, 900, fg_color="#EBEBEB")
        self.subcanvas_frame.grid(column = 2, row = 2, rowspan=8, padx=20)

        self.visualization_canvases = {vision_system: TrayCanvas(self.main_wind) for vision_system in GuiClass.vision_systems_}
        for vision_system in GuiClass.vision_systems_:
            self.visualization_canvases[vision_system].bind('<Button-1>', partial(self.vis_clicked, vision_system))
        self.visualization_labels: list[ctk.CTkLabel] = []

        self.show_all_canvases(1,1,1)
        self.vision_selection.trace_add("write", self.show_all_canvases)
    
    def add_services_widgets_to_frame(self):

        self.selected_service_robot = ctk.StringVar(value=GuiClass.robots_[0])
        robot_service_label = ctk.CTkLabel(self.services_frame, text="Select the robot to call the service for")
        robot_service_label.grid(column=1, row=1)

        robot_selection_menu = ctk.CTkOptionMenu(self.services_frame, variable=self.selected_service_robot, values=GuiClass.robots_)
        robot_selection_menu.grid(column=1, row=2)
        
        service_selection_label = ctk.CTkLabel(self.services_frame, text="Select the service to call")
        service_selection_label.grid(column=1, row=3, padx = 30)

        service_types = copy(GuiClass.service_types_)
        if len(self.named_positions[self.selected_service_robot.get()]) == 0:
            service_types = GuiClass.service_types_[1:]
        
        self.selected_service = ctk.StringVar(value=service_types[0])
        service_selection_menu = ctk.CTkOptionMenu(self.services_frame, variable=self.selected_service, values=service_types)
        service_selection_menu.grid(column=1, row = 4)

        self.service_menu_widgets = []
        self.selected_named_pose = ctk.StringVar()
        if len(self.named_positions[self.selected_service_robot.get()]) > 0:
            self.selected_named_pose.set(self.named_positions[self.selected_service_robot.get()][0])
        self.selected_frame = ctk.StringVar(value="")

        self.frame_menu = ctk.CTkComboBox(self.services_frame, variable=self.selected_frame, values=self.frames_list)
        
        self.update_service_menu(1,1,1)

        call_service_button = ctk.CTkButton(self.services_frame, text="Call Service", command=self.call_robot_service)
        call_service_button.grid(column=1, row=7)

        self.selected_service_robot.trace_add("write", self.update_service_menu)
        self.selected_service.trace_add("write", self.update_service_menu)
        self.selected_frame.trace_add("write", self.only_show_matching_frames)
    
    def get_current_frames(self):
        frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        try:
            self.frames_list = list(frames_dict.keys())
        except:
            self.frames_list = []
    
    def update_robot_statuses(self):
        for robot in GuiClass.robots_:
            self.status_frames[robot].update_status()

    def update_service_menu(self, _, __, ___):
        for widget in self.service_menu_widgets:
            widget.grid_forget()
        
        self.selected_frame.set("")

        if self.selected_service.get() == "move_to_named_pose":
            self.service_menu_widgets.append(ctk.CTkLabel(self.services_frame, text="Select the pose to move to:"))
            self.service_menu_widgets[-1].grid(column = 1, row = 5)

            self.service_menu_widgets.append(ctk.CTkOptionMenu(self.services_frame, variable=self.selected_named_pose, values = self.named_positions[self.selected_service_robot.get()]))
            self.service_menu_widgets[-1].grid(column = 1, row = 6)

        elif self.selected_service.get() == "pick_from_slot":
            self.service_menu_widgets.append(ctk.CTkLabel(self.services_frame, text="Select the frame for picking:"))
            self.service_menu_widgets[-1].grid(column = 1, row = 5)

            self.service_menu_widgets.append(self.frame_menu)
            self.service_menu_widgets[-1].grid(column = 1, row = 6)
        
        else:
            self.service_menu_widgets.append(ctk.CTkLabel(self.services_frame, text="Select the frame for placing:"))
            self.service_menu_widgets[-1].grid(column = 1, row = 5)

            self.service_menu_widgets.append(self.frame_menu)
            self.service_menu_widgets[-1].grid(column = 1, row = 6)

    def only_show_matching_frames(self, _, __, ___):
        selection = self.selected_frame.get()

        if selection in self.frames_list:
            self.frame_menu.configure(values = self.frames_list)
        else:
            options = []
            for topic in self.frames_list:
                if selection.lower() in topic.lower():
                    options.append(topic)
                else:
                    for i in range(len(topic)-len(selection)):
                        if SequenceMatcher(None, selection.lower(), topic[i:i+len(selection)].lower()).ratio() > 0.6:
                            options.append(topic)
            self.frame_menu.configure(values = list(set(options)))
    
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
    
    def call_robot_service(self):
        if self.selected_service.get() == "move_to_named_pose":
            move_to_named_pose_request = MoveToNamedPose.Request()
            move_to_named_pose_request.name = self.selected_named_pose.get()

            future = self.service_clients[self.selected_service_robot.get()][self.selected_service.get()].call_async(move_to_named_pose_request)

            start = time()
            while not future.done():
                pass
                if time()-start >= 15.0:
                    self.get_logger().warn("Unable to Move fanuc to desired pose")
                    return
        elif self.selected_service.get() == "pick_from_slot":
            pass
        
        else:
            pass

    def image_cb(self, vision_system: str, msg: ImageMsg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        width = int(cv_image.shape[1] * self.img_max_height / cv_image.shape[0])
        
        self.most_recent_imgs[vision_system] = ctk.CTkImage(Image.fromarray(cv_image), size=(width, self.img_max_height))
        if vision_system == self.vision_selection.get():
            self.live_image_label.current_image = self.live_image_label[vision_system]

        height_in_inches = cv_image.shape[0] / 30
        height_in_meters = height_in_inches * 0.0254
        self.visualization_canvases[vision_system].conversion_factor = 400 / height_in_meters
        self.visualization_canvases[vision_system].width = width


    def trays_cb_(self, vision_system: str, msg: Trays):
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        self.visualization_canvases[vision_system].trays_info_recieved = True
        self.visualization_canvases[vision_system].all_trays = all_trays
        self.visualization_canvases[vision_system].update_canvas()
    
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
        self.visualization_canvases[self.vision_selection.get()].pack(in_=self.center_visualization_frame, pady=20, padx=20)
        self.visualization_labels.append(ctk.CTkLabel(self.center_visualization_frame, text=self.vision_selection.get()))
        self.visualization_labels[-1].pack(pady=5, padx=20)

        current_row = 0
        for vision_system in GuiClass.vision_systems_:
            if vision_system != self.vision_selection.get():
                self.visualization_canvases[vision_system].side_canvas = True
                self.visualization_canvases[vision_system].update_canvas()
                self.visualization_canvases[vision_system].grid(in_=self.subcanvas_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1
                self.visualization_labels.append(ctk.CTkLabel(self.subcanvas_frame, text=vision_system))
                self.visualization_labels[-1].grid(in_=self.subcanvas_frame, column = 0, row = current_row, pady=5, padx=20, sticky="ew")
                current_row += 1

    def vis_clicked(self, vision_system, event):
        self.vision_selection.set(vision_system)


    
    
    
