import customtkinter as ctk
from rclpy.node import Node
from tkinter import ttk, END
import tkinter as tk
from PIL import Image
from sensor_msgs.msg import Image as ImageMsg, JointState
from rclpy.qos import qos_profile_default
from cv_bridge import CvBridge
from time import time, localtime, strftime
from example_interfaces.srv import Trigger
from aprs_interfaces.msg import SlotPixel, PixelCenter, PixelSlotInfo
from math import sin, cos
from copy import copy
from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml
from difflib import SequenceMatcher
from aprs_interfaces.srv import MoveToNamedPose, Pick, Place
from sensor_msgs.msg import JointState

FRAMEWIDTH=1200
FRAMEHEIGHT=750
FAR_LEFT_COLUMN = 1
LEFT_COLUMN=2
MIDDLE_COLUMN = 3
RIGHT_COLUMN = 4
FAR_RIGHT_COLUMN = 5

GEAR_COLORS_AND_SIZES = {
    1: ("yellow",5),
    2: ("orange", 6),
    3: ("green", 8)
}

GEAR_TRAY_COLORS_AND_SIZES = {
    13: ("red", (13, 13)),
    14: ("blue", (22,22)),
    15: ("red", (10, 22))
}

ROBOTS = ["fanuc", "motoman"]

class DemoControlWindow(Node):
    _service_types = ["move_to_named_pose", "pick_from_slot", "place_in_slot"]

    def __init__(self):
        super().__init__("aprs_demo_gui")
        
        ctk.set_appearance_mode("light")  # Modes: system (default), light, dark
        ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
        
        self.main_window = ctk.CTk()
        
        self.main_window.title("Demo Control")

        self.main_window.grid_rowconfigure(0, weight=1)
        self.main_window.grid_rowconfigure(100, weight=1)
        self.main_window.grid_columnconfigure(0, weight=1)
        self.main_window.grid_columnconfigure(6, weight=1)
        
        s = ttk.Style()
        s.theme_use('clam')
        s.configure('TNotebook', font='Arial Bold')
        
        self.main_window.geometry("1200x950")
        
        # VISION VARIABLES
        self.bridge = CvBridge()
        self.fanuc_image_update_var = ctk.IntVar(value=1)
        self.motoman_image_update_var = ctk.IntVar(value=1)
        self.teach_image_update_var = ctk.IntVar(value=1)
        self.most_recent_fanuc_vision_time = -100
        self.most_recent_motoman_vision_time = -100
        self.most_recent_teach_vision_time = -100
        self.fanuc_image = None
        self.motoman_image = None
        self.teach_image = None

        # ROBOT CONNECTIONS
        self.most_recent_joint_states_times = {robot: -100 for robot in ROBOTS}
        self.most_recent_joint_states = {robot: None for robot in ROBOTS}
        self.joint_states_recieved = {robot: False for robot in ROBOTS}
        self.joint_states_updated = {robot: ctk.IntVar(value=1) for robot in ROBOTS}

        # GET_NAMED_POSITIONS
        self.named_positions = {robot: self.get_named_positions(robot) for robot in ROBOTS}

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_transforms = []

        frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        try:
            self.frames_list = list(frames_dict.keys())
        except:
            self.frames_list = []

        # Service clients
        self.service_clients = {robot: {"move_to_named_pose": self.create_client(MoveToNamedPose, f"/{robot}/move_to_named_pose"),
                              "pick_from_slot": self.create_client(Pick, f"/{robot}/pick_from_slot"),
                              "place_in_slot": self.create_client(Place, f"/{robot}/place_in_slot")}
                              for robot in ROBOTS}
        
        self.notebook = ttk.Notebook(self.main_window)
        
        self.vision_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.vision_frame.pack(fill='both', expand=True)
        self.notebook.add(self.vision_frame, text="Run Demo")
        self.setup_vision_tab()

        self.service_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.service_frame.pack(fill='both', expand=True)
        self.notebook.add(self.service_frame, text="Call Service")
        self.setup_services_tab()

        self.fanuc_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.fanuc_frame.pack(fill='both', expand=True)
        self.notebook.add(self.fanuc_frame, text="Fanuc Info")
        self.setup_fanuc_frame()
        self.temp_fanuc_widgets = []

        self.motoman_frame = ctk.CTkFrame(self.notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        self.motoman_frame.pack(fill='both', expand=True)
        self.notebook.add(self.motoman_frame, text="Motoman Info")
        self.setup_motoman_frame()
        
        self.notebook.grid(pady=10, column=LEFT_COLUMN, columnspan = 3, sticky=tk.E+tk.W+tk.N+tk.S)
        
        # ROS2 SUBSCRIBERS
        self.fanuc_image_subscriber = self.create_subscription(
            ImageMsg, 
            '/fanuc/table_vision/raw_image',
            self.fanuc_image_cb,
            qos_profile_default)
        
        self.motoman_image_subscriber = self.create_subscription(
            ImageMsg, 
            '/motoman/table_vision/raw_image',
            self.motoman_image_cb,
            qos_profile_default)
        
        self.teach_table_image_subscriber = self.create_subscription(
            ImageMsg, 
            '/teach/table_vision/raw_image',
            self.teach_table_image_cb,
            qos_profile_default)
        
        self.fanuc_pixel_subscriber = self.create_subscription(
            SlotPixel,
            "/fanuc/slot_pixel_centers",
            self.update_fanuc_canvas,
            qos_profile_default
        )

        self.motoman_pixel_subscriber = self.create_subscription(
            SlotPixel,
            "/motoman/table_vision/slot_pixel_centers",
            self.update_motoman_canvas,
            qos_profile_default
        )

        self.teach_table_pixel_subscriber = self.create_subscription(
            SlotPixel,
            "/teach_table/slot_pixel_centers",
            self.update_teach_table_canvas,
            qos_profile_default
        )

        self.fanuc_joint_states_subscriber = self.create_subscription(
            JointState,
            "/fanuc/joint_states",
            self.fanuc_joint_state_cb,
            10
        )

        self.motoman_joint_states_subscriber = self.create_subscription(
            JointState,
            "/motoman/joint_states",
            self.motoman_joint_state_cb,
            10
        )
        
        # ROS2 SERVICE CLIENTS
        self.locate_fanuc_trays_client = self.create_client(
            Trigger,
            "/fanuc/table_vision/locate_trays"
        )
        self.update_fanuc_slots_client = self.create_client(
            Trigger,
            "/fanuc/table_vision/update_slots"
        )

        self.locate_motoman_trays_client = self.create_client(
            Trigger,
            "/motoman/table_vision/locate_trays"
        )
        self.update_motoman_slots_client = self.create_client(
            Trigger,
            "/motoman/table_vision/update_slots"
        )

        self.locate_teach_table_trays_client = self.create_client(
            Trigger,
            "/teach/table_vision/locate_trays"
        )
        self.update_teach_table_slots_client = self.create_client(
            Trigger,
            "/teach/table_vision/update_slots"
        )
        
        vision_connection_timer = self.create_timer(0.5, self.vision_connection_cb)
        joint_states_timer = self.create_timer(0.5, self.robot_connection_cb)

        # Robot Status Labels
        ctk.CTkLabel(self.main_window, text="Fanuc Status:").grid(column = LEFT_COLUMN, row = 3)
        self.fanuc_status_label = ctk.CTkLabel(self.main_window, text="Not Connected", text_color="red")
        self.fanuc_status_label.grid(column = LEFT_COLUMN, row = 4, padx=5)

        ctk.CTkLabel(self.main_window, text="Motoman Status:").grid(column = RIGHT_COLUMN, row = 3)
        self.motoman_status_label = ctk.CTkLabel(self.main_window, text="Not Connected", text_color="red")
        self.motoman_status_label.grid(column = RIGHT_COLUMN, row = 4, padx=5) 
        
    # VISION FUNCTIONS
    def setup_vision_tab(self):
        self.vision_frame.grid_rowconfigure(0, weight=1)
        self.vision_frame.grid_rowconfigure(100, weight=1)
        self.vision_frame.grid_columnconfigure(0, weight=1)
        self.vision_frame.grid_columnconfigure(10, weight=1)
        
        # Headers
        fanuc_header_label = ctk.CTkLabel(self.vision_frame, text="FANUC")
        fanuc_header_label.grid(column = LEFT_COLUMN, row=1)
        motoman_header_label = ctk.CTkLabel(self.vision_frame, text="MOTOMAN")
        motoman_header_label.grid(column = MIDDLE_COLUMN, row=1)
        teach_table_header_label = ctk.CTkLabel(self.vision_frame, text="TEACH TABLE")
        teach_table_header_label.grid(column = RIGHT_COLUMN, row=1)
        
        # Status Labels
        fanuc_status_header_label = ctk.CTkLabel(self.vision_frame, text="Status:")
        fanuc_status_header_label.grid(column = LEFT_COLUMN, row=2)
        self.fanuc_vision_status_label = ctk.CTkLabel(self.vision_frame, text="Not Connected", text_color = "red")
        self.fanuc_vision_status_label.grid(column=LEFT_COLUMN, row=3)

        motoman_status_header_label = ctk.CTkLabel(self.vision_frame, text="Status:")
        motoman_status_header_label.grid(column = MIDDLE_COLUMN, row=2)
        self.motoman_vision_status_label = ctk.CTkLabel(self.vision_frame, text="Not Connected", text_color = "red")
        self.motoman_vision_status_label.grid(column=MIDDLE_COLUMN, row=3)
        
        teach_status_header_label = ctk.CTkLabel(self.vision_frame, text="Status:")
        teach_status_header_label.grid(column = RIGHT_COLUMN, row=2)
        self.teach_table_status_label = ctk.CTkLabel(self.vision_frame, text="Not Connected", text_color = "red")
        self.teach_table_status_label.grid(column=RIGHT_COLUMN, row=3)
        
        # Image Labels
        self.fanuc_image_label = ctk.CTkLabel(self.vision_frame, text="")
        self.fanuc_image_label.grid(column=LEFT_COLUMN, row=4)

        self.motoman_image_label = ctk.CTkLabel(self.vision_frame, text="")
        self.motoman_image_label.grid(column=MIDDLE_COLUMN, row=4)
        
        self.teach_table_image_label = ctk.CTkLabel(self.vision_frame, text="")
        self.teach_table_image_label.grid(column=RIGHT_COLUMN, row=4)
        
        # Service buttons
        self.locate_trays_fanuc_vision_button = ctk.CTkButton(self.vision_frame, text="Locate fanuc trays", command=self.locate_fanuc_trays)
        self.locate_trays_fanuc_vision_button.grid(column=LEFT_COLUMN, row=5)
        self.update_slots_fanuc_vision_button = ctk.CTkButton(self.vision_frame, text="Update fanuc slots", command=self.update_fanuc_slots, state=tk.DISABLED)
        self.update_slots_fanuc_vision_button.grid(column=LEFT_COLUMN, row=6)

        self.locate_trays_motoman_vision_button = ctk.CTkButton(self.vision_frame, text="Locate motoman trays", command=self.locate_motoman_trays)
        self.locate_trays_motoman_vision_button.grid(column=MIDDLE_COLUMN, row=5)
        self.update_slots_motoman_vision_button = ctk.CTkButton(self.vision_frame, text="Update motoman slots", command=self.update_motoman_slots, state=tk.DISABLED)
        self.update_slots_motoman_vision_button.grid(column=MIDDLE_COLUMN, row=6)
        
        self.locate_trays_teach_table_vision_button = ctk.CTkButton(self.vision_frame, text="Locate teach table trays", command=self.locate_teach_table_trays)
        self.locate_trays_teach_table_vision_button.grid(column=RIGHT_COLUMN, row=5)
        self.update_slots_teach_table_vision_button = ctk.CTkButton(self.vision_frame, text="Update teach table slots", command=self.update_teach_table_slots, state=tk.DISABLED)
        self.update_slots_teach_table_vision_button.grid(column=RIGHT_COLUMN, row=6)
        
        # Map canvases
        self.fanuc_canvas = tk.Canvas(self.vision_frame, width = 300, height=300, bd = 0, highlightthickness=0)
        self.fanuc_canvas.grid(row = 7,column = LEFT_COLUMN, sticky = "we", padx=50)

        self.motoman_canvas = tk.Canvas(self.vision_frame, width = 300, height=300, bd = 0, highlightthickness=0)
        self.motoman_canvas.grid(row = 7,column = MIDDLE_COLUMN, sticky = "we", padx=50)
        
        self.teach_table_canvas = tk.Canvas(self.vision_frame, width = 300, height=300, bd = 0, highlightthickness=0)
        self.teach_table_canvas.grid(row = 7,column = RIGHT_COLUMN, sticky = "we", padx=50)
        
        self.fanuc_image_update_var.trace_add("write", self.fanuc_image_update)
        self.motoman_image_update_var.trace_add("write", self.motoman_image_update)
        self.teach_image_update_var.trace_add("write", self.teach_image_update)
        
    
    def fanuc_image_cb(self, msg: ImageMsg):
        # fanuc_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # # print(fanuc_image.shape)
        # self.fanuc_image.put(ctk.CTkImage(Image.fromarray(fanuc_image), size=(375, 211)))
        # self.fanuc_image_update()
        self.most_recent_fanuc_vision_time = time()
        self.fanuc_image = msg
        self.fanuc_image_update_var.set((self.fanuc_image_update_var.get()+1)%2)
    
    def fanuc_image_update(self, _, __, ___):
        cv_image = self.bridge.imgmsg_to_cv2(self.fanuc_image, "rgb8")
        frame_to_show = Image.fromarray(cv_image)
        self.fanuc_image_label.configure(image=ctk.CTkImage(frame_to_show, size=(375, 211)))
    
    def motoman_image_cb(self, msg: ImageMsg):
        self.most_recent_motoman_vision_time = time()
        self.motoman_image = msg
        self.motoman_image_update_var.set((self.motoman_image_update_var.get()+1)%2)
    
    def motoman_image_update(self, _, __, ___):
        cv_image = self.bridge.imgmsg_to_cv2(self.motoman_image, "rgb8")
        frame_to_show = Image.fromarray(cv_image)
        self.motoman_image_label.configure(image=ctk.CTkImage(frame_to_show, size=(375, 211)))

    def teach_table_image_cb(self, msg: ImageMsg):
        # teach_table_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # # print(teach_table_image.shape)
        # self.teach_image.put(ctk.CTkImage(Image.fromarray(teach_table_image), size=(375, 211)))
        # self.teach_image_update()
        self.most_recent_teach_vision_time = time()
        self.teach_image = msg
        self.teach_image_update_var.set((self.teach_image_update_var.get()+1)%2)
    
    def teach_image_update(self, _, __, ___):
        cv_image = self.bridge.imgmsg_to_cv2(self.teach_image, "rgb8")
        frame_to_show = Image.fromarray(cv_image)
        self.teach_table_image_label.configure(image=ctk.CTkImage(frame_to_show, size=(375, 211)))
    
    def vision_connection_cb(self):
        current_time = time()
        if current_time - self.most_recent_fanuc_vision_time > 5.0:
            self.fanuc_vision_status_label.configure(text="Not Connected", text_color = "red")
        else:
            self.fanuc_vision_status_label.configure(text="Connected", text_color = "green")
        
        if current_time - self.most_recent_motoman_vision_time > 5.0:
            self.motoman_vision_status_label.configure(text="Not Connected", text_color = "red")
        else:
            self.motoman_vision_status_label.configure(text="Connected", text_color = "green")

        if current_time - self.most_recent_teach_vision_time > 5.0:
            self.teach_table_status_label.configure(text="Not Connected", text_color = "red")
        else:
            self.teach_table_status_label.configure(text="Connected", text_color = "green")
    
    def locate_fanuc_trays(self):
        request = Trigger.Request()
        future = self.locate_fanuc_trays_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to locate Fanuc trays. Be sure that the Fanuc is not blocking the vision system\n")
                return

        self.locate_trays_fanuc_vision_button.configure(state=tk.DISABLED)
        self.update_slots_fanuc_vision_button.configure(state=tk.NORMAL)
    
    def update_fanuc_slots(self):
        request = Trigger.Request()
        future = self.update_fanuc_slots_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to update fanuc slots. Be sure that the Fanuc is not blocking the vision system")
                return
            
    def locate_motoman_trays(self):
        request = Trigger.Request()
        future = self.locate_motoman_trays_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to locate Motoman trays. Be sure that the Motoman is not blocking the vision system")
                return

        self.locate_trays_motoman_vision_button.configure(state=tk.DISABLED)
        self.update_slots_motoman_vision_button.configure(state=tk.NORMAL)
    
    def update_motoman_slots(self):
        request = Trigger.Request()
        future = self.update_motoman_slots_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to update motoman slots. Be sure that the Motoman is not blocking the vision system")
                return
    
    def locate_teach_table_trays(self):
        request = Trigger.Request()
        future = self.locate_teach_table_trays_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to locate teach table trays. Be sure that there is nothing blocking the teach table")
                return
            
        self.locate_trays_teach_table_vision_button.configure(state=tk.DISABLED)
        self.update_slots_teach_table_vision_button.configure(state=tk.NORMAL)
    
    def update_teach_table_slots(self):
        request = Trigger.Request()
        future = self.update_teach_table_slots_client.call_async(request)

        start = time()
        while not future.done():
            pass
            if time()-start >= 5.0:
                self.get_logger().warn("Unable to update teach table slots. Be sure that there is nothing blocking the teach table")
                return
    

    # Pixel callbacks
    def update_fanuc_canvas(self, msg: SlotPixel):
        self.fanuc_canvas.delete("all")
        for tray in msg.kit_trays:
            tray: PixelCenter
            self.draw_kitting_tray(self.fanuc_canvas, tray.x, tray.y, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.fanuc_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)
        for tray in msg.part_trays:
            tray: PixelCenter
            self.draw_gear_tray(self.fanuc_canvas, tray.x, tray.y, tray.identifier, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.fanuc_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)
    
    def update_motoman_canvas(self, msg: SlotPixel):
        self.motoman_canvas.delete("all")
        for tray in msg.kit_trays:
            tray: PixelCenter
            self.draw_kitting_tray(self.motoman_canvas, tray.x, tray.y, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.motoman_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)
        for tray in msg.part_trays:
            tray: PixelCenter
            self.draw_gear_tray(self.motoman_canvas, tray.x, tray.y, tray.identifier, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.motoman_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)
    
    def update_teach_table_canvas(self, msg: SlotPixel):
        self.teach_table_canvas.delete("all")
        for tray in msg.kit_trays:
            tray: PixelCenter
            self.draw_kitting_tray(self.teach_table_canvas, tray.x, tray.y, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.teach_table_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)
        for tray in msg.part_trays:
            tray: PixelCenter
            self.draw_gear_tray(self.teach_table_canvas, tray.x, tray.y, tray.identifier, angle=tray.angle)
            for slot in tray.slots:
                slot: PixelSlotInfo
                if slot.occupied:
                    self.draw_gear(self.teach_table_canvas, slot.slot_center_x, slot.slot_center_y, slot.size)

    # Draw on canvas
    def draw_gear(self, canvas: tk.Canvas, center_x: int, center_y: int, gear_type: int):
        color, size = GEAR_COLORS_AND_SIZES[gear_type]
        canvas.create_oval(center_x-size, center_y-size, center_x+size, center_y+size, fill=color)
    
    def draw_gear_tray(self, canvas: tk.Canvas, center_x: int, center_y: int, tray_type, angle: float = 0.0):
        color, size = GEAR_TRAY_COLORS_AND_SIZES[tray_type]
        points = [center_x-size[0], center_y-size[1], center_x-size[0], center_y+size[1], center_x+size[0], center_y+size[1], center_x+size[0], center_y-size[1]]
        self.rotate_shape(center_x, center_y, points, angle)
        canvas.create_polygon(points, fill=color)
    
    def draw_kitting_tray(self, canvas: tk.Canvas, center_x: int, center_y: int, angle: float = 0.0):
        points = [center_x-17,center_y-21, center_x+8,center_y-21, center_x+17,center_y-10, center_x+17, center_y+10, center_x+8,center_y+21, center_x-17, center_y+21]
        self.rotate_shape(center_x, center_y, points, angle)
        canvas.create_polygon(points, fill="brown")
        
    def rotate_shape(self, center_x: int, center_y: int, points, rotation: float):
        for i in range(0,len(points),2):
            original_x = copy(points[i] - center_x)
            original_y = copy(points[i+1] - center_y)
            points[i] = int((original_x * cos(rotation) + original_y * sin(rotation))) + center_x
            points[i+1] = int((-1 * original_x * sin(rotation) + original_y * cos(rotation))) + center_y
    
    def fanuc_joint_state_cb(self, msg:JointState):
        self.most_recent_joint_states_times["fanuc"] = time()
        self.most_recent_joint_states["fanuc"] = msg
        if not self.joint_states_recieved["fanuc"]:
            self.update_fanuc_frame()

    def motoman_joint_state_cb(self, msg:JointState):
        self.most_recent_joint_states_times["motoman"] = time()
        self.most_recent_joint_states["motoman"] = msg
        if not self.joint_states_recieved["motoman"]:
            self.update_motoman_frame()
        

    def robot_connection_cb(self):
        if time() - self.most_recent_joint_states_times["fanuc"] <= 3.0:
            self.fanuc_status_label.configure(text="Connected", text_color="green")
        else:
            self.fanuc_status_label.configure(text="Not Connected", text_color="red")

        if time() - self.most_recent_joint_states_times["motoman"] <= 3.0:
            self.motoman_status_label.configure(text="Connected", text_color="green")
        else:
            self.motoman_status_label.configure(text="Not Connected", text_color="red")
        
        frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        try:
            self.frames_list = list(frames_dict.keys())
        except:
            self.frames_list = []
    
    # Services tab
    def setup_services_tab(self):
        self.service_frame.grid_rowconfigure(0, weight=1)
        self.service_frame.grid_rowconfigure(100, weight=1)
        self.service_frame.grid_columnconfigure(0, weight=1)
        self.service_frame.grid_columnconfigure(10, weight=1)

        self.selected_service_robot = ctk.StringVar(value=ROBOTS[0])
        robot_service_label = ctk.CTkLabel(self.service_frame, text="Select the robot to call the service for")
        robot_service_label.grid(column=MIDDLE_COLUMN, row=1)

        robot_selection_menu = ctk.CTkOptionMenu(self.service_frame, variable=self.selected_service_robot, values=ROBOTS)
        robot_selection_menu.grid(column=MIDDLE_COLUMN, row=2)
        
        service_selection_label = ctk.CTkLabel(self.service_frame, text="Select the service to call")
        service_selection_label.grid(column=MIDDLE_COLUMN, row=3, padx = 30)

        service_types = copy(self._service_types)
        if len(self.named_positions[self.selected_service_robot.get()]) == 0:
            service_types = self._service_types[1:]
        
        self.selected_service = ctk.StringVar(value=service_types[0])
        service_selection_menu = ctk.CTkOptionMenu(self.service_frame, variable=self.selected_service, values=service_types)
        service_selection_menu.grid(column=MIDDLE_COLUMN, row = 4)

        self.service_menu_widgets = []
        self.selected_named_pose = ctk.StringVar()
        if len(self.named_positions[self.selected_service_robot.get()]) > 0:
            self.selected_named_pose.set(self.named_positions[self.selected_service_robot.get()][0])
        self.selected_frame = ctk.StringVar(value="")

        self.frame_menu = ctk.CTkComboBox(self.service_frame, variable=self.selected_frame, values=self.frames_list)
        
        self.update_service_menu(1,1,1)

        call_service_button = ctk.CTkButton(self.service_frame, text="Call Service", command=self.call_robot_service)
        call_service_button.grid(column=MIDDLE_COLUMN, row=30)

        self.selected_service_robot.trace_add("write", self.update_service_menu)
        self.selected_service.trace_add("write", self.update_service_menu)
        self.selected_frame.trace_add("write", self.only_show_matching_frames)

    
    def update_service_menu(self, _, __, ___):
        for widget in self.service_menu_widgets:
            widget.grid_forget()
        
        self.selected_frame.set("")

        if self.selected_service.get() == "move_to_named_pose":
            self.service_menu_widgets.append(ctk.CTkLabel(self.service_frame, text="Select the pose to move to:"))
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 5)

            self.service_menu_widgets.append(ctk.CTkOptionMenu(self.service_frame, variable=self.selected_named_pose, values = self.named_positions[self.selected_service_robot.get()]))
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 6)

        elif self.selected_service.get() == "pick_from_slot":
            self.service_menu_widgets.append(ctk.CTkLabel(self.service_frame, text="Select the frame for picking:"))
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 5)

            self.service_menu_widgets.append(self.frame_menu)
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 6)
        
        else:
            self.service_menu_widgets.append(ctk.CTkLabel(self.service_frame, text="Select the frame for placing:"))
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 5)

            self.service_menu_widgets.append(self.frame_menu)
            self.service_menu_widgets[-1].grid(column = MIDDLE_COLUMN, row = 6)

    def call_robot_service(self):
        if self.selected_service.get() == "move_to_named_pose":
            move_to_named_pose_request = MoveToNamedPose.Request()
            move_to_named_pose_request.name = self.selected_named_pose

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

    # Get Named Positions
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

    def setup_fanuc_frame(self):
        self.fanuc_frame.grid_rowconfigure(0, weight=1)
        self.fanuc_frame.grid_rowconfigure(100, weight=1)
        self.fanuc_frame.grid_columnconfigure(0, weight=1)
        self.fanuc_frame.grid_columnconfigure(10, weight=1)

        self.fanuc_states_not_recieved_yet_label = ctk.CTkLabel(self.fanuc_frame, text="No joint states have been recieved for the fanuc")
        self.fanuc_states_not_recieved_yet_label.grid(row = 1, column = MIDDLE_COLUMN)

        self.fanuc_joint_states_recieved_time_label = ctk.CTkLabel(self.fanuc_frame, text="")
        self.fanuc_joint_states_names_label = ctk.CTkLabel(self.fanuc_frame, text="")
        self.fanuc_joint_states_positions_label = ctk.CTkLabel(self.fanuc_frame, text="")

        self.update_fanuc_info_button = ctk.CTkButton(self.fanuc_frame, text="Update info", command=self.update_fanuc_info)
    
    def update_fanuc_frame(self):
        if not self.joint_states_recieved["fanuc"]:
            self.joint_states_recieved["fanuc"] = True
            self.fanuc_states_not_recieved_yet_label.grid_forget()
            self.fanuc_joint_states_recieved_time_label.grid(column=LEFT_COLUMN, row = 1, padx = 15)
            self.fanuc_joint_states_recieved_time_label.configure(text=strftime('%Y-%m-%d %H:%M:%S', localtime(self.most_recent_joint_states_times["fanuc"])))
            
            self.fanuc_joint_states_names_label.grid(column = RIGHT_COLUMN, row = 1, padx = 15)
            self.fanuc_joint_states_positions_label.grid(column = RIGHT_COLUMN+1, row = 1, padx = 15)
            self.update_motoman_info_button.grid(column = MIDDLE_COLUMN, row = 20, padx=15, pady=10)

            self.most_recent_joint_states: dict[str, JointState]
        joint_states_names_str = "Name\n"
        joint_states_positions_str = "Position\n"
        for i in range(len(self.most_recent_joint_states["fanuc"].name)):
            joint_states_names_str += self.most_recent_joint_states["fanuc"].name[i] +'\n'
            joint_states_positions_str += str(self.most_recent_joint_states["fanuc"].position[i]) + '\n'

        self.fanuc_joint_states_names_label.configure(text = joint_states_names_str)
        self.fanuc_joint_states_positions_label.configure(text = joint_states_positions_str)
    
    def update_fanuc_info(self):
        self.motoman_joint_states_recieved_time_label.configure(text=strftime('%Y-%m-%d %H:%M:%S', localtime(self.most_recent_joint_states_times["fanuc"])))

        self.most_recent_joint_states: dict[str, JointState]
        joint_states_names_str = "Name\n"
        joint_states_positions_str = "Position\n"
        for i in range(len(self.most_recent_joint_states["motoman"].name)):
            joint_states_names_str += self.most_recent_joint_states["motoman"].name[i] +'\n'
            joint_states_positions_str += str(self.most_recent_joint_states["motoman"].position[i]) + '\n'

        self.fanuc_joint_states_names_label.configure(text = joint_states_names_str)
        self.fanuc_joint_states_positions_label.configure(text = joint_states_positions_str)
    
    def setup_motoman_frame(self):
        self.motoman_frame.grid_rowconfigure(0, weight=1)
        self.motoman_frame.grid_rowconfigure(100, weight=1)
        self.motoman_frame.grid_columnconfigure(0, weight=1)
        self.motoman_frame.grid_columnconfigure(10, weight=1)

        self.motoman_states_not_recieved_yet_label = ctk.CTkLabel(self.motoman_frame, text="No joint states have been recieved for the motoman")
        self.motoman_states_not_recieved_yet_label.grid(row = 1, column = MIDDLE_COLUMN)

        self.motoman_joint_states_recieved_time_label = ctk.CTkLabel(self.motoman_frame, text="")
        self.motoman_joint_states_names_label = ctk.CTkLabel(self.motoman_frame, text="")
        self.motoman_joint_states_positions_label = ctk.CTkLabel(self.motoman_frame, text="")
        self.motoman_joint_states_velocities_label = ctk.CTkLabel(self.motoman_frame, text="")
        self.motoman_joint_states_accelerations_label = ctk.CTkLabel(self.motoman_frame, text="")

        self.update_motoman_info_button = ctk.CTkButton(self.motoman_frame, text="Update info", command=self.update_motoman_info)
    
    def update_motoman_frame(self):
        if not self.joint_states_recieved["motoman"]:
            self.joint_states_recieved["motoman"] = True
            self.motoman_states_not_recieved_yet_label.grid_forget()
            self.motoman_joint_states_recieved_time_label.grid(column=LEFT_COLUMN, row = 1, padx = 15)
            self.motoman_joint_states_recieved_time_label.configure(text=strftime('%Y-%m-%d %H:%M:%S', localtime(self.most_recent_joint_states_times["motoman"])))
            
            self.motoman_joint_states_names_label.grid(column = RIGHT_COLUMN, row = 1, padx = 15)
            self.motoman_joint_states_positions_label.grid(column = RIGHT_COLUMN+1, row = 1, padx = 15)
            self.motoman_joint_states_velocities_label.grid(column = RIGHT_COLUMN+2, row = 1, padx = 15)
            self.motoman_joint_states_accelerations_label.grid(column = RIGHT_COLUMN+3, row = 1, padx = 15)
            self.update_motoman_info_button.grid(column = MIDDLE_COLUMN, row = 20, padx=15, pady=10)

            self.most_recent_joint_states: dict[str, JointState]
        joint_states_names_str = "Name\n"
        joint_states_positions_str = "Position\n"
        joint_states_velocities_str = "Velocities\n"
        joint_states_accelerations_str = "Accelerations\n"
        for i in range(len(self.most_recent_joint_states["motoman"].name)):
            joint_states_names_str += self.most_recent_joint_states["motoman"].name[i] +'\n'
            joint_states_positions_str += str(self.most_recent_joint_states["motoman"].position[i]) + '\n'
            joint_states_velocities_str += str(self.most_recent_joint_states["motoman"].velocity[i]) + '\n'
            joint_states_accelerations_str += str(self.most_recent_joint_states["motoman"].effort[i])  + '\n'

        self.motoman_joint_states_names_label.configure(text = joint_states_names_str)
        self.motoman_joint_states_positions_label.configure(text = joint_states_positions_str)
        self.motoman_joint_states_velocities_label.configure(text = joint_states_velocities_str)
        self.motoman_joint_states_accelerations_label.configure(text = joint_states_accelerations_str)
    
    def update_motoman_info(self):
        self.motoman_joint_states_recieved_time_label.configure(text=strftime('%Y-%m-%d %H:%M:%S', localtime(self.most_recent_joint_states_times["motoman"])))

        self.most_recent_joint_states: dict[str, JointState]
        joint_states_names_str = "Name\n"
        joint_states_positions_str = "Position\n"
        joint_states_velocities_str = "Velocities\n"
        joint_states_accelerations_str = "Accelerations\n"
        for i in range(len(self.most_recent_joint_states["motoman"].name)):
            joint_states_names_str += self.most_recent_joint_states["motoman"].name[i] +'\n'
            joint_states_positions_str += str(self.most_recent_joint_states["motoman"].position[i]) + '\n'
            joint_states_velocities_str += str(self.most_recent_joint_states["motoman"].velocity[i]) + '\n'
            joint_states_accelerations_str += str(self.most_recent_joint_states["motoman"].effort[i])  + '\n'

        self.motoman_joint_states_names_label.configure(text = joint_states_names_str)
        self.motoman_joint_states_positions_label.configure(text = joint_states_positions_str)
        self.motoman_joint_states_velocities_label.configure(text = joint_states_velocities_str)
        self.motoman_joint_states_accelerations_label.configure(text = joint_states_accelerations_str)
