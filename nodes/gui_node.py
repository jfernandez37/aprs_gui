#!/usr/bin/env python3
import rclpy
from aprs_gui.demo_control_window import GuiClass
from rclpy.executors import MultiThreadedExecutor
import threading

def main(args=None):
    rclpy.init(args=args)
    
    demo = GuiClass()
    executor = MultiThreadedExecutor()
    executor.add_node(demo)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    demo.main_wind.mainloop()
        
    executor.shutdown()

if __name__ == '__main__':
    main()
