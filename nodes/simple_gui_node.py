#!/usr/bin/env python3
import rclpy
from aprs_gui.simple_gui import SimpleGuiClass
from rclpy.executors import MultiThreadedExecutor
import threading

def main(args=None):
    rclpy.init(args=args)
    
    demo = SimpleGuiClass()
    executor = MultiThreadedExecutor()
    executor.add_node(demo)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    demo.main_window.mainloop()
        
    executor.shutdown()

if __name__ == '__main__':
    main()
