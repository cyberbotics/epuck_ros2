import threading
import time
from rclpy.node import Node


class TimerThread(threading.Thread):
    def __init__(self, period, callback, event):
        self.callback = callback
        self.event = event
        self.period = period
        super(TimerThread, self).__init__()

    def run(self):
        last_time = time.time()
        while not self.event.wait(self.period - (time.time() - last_time)):
            last_time = time.time()
            self.callback()


class WebotsNode(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

    def create_timer(self, period, callback):
        timer_event = threading.Event()
        timer_thread = TimerThread(period, callback, timer_event)
        timer_thread.start()
