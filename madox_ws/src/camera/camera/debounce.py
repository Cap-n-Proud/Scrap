import threading
import time


class ButtonHandler(threading.Thread):
    def __init__(self, pin, func, edge="both", bouncetime=200):
        super().__init__(daemon=True)

        self.edge = edge
        self.func = func
        self.pin = pin
        self.bouncetime = float(bouncetime) / 1000

        self.lastpinval = self.pin
        self.lock = threading.Lock()

    def __call__(self, button, *args):
        # if not self.lock.acquire(blocking=False):
        #     print("LOCKED")
        #     return
        # t = threading.Timer(self.bouncetime, self.read, [button], args=args)
        # t.start()
        self.read(button)

    def read(self, button, *args):
        pinval = self.pin

        if (
            pinval == 0 and self.lastpinval == 1
        ):  # and (self.edge in ["falling", "both"]):
            self.func(*args)
            print("release")

        if (
            pinval == 1 and self.lastpinval == 0
        ):  # and (self.edge in ["rising", "both"]):
            self.func(*args)
            print("pressed")
        # time.sleep(0.5)
        print(pinval, self.lastpinval)
        self.lastpinval = pinval
        self.lock.release()
