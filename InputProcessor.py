import threading
import keyboard


class InputPool:
    def __init__(self):
        self.lock = threading.Lock()
        self.input_queue = []

    def push(self, s):
        self.lock.acquire()
        self.input_queue.append(s)
        self.lock.release()

    def shift(self):
        self.lock.acquire()
        s = self.input_queue.pop(0)
        self.lock.release()
        return s

    def length(self):
        self.lock.acquire()
        l = len(self.input_queue)
        self.lock.release()
        return l

    def clear(self):
        self.lock.acquire()
        self.input_queue.clear()
        self.lock.release()


class InputProcessor(threading.Thread):
    def __init__(self, th_name, input_pool):
        self.input_pool = input_pool
        self.thread_name = th_name
        threading.Thread.__init__(self)

    def __str__(self):
        return self.thread_name

    def run(self):
        while True:
            input_s = input()
            self.input_pool.push(input_s)
            if input_s == 'exit':
                break



