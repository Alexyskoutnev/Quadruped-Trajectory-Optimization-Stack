from collections import deque

import numpy as np

class LimitedFIFOQueue:
    def __init__(self, max_size):
        self.queue = deque()
        self.max_size = max_size

    def enqueue(self, item):
        if len(self.queue) >= self.max_size:
            self.queue.popleft()
        self.queue.append(item)

    def dequeue(self):
        if self.queue:
            return self.queue.popleft()
        else:
            raise IndexError("Queue is empty")

    def __len__(self):
        return len(self.queue)

    def average(self):
        if len(self.queue) == 0:
            return 0
        return sum(self.queue) / len(self.queue)

class FIFOQueue:
    def __init__(self):
        self.queue =deque()

    def enqueue(self, item):
        self.queue.append(item)

    def dequeue(self):
        if not self.is_empty():
            return self.queue.popleft()
        else:
            raise IndexError("Visual Queue ID is empty")

    def is_empty(self):
        return len(self.queue) == 0

    def size(self):
        return len(self.queue)

class Limited_Stack(object):
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.stack = deque()

    def push(self, item):
        if type(item[0]) is np.ndarray or type(item[1]) is np.ndarray :
            _item = (item[0].tolist(), item[1].tolist())
        else:
            _item = item
        self.stack.append(_item)
        if len(self.stack) > self.max_size:
            self.stack.popleft()

    def pop(self):
        if not self.is_empty():
            return self.stack.pop()
        else:
            raise IndexError("Stack is empty")

    def peek(self):
        if not self.is_empty():
            return self.stack[-1]
        else:
            raise IndexError("Stack is empty")

    def is_empty(self):
        return len(self.stack) == 0

    def size(self):
        return len(self.stack)

    def clear(self):
        self.stack.clear()

