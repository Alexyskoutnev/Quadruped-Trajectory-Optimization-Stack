from collections import deque

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