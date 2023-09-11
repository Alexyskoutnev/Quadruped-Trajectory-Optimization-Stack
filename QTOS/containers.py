from collections import deque

import numpy as np

class LimitedFIFOQueue:
    """A limited-size First-In-First-Out (FIFO) queue.

    Args:
        max_size (int): The maximum size of the queue.

    Attributes:
        queue (deque): The deque representing the FIFO queue.
        max_size (int): The maximum size of the queue.

    Methods:
        enqueue(item): Add an item to the end of the queue.
        dequeue(): Remove and return the oldest item from the queue.
        __len__(): Return the current number of items in the queue.
        average(): Calculate and return the average of the items in the queue.
    """
    
    def __init__(self, max_size):
        self.queue = deque()
        self.max_size = max_size

    def enqueue(self, item):
        """Add an item to the end of the queue.

        If the queue exceeds its maximum size, the oldest item is removed.

        Args:
            item: The item to be added to the queue.
        """
        if len(self.queue) >= self.max_size:
            self.queue.popleft()
        self.queue.append(item)

    def dequeue(self):
        """Remove and return the oldest item from the queue.

        Returns:
            The oldest item in the queue.
        
        Raises:
            IndexError: If the queue is empty.
        """
        if self.queue:
            return self.queue.popleft()
        else:
            raise IndexError("Queue is empty")

    def __len__(self):
        """Return the current number of items in the queue.

        Returns:
            int: The number of items in the queue.
        """

        return len(self.queue)

    def average(self):
        """Calculate and return the average of the items in the queue.

        Returns:
            float: The average value of the items in the queue.

        Note:
            If the queue is empty, the average is considered as 0.
        """
        if len(self.queue) == 0:
            return 0
        return sum(self.queue) / len(self.queue)

class FIFOQueue:
    """A simple First-In-First-Out (FIFO) queue.

    Attributes:
        queue (deque): The deque representing the FIFO queue.

    Methods:
        enqueue(item): Add an item to the end of the queue.
        dequeue(): Remove and return the oldest item from the queue.
        is_empty(): Check if the queue is empty.
        size(): Return the current number of items in the queue.
    """

    def __init__(self):
        self.queue =deque()

    def enqueue(self, item):
        """Add an item to the end of the queue.

        Args:
            item: The item to be added to the queue.
        """
        self.queue.append(item)

    def dequeue(self):
        """Remove and return the oldest item from the queue.

        Returns:
            The oldest item in the queue.
        
        Raises:
            IndexError: If the queue is empty.
        """
        if not self.is_empty():
            return self.queue.popleft()
        else:
            raise IndexError("Visual Queue ID is empty")

    def is_empty(self):
        """Check if the queue is empty.

        Returns:
            bool: True if the queue is empty, False otherwise.
        """
        return len(self.queue) == 0

    def size(self):
        """Return the current number of items in the queue.

        Returns:
            int: The number of items in the queue.
        """
        return len(self.queue)

class Limited_Stack(object):
    """A limited-size stack.

    Args:
        max_size (int): The maximum size of the stack.

    Attributes:
        max_size (int): The maximum size of the stack.
        stack (deque): The deque representing the stack.

    Methods:
        push(item): Add an item to the top of the stack. If the stack exceeds
                    the maximum size, remove the oldest items.
        pop(): Remove and return the top item from the stack.
        peek(): Return the top item from the stack without removing it.
        is_empty(): Check if the stack is empty.
        size(): Return the current number of items in the stack.
        clear(): Remove all items from the stack.
    """

    def __init__(self, max_size=10):
        """Initialize a Limited_Stack object.

        Args:
            max_size (int, optional): The maximum number of items the stack can hold. Defaults to 10.
        """
        self.max_size = max_size
        self.stack = deque()

    def push(self, item):
        """Push an item onto the stack.

        Args:
            item: The item to be pushed onto the stack.
        """
        if type(item[0]) is np.ndarray or type(item[1]) is np.ndarray :
            _item = (item[0].tolist(), item[1].tolist())
        else:
            _item = item
        self.stack.append(_item)
        if len(self.stack) > self.max_size:
            self.stack.popleft()

    def pop(self):
        """Pop and return the top item from the stack.

        Returns:
            The top item in the stack.

        Raises:
            IndexError: If the stack is empty.
        """
        if not self.is_empty():
            return self.stack.pop()
        else:
            raise IndexError("Stack is empty")

    def peek(self):
        """Return the top item from the stack without removing it.

        Returns:
            The top item in the stack.

        Raises:
            IndexError: If the stack is empty.
        """
        if not self.is_empty():
            return self.stack[-1]
        else:
            raise IndexError("Stack is empty")

    def is_empty(self):
        """Check if the stack is empty.

        Returns:
            bool: True if the stack is empty, False otherwise.
        """
        return len(self.stack) == 0

    def size(self):
        """Return the current number of items in the stack.

        Returns:
            int: The number of items in the stack.
        """
        return len(self.stack)

    def clear(self):
        """Remove all items from the stack."""
        self.stack.clear()

