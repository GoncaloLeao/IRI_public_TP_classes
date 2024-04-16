"""
A simple implementation of a mutable priority queue, required by the Dijkstra and A* algorithms.
This structure allows for both the decrease-key and extract-min operations to run in O(log(N)) time.
By: Gonçalo Leão
"""
from typing import TypeVar, Generic

T = TypeVar('T')
# type T must have: (i) accessible field int queue_index; (ii) operator< defined


# Returns the index of the parent of the i-th element.
def parent(i: int) -> int:
    return i // 2


# Returns the index of the left-child of the i-th element.
def left_child(i: int) -> int:
    return i * 2


class MutablePriorityQueue(Generic[T]):
    def __init__(self) -> None:
        self.heap: [T] = [None]  # indices will be used starting in 1 to facilitate parent / child calculations

    # Checks if the queue is empty.
    # Temporal complexity: O(1)
    def empty(self) -> bool:
        return len(self.heap) == 1

    # Removes the element with the smallest key from the queue, according to the < operator of type T.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def extract_min(self) -> T:
        x: T = self.heap[1]
        self.heap[1] = self.heap[-1]
        self.heap.pop()
        if len(self.heap) > 1:
            self.heapify_down(1)
        x.queue_index = 0
        return x

    # Inserts a new element in the queue.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def insert(self, x: T) -> None:
        self.heap.append(x)
        self.heapify_up(len(self.heap) - 1)

    # Updates an existing element of the queue, so that it has a smaller key, according to the < operator of type T.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def decrease_key(self, x: T) -> None:
        self.heapify_up(x.queue_index)

    # Moves the element at index i further up the queue, to reflect its correct key placement (smallest key elements first).
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def heapify_up(self, i: int) -> None:
        x: T = self.heap[i]
        while i > 1 and x < self.heap[parent(i)]:
            self.set(i, self.heap[parent(i)])
            i = parent(i)
        self.set(i, x)

    # Moves the element at index i further down the queue, to reflect its correct key placement (smallest key elements first).
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def heapify_down(self, i: int) -> None:
        x: T = self.heap[i]
        while True:
            k: int = left_child(i)
            if k >= len(self.heap):
                break  # stop because i-th element has no children
            if (k+1 < len(self.heap)) and self.heap[k+1] < self.heap[k]:
                k += 1  # right child of i is the smallest and should switch with i, rather than the left-child, which is done by default in each iteration
            if not (self.heap[k] < x):
                break  # stop because child is not smaller than the i-th element
            self.set(i, self.heap[k])
            i = k
        self.set(i, x)

    # Sets the i-th element of the queue to be x.
    # Temporal complexity: O(1)
    def set(self, i: int, x: T) -> None:
        self.heap[i] = x
        x.queue_index = i


