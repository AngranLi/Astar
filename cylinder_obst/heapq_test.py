import heapq
import math

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

frontier = PriorityQueue()
priority = math.sqrt(2**2+3**2)
frontier.put((0,0), priority)
priority = math.sqrt(1**2+2**2)
frontier.put((1,1), priority)
print frontier.get()
