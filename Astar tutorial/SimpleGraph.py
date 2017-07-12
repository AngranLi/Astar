class SimpleGraph:
    def __init__(self):
        self.edges = {}
    
    def neighbors(self, id):
        return self.edges[id]

example_graph = SimpleGraph()
example_graph.edges = {
	'A':['B'],
	'B':['A', 'C', 'D'],
	'C':['A'],
	'D':['A', 'E']
	'E':['B']
}
