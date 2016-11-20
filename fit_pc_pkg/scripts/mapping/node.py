class MappingNode:

    def __init__(self, parent_node, bounds):
        self.parent = parent_node
        self.bounds = bounds
        self.children = list()
        self.has_leaves = True
