from node import MappingNode
from bounds import MappingBounds


class MappingTree:
    """ A quadtree to store the location of objects in a coordinate plane. """

    def __init__(self, starting_bounds, max_child_items):
        self.root_node = MappingNode(None, starting_bounds)
        self.max_leaves = max_child_items
        self.has_leaves = True

    """ Adds an object to the tree, expanding the root node if necessary. """
    def add_object(self, obj):
        self.root_node = MappingTree.__insert_node(self.root_node, obj, self.max_leaves)

    """ Returns a list of all the objects found within the specified bounds. """
    def find_objects_within_bounds(self, bounds):
        return MappingTree.__search_node_for_objects(self.root_node, bounds)

    @staticmethod
    def __search_node_for_objects(node, bounds):
        objects = list()
        if node.has_leaves:
            for leaf in node.children:
                if MappingBounds.does_intersect(leaf.bounds, bounds):
                    objects.append(leaf)
        else:
            for branch in node.children:
                objects = objects + MappingTree.__search_node_for_objects(branch, bounds)
        return objects

    @staticmethod
    def __insert_node(parent, node, max_leaves):
        # Check to make sure should be placed under parent (i.e. is it in that quadrant of our map)
        if MappingBounds.does_intersect(node.bounds, parent.bounds) is False:
            # Nope, node doesn't fall within this quadrant of our map
            if parent.parent is not None:
                # Uh-oh, the node shouldn't be added to this parent, and our parent isn't the root node, so we can't
                # just expand that
                raise Exception('Tried to insert node outside of bounds of non-root node')

            while MappingBounds.does_intersect(parent.bounds, node.bounds) is False:
                # Expand root node
                parent = MappingTree.__expand_root_in_direction_of_child(parent, node)
            MappingTree.__insert_node(parent, node, max_leaves)

        elif parent.has_leaves:
            # Child nodes go here, not on some lower branch
            if len(parent.children) >= max_leaves:
                # This branch is full, so we need to create new sub-branches
                children = parent.children

                quadrants = MappingTree.__divide_node_bounds(parent)
                parent.children = quadrants
                parent.has_leaves = 0
                for child in children:
                    MappingTree.__insert_node(parent, child, max_leaves)
            else:
                # Add the new node
                parent.children.append(node)

        # Node is just branch with other branches (sub-quadrants) for children. Add node to correct child branch.
        else:
            # TODO Just treating the leaf as a point right now. Might want to change.
            center_x = (node.bounds.x1 + node.bounds.x2) / 2.0
            center_y = (node.bounds.y1 + node.bounds.y2) / 2.0
            center_point = MappingBounds(center_x, center_y, center_x, center_y)
            # Branch is a sub-quadrant of parent
            for branch in parent.children:
                if MappingBounds.does_intersect(center_point, branch.bounds):
                    MappingTree.__insert_node(branch, node, max_leaves)
                    break

        return parent

    """ Creates a larger root node sized such that previous root is one quadrant. Child parameter is used to pick expansion
        direction, but child is NOT actually inserted into the node (it may not belong there). Resulting node will be
        twice as large in both x and y directions. """
    @staticmethod
    def __expand_root_in_direction_of_child(root, child):
        old_width = root.bounds.x2 - root.bounds.x1
        old_height = root.bounds.y2 - root.bounds.y1

        # Check if we need to expand horizontally
        if child.bounds.x1 < root.bounds.x1:  # Need to expand left
            new_x2 = root.bounds.x2
            new_x1 = new_x2 - 2*old_width
        else:  # Expand right
            new_x1 = root.bounds.x1
            new_x2 = new_x1 + 2*old_width

        # Check if we need to expand vertically
        if child.bounds.y1 < root.bounds.y1:  # Need to expand down
            new_y2 = root.bounds.y2
            new_y1 = new_y2 + 2*old_height
        else:  # Expand up
            new_y1 = root.bounds.y1
            new_y2 = new_y1 + 2*old_height

        new_bounds = MappingBounds(new_x1, new_x2, new_y1, new_y2)
        new_root = MappingNode(None, new_bounds)
        new_root.children = MappingTree.__divide_node_bounds(new_root)
        new_root.has_leaves = False

        # Put the old root where it belongs under the new one
        center_x = (root.bounds.x1 + root.bounds.x2) / 2.0
        center_y = (root.bounds.y1 + root.bounds.y2) / 2.0
        center_point = MappingBounds(center_x, center_x, center_y, center_y)
        for i in range(4):
            branch = new_root.children[i]
            if MappingBounds.does_intersect(center_point, branch.bounds):
                new_root.children[i] = root
                break

        return new_root

    """ Takes a node, looks at its bounds, splits it into four quadrants, and returns
        a list of the four quadrants. Result ordered according to Cartesian quadrant numbers. """
    @staticmethod
    def __divide_node_bounds(node):
        result = list()
        x1 = node.bounds.x1
        x2 = node.bounds.x2
        y1 = node.bounds.y1
        y2 = node.bounds.y2

        # Create upper right quadrant (I)
        x_left = (x1 + x2) / 2.0
        x_right = x2
        y_top = y2
        y_bottom = (y1 + y2) / 2.0
        bounds = MappingBounds(x_left, x_right, y_bottom, y_top)
        new_node = MappingNode(node, bounds)
        result.append(new_node)

        # Create upper left quadrant (II)
        x_left = x1
        x_right = (x1 + x2) / 2.0
        y_top = y2
        y_bottom = (y1 + y2) / 2.0
        bounds = MappingBounds(x_left, x_right, y_bottom, y_top)
        new_node = MappingNode(node, bounds)
        result.append(new_node)

        # Create lower left quadrant (III)
        x_left = x1
        x_right = (x1 + x2) / 2.0
        y_top = (y1 + y2) / 2.0
        y_bottom = y1
        bounds = MappingBounds(x_left, x_right, y_bottom, y_top)
        new_node = MappingNode(node, bounds)
        result.append(new_node)

        # Create lower right quadrant (IV)
        x_left = (x1 + x2) / 2.0
        x_right = x2
        y_top = (y1 + y2) / 2.0
        y_bottom = y1
        bounds = MappingBounds(x_left, x_right, y_bottom, y_top)
        new_node = MappingNode(node, bounds)
        result.append(new_node)

        return result
