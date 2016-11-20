from bounds import MappingBounds
from tree import MappingTree
from obstacle import MappingObstacle
from node import MappingNode


class TreeTest:
    """ Just used for testing the tree """

    def __init__(self):
        max_child_items = 5

        starting_bounds = MappingBounds(0, 5, 0, 5)
        tree = MappingTree(starting_bounds, max_child_items)

        x = [-4, -1, 0, 5, 6, 8, 13]
        y = [-2, 2, 3, 7, -2, 4, -1]

        for i in range(len(x)):
            pos = MappingBounds(x[i], x[i], y[i], y[i])
            obstacle = MappingObstacle(pos)
            tree.add_object(obstacle)

        TreeTest.print_branch(tree.root_node, 1)

        search_bounds = MappingBounds(-10, 6.2, -5, 5)
        res = tree.find_objects_within_bounds(search_bounds)
        node = MappingNode(None, search_bounds)
        node.children = res
        node.has_leaves = False
        print 'Search results:'
        TreeTest.print_branch(node, 1)

    @staticmethod
    def print_branch(branch, level):
        indent_leaf = '   ' * level
        indent_branch = '   ' * (level - 1)
        x1 = branch.bounds.x1
        x2 = branch.bounds.x2
        y1 = branch.bounds.y1
        y2 = branch.bounds.y2

        if isinstance(branch, MappingObstacle):
            posx = branch.bounds.x1
            posy = branch.bounds.y1
            print indent_leaf + 'Obstacle: ({0:.2f},{1:.2f})'.format(posx, posy)
        else:
            print indent_branch + 'Branch: {0:.2f} <= x < {1:.2f}, {2:.2f} <= y < {3:.2f}'.format(x1, x2, y1, y2)
            if len(branch.children) > 0:
                if branch.has_leaves:
                    for leaf in branch.children:
                        posx = leaf.bounds.x1
                        posy = leaf.bounds.y1
                        print indent_leaf + 'Leaf: ({0:.2f},{1:.2f})'.format(posx, posy)
                else:
                    for sub_branch in branch.children:
                        TreeTest.print_branch(sub_branch, level+1)
            else:
                print indent_leaf + 'Empty'

if __name__ == '__main__':
    TreeTest()
