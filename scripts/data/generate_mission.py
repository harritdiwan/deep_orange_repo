import matplotlib.pyplot as plt
import networkx as nx
import os
import sys

# Change working directory
script_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(os.path.dirname(script_path))
sys.path.append(os.getcwd())

import constants
from controller.mission.node import node
from mapping.map import map
from mapping.point_picker import PointPicker


def callback(point):
    print ("X = ", point[0])
    print ("Y = ", point[1])

def selected(points):
    if points is None: return
    l = len(points)

    snode = node(points[0][0], 0, [points[0][1]]) # start
    pnode = snode # prev

    for index, val in enumerate(points):
        if index < (l - 1):
            nxt = points[index + 1]
            nnode = node(nxt[0], index + 1, [nxt[1]]) # next
            DG.add_weighted_edges_from([(pnode, nnode, pnode.dist(nnode))])
            pnode = nnode # update prev

if __name__ == "__main__":
    if not os.path.isfile('data/mission.pkl'):
        DG = nx.DiGraph()

        mp = map()

        mp.lt.join()
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.set_title('Waypoints picker')

        pk = PointPicker(fig,ax1,3,sc=callback)
        line, = ax1.plot([wp[0] for wp in mp._waypoints], [constants.MH+constants.TR-wp[1] for wp in mp._waypoints], 'ko', picker=5)

        fig.canvas.mpl_connect('pick_event', pk.onpick)

        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

        if len(pk.selected) > 0:
            selected(pk.data)
        else:
            selected(pk.all)

        if DG.number_of_nodes() > 0:
            fn = filter(lambda n: n.name == 0, DG.nodes())[0]
            ln = filter(lambda n: n.name == len(DG.nodes()) - 1, DG.nodes())[0]
            DG.add_weighted_edges_from([(ln, fn, ln.dist(fn))])

            nx.write_gpickle(DG, "data/mission.pkl")
    else:
        DG = nx.read_gpickle("data/mission.pkl")

        for idx, node in enumerate(sorted(DG.nodes(), key=lambda n: n.name)):
            nx.draw(DG, dict((n, n.pos) for n in DG.nodes()),
                    node_color=["g" if n.name == idx else "y" for n in DG.nodes()])
            plt.show(block=False)
            node.speed_limit = int(input("Insert speed limit: "))
            plt.close()

        # nx.draw(DG, dict((n, n.pos) for n in DG.nodes()), node_color=["g" if False else "y" for n in DG.nodes()])
        # plt.show(block=True)

        nx.write_gpickle(DG, "data/mission.pkl")
