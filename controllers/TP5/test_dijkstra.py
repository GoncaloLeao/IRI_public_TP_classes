"""
A simple example to test the Dijkstra algorithm.
By: Gonçalo Leão
"""
from controllers.TP5.graph import Graph
from controllers.TP5.vertex_edge import Vertex


def test_dijkstra():
    myGraph: Graph = Graph()

    for i in range(1, 8):
        assert myGraph.add_vertex(i)

    assert myGraph.add_edge(1, 2, 2)
    assert myGraph.add_edge(1, 4, 7)
    assert myGraph.add_edge(2, 4, 3)
    assert myGraph.add_edge(2, 5, 5)
    assert myGraph.add_edge(3, 1, 2)
    assert myGraph.add_edge(3, 6, 5)
    assert myGraph.add_edge(4, 3, 1)
    assert myGraph.add_edge(4, 5, 1)
    assert myGraph.add_edge(4, 6, 6)
    assert myGraph.add_edge(4, 7, 4)
    assert myGraph.add_edge(5, 7, 2)
    assert myGraph.add_edge(6, 4, 3)
    assert myGraph.add_edge(7, 6, 4)

    myGraph.dijkstra(3)
    check_all_paths(myGraph, "1<-3|2<-1|3<-|4<-2|5<-4|6<-3|7<-5|")

    myGraph.dijkstra(1)
    check_all_paths(myGraph, "1<-|2<-1|3<-4|4<-2|5<-4|6<-4|7<-5|")
    check_single_path(myGraph.get_path(1, 7), "1 2 4 5 7 ")

    myGraph.dijkstra(5)
    check_single_path(myGraph.get_path(5, 6), "5 7 6 ")

    myGraph.dijkstra(7)
    check_single_path(myGraph.get_path(7, 1), "7 6 4 3 1 ")


def check_all_paths(graph: Graph, expected_str: str) -> None:
    actual_str: str = ""
    vs: [Vertex] = graph.vertex_set
    for i in range(len(vs)):
        actual_str += (str(vs[i].id) + "<-")
        if vs[i].path is not None:
            actual_str += str(vs[i].path.origin.id)
        actual_str += "|"
    assert actual_str == expected_str, "check_all_paths: actual_str = " + actual_str + ", but expected_str = " + expected_str


def check_single_path(path: [Vertex], expected_str: str) -> None:
    actual_str: str = ""
    for i in range(len(path)):
        actual_str += (str(path[i].id) + " ")
    assert actual_str == expected_str, "check_single_path: actual_str = " + actual_str + ", but expected_str = " + expected_str


if __name__ == '__main__':
    test_dijkstra()
