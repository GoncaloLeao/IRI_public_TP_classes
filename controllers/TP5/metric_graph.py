"""
A simple implementation of a metric graph, including a utility graph for visualization.
By: Gonçalo Leão
"""
from __future__ import annotations
import math
from typing import Union, Callable

import networkx as nx

from controllers.TP5.graph import Graph
from controllers.TP5.vertex_edge import Vertex, Edge
from controllers.TP5.mutable_priority_queue import MutablePriorityQueue


class VertexInfo:
    def __init__(self, id: int, x: float, y: float):
        self.id: int = id
        self.x: float = x
        self.y: float = y

class MetricGraph(Graph):
    def __init__(self):
        self.vertex_set: [Vertex] = []
        self.vertices_info: [VertexInfo] = []
        self.visual_graph: nx.Graph = nx.Graph()

    def add_vertex(self, id: int, pos: (float, float), color: str) -> bool:
        if super().add_vertex(id):
            self.vertices_info.append(VertexInfo(id, pos[0], pos[1]))
            self.visual_graph.add_node(id, pos=pos, color=color)
            return True
        return False


    def add_edge(self, origin: int, dest: int, weight: float, set_as_path: bool = False) -> bool:
        if super().add_edge(origin, dest, weight, set_as_path):
            self.visual_graph.add_edge(origin, dest)
            return True
        return False
