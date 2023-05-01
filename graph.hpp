#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

class Graph {
public:
  Graph(int num_vertices);
  ~Graph();

  /*
  Adjacency Matrix that represents the graph.
    -Each value M[i][j] is the distance between the cities i and j.
    -If M[i][j] is equal -1, the cities isn't connected.
  */
  int** City_Adjacency;

  // Number of nodes
  int num_nodes;
};

#endif // GRAPH_HPP