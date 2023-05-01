#include "graph.hpp"
#include <iostream>

using namespace std;

Graph::Graph(int num_vertices) {
  int i, j;
  // Memory Allocation
  this->City_Adjacency = new int *[num_vertices];
  for (i = 0; i < num_vertices; i++) {
    City_Adjacency[i] = new int[num_vertices];
  }
  // Initializing the adjacency matrix
  for (i = 0; i < num_vertices; i++) {
    for (j = 0; j < num_vertices; j++) {
        this->City_Adjacency[i][j] = -1;
    }
  }
  // Setting the number of nodes
  this->num_nodes = num_vertices;
}

Graph::~Graph() {
  int i;
  for (i = 0; i < this->num_nodes; i++) {
    delete[] City_Adjacency[i];
  }
  delete[] this->City_Adjacency;
}