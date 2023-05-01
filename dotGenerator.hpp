#include <iostream>
#include <fstream>
#include <string>

using namespace std;

void generate_dot(int **adj_matrix, int num_nodes, string filename) {
    ofstream fout(filename);
    fout << "digraph G {\n";
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            if (adj_matrix[i][j] > 0) {
                fout << "  " << i+1 << " -> " << j+1 << " [label=\"" << adj_matrix[i][j] << "\"];\n";
            }
        }
    }
    fout << "}";
    fout.close();
}