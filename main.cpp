#include "dotGenerator.hpp"
#include "graph.hpp"
#include <fstream>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>

using namespace std;



/*
********* Lista de Algoritmos em Grafos *************
-> BFS (Walk through the graph by layers)
-> DFS (Walk through the graph as deep as possible)
-> Kosaraju (Find Strongly Connected Components)
-> Ordenação Topologica (This is a custom DFS algorithm that can verify whether a graph is a DAG (directed acyclic graph) or find its topological order)
-> 2-Satisfability Solver (Find or check if there is a solution for expressions like S = (x+y)(!z+x)(!y+z))
-> Dijkstra (Find the Shortest Path from a node to all other nodes)
-> Bellman-Ford Algorithm (Find the Shortest Path from a node to all other nodes but the graph may contains negative edges)
-> Prim (Spanning Tree)
-> Kruskal
-> Reverse Delete
-> Boruvka
*/


//********** Algoritmos de BUSCA: *****************
// Complexidade = O(n+m) ou O(V+E)
void BFS(Graph& G) {
  queue<int> q;
  vector<bool> visited;
  visited.resize(G.num_nodes);
  q.push(0);
  visited[0] = true;
  int vertice;
  while (!q.empty()) {
    vertice = q.front();
    q.pop();
    cout <<vertice+1 << endl;
    for(int i = 0; i < G.num_nodes; i++) {
      if(G.City_Adjacency[i][vertice] != -1 && !visited[i]) {
        visited[i] = true; 
        q.push(i);
      }
    }
  }  
}

// Complexidade = O(n+m) ou O(V+E)
void DFS_recursiva(Graph& G, int s, int* visited) {
  cout << s+1 << endl;
  visited[s] = 1;
  for(int i = 0; i < G.num_nodes; i++) {
    if(G.City_Adjacency[s][i] != -1 && !visited[i])
      DFS_recursiva(G, i, visited);
  }
}

/*
********* COMPONENTES FORTEMENTE CONECTADOS ***************
Para descobrir se um um grafo direcionado G é fortemente conectado podemos utilizr o algoritmo abaixo:
-> Pegue um vertice v qualquer de G
-> Rode uma BFS com origem v no grafo G
-> Rode uma BFS com origem v no grafo G', sendo G' o grafo G inverso
-> O grafo G é fortemente conectado se todos os nós são alcançaveis nas duas execuções da BFS

O algoritmo possui complexidade O(m+n)
*/

/*
  Algoritmo de Kosaraju: um algoritmo que acha os componentes fortemente conectados de um grafo G.
  Consiste em:
  -> Rode uma DFS para computar o tempo de termino de todos os vértices de G (O(m+n))
  -> Armazene os vertices pelos tempos de termino em ordem decrescente, por exemplo em uma pilha
  -> Compute G', G transposto/inverso O(m+v)
  -> Rode uma DFS nos vértices armazenados seguindo a ordem decrescente (O(m+n))
  -> Cada DFS rodado no item anterior gera um componente fortemente conectado
  

  O algoritmo tem custo O(m+n)
*/
void getFinishTime(Graph& G, int s, int* visited, stack<int>& order) {
  visited[s] = 1;
  for(int i = 0; i < G.num_nodes; i++) {
    if(G.City_Adjacency[s][i] != -1 && !visited[i])
      getFinishTime(G, i, visited, order);
  }
  order.push(s);
}
void printSCC(Graph& G, int s, int* visited) {
  visited[s] = 1;
  cout << s << " ";
  for(int i = 0; i < G.num_nodes; i++) {
    if(G.City_Adjacency[i][s] != -1 && visited[i] == 0) {
      printSCC(G, i, visited);
    }
  }
}
void kosaraju(Graph& G) {
  int* visited = new int[G.num_nodes];
  stack<int> order;
  for(int i = 0; i < G.num_nodes; i++)
    visited[i] = 0;

  // Computing finish times
  for(int i = 0; i < G.num_nodes; i++) {
    if(!visited[i])
      getFinishTime(G, i, visited, order);
  }
  for(int i = 0; i < G.num_nodes; i++)
    visited[i] = 0;

  // Computing SCC's
  int aux;
  while(!order.empty()) {
    aux = order.top();
    order.pop();
    if(!visited[aux]) {
      printSCC(G, aux, visited);
      cout << endl;
    }
  }
  delete [] visited;
}

/*
*********** ORDENAÇÃO TOPOLOGICA ************
DAG (Directed Acyclic Graph): É um grafo direcionado que não possui ciclos
-> Se um grafo G é um DAG, entao G possui ordenação topologica
-> Se um grafo G possui ordenação topologica, então G é um DAG
-> Se um grafo G é um DAG, então ele possui pelo menos 1 vértice que não possui arestas de entrada
** O vértice que inicia a ordenação topologica deve ser um vértice que não possui aresta de entrada

// Algoritmos para encontrar ordenação topologica

Primeiro algoritmo (O(m+n)):
Enquanto todos os nós não forem visitados e possuir algumm vertice sem aresta de entrada:
Encontre todos os nós que não possuem arestas de entrada;
Selecione um desses nós e remova-o do grafo, junto de suas arestas de saida
-> A ordem que os nós foram removidos formam uma ordem topologica válida;
-> Se o grafo não for um DAG a saída não contará com todos os nós.

// Algoritmo usando DFS:
Rode uma DFS no grafo G e grave os tempos de termino de cada vertice
A ordem decrescente dos tempos de termino foram uma ordenação topologica.
Caso um vértice que já visitado mas ainda não tem tempo de termino for visitado novamente, o grafo G não é um DAG
e não possui ordenação topologica
*/

int topologicalDFS(Graph& G, int* visited, int v, stack<int>& order) {
  visited[v] = 1;
  for(int i = 0; i < G.num_nodes; i++) {
    if(G.City_Adjacency[v][i] != -1 && !visited[i]){
      if(!topologicalDFS(G, visited, i, order)) {
        return 0;
      }
    }
    else if(G.City_Adjacency[v][i] != -1 && visited[i] == 1)
      return false; 
  }
  visited[v] = 2; //Finish Time is Calculated
  order.push(v);
  return true;
}

// Return if G is a DAG or not. Print the topological order if G is a DAG
int topologicalSorting (Graph& G) {
  int* visited = new int[G.num_nodes];
  for(int i = 0; i < G.num_nodes; i++)
    visited[i] = 0;
  stack<int> order;


  for(int i = 0; i < G.num_nodes; i++) {
    if(!visited[i]) {
      if(!topologicalDFS(G, visited, i, order))
        return 0;
    }
  }

  // Print Topological Order
  while(!order.empty()) {
    cout << order.top() << " ";
    order.pop();
  }
  cout << endl;

  delete[] visited;
  return 1;
}

/*
********* 2-Satisfability Problem *************
Busca indicar se expressões booleanas como a indicada abaixo possuem soluções:
S = (x1 + x2)(x1 + !x3)(x2 + x4)(!x2 + x3)

Algoritmo solução:
Considere S uma expressão booleana com N variáveis
-> Crie um grafo direcionado G com 2N vértices da seguinte forma:
  * Para cada variável Xi, adicione dois nós em G: Xi e !Xi
  * Para cada clausula (Xi + Yi) adicione duas arestas: (!Xi -> Yi) e (!Yi -> Xi). Que representam
    cenários onde a clausula é verdadeira.
-> S não possui solução sse:
  * Existe um caminho de X para !X e
  * Existe um caminho de !X pra X, ou seja existe um ciclo.
-> Para cada variável X de S:
  * Rode uma BFS para testar se há um caminho de X para !X
  * Rode uma BFS para testar se há um caminho de !X para X
  * Caso os dois sejam verdadeiros, S não posui solução
  * Se para toda variável o teste deu falso, então S possui solução.

*/


/* ___________________________________________________________________________________________________________________________

****************** CAMINHOS MAIS CURTOS ******************************
O problema de caminhos curtos em grafos é um problema clássico que consiste em encontrar o caminho
mais curto entre dois vértices de um Grafo ponderado G, seja ele direcionado ou não.
*/

/*
  ************************** DIJKSTRA ALGORITHM ******************************
  Um algoritmo que encontra o menor caminho de um vértice s de um grafo G para todos os outros vértices do grafo.
  -> O algotitmo utiliza uma euristica gulosa, na qual escolhe sempre o vértice com menor distância estimada
     e atualiza as distâncias de seus vizinhos.
  -> O algoritmo utiliza uma priority queue para manter os vértices a serem vistidatos em ordem crescente.
     Essa fila pode ser implementada usando uma heap binária por exemplo.
  -> O algoritmo assume que as arestas possuem pesos não negativos
  -> Caso exista um peso negativo no grafo, o algoritmo entra em loop infinito
  -> A Complexidade do algoritmo de Dijkstra é de O(E + V*log(V)). Sendo muito eficiente para grafos densos.

  Algoritmo:
    -> Inicialize a distância estimada para cada vértice como infinita;
    -> Inicialize a distância estimada do vértice de origem como 0;
    -> Insira todos os vértices do grafo em uma fila de prioridade tendo a distância como chave de sorting;
    -> Enquanto a fila não estiver vazia, remova o vértice com a menor distância estimada;
    -> Para cada vértice adjacente ao vértice removido, atualize a distância estimada;
    -> Quando a fila estiver vazia, a distância estimada representará a menor distância até o vértice de origem
*/

// Relembrando: complexidade = O(E + V*log(V))
void Dijkstra(Graph& G, int source) {
  priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> p_queue;
  int* distance = new int[G.num_nodes];

  // Setting distances
  for(int i = 0; i < G.num_nodes; i++)
    distance[i] = INT32_MAX;
  distance[source] = 0;

  // Pushing nodes to the queue
  for(int i = 0; i < G.num_nodes; i++) {
    p_queue.push({distance[i], i});
  }

  int node;
  while(!p_queue.empty()){
    node = p_queue.top().second;
    p_queue.pop();
    for(int i = 0; i < G.num_nodes; i++) {
      if(G.City_Adjacency[node][i] != -1) {
        if(distance[i] > distance[node] + G.City_Adjacency[node][i]) {
          distance[i] = distance[node] + G.City_Adjacency[node][i];
          p_queue.push({distance[i], i});
        }
      }
    }
  }


  // Printing Shortest Distances
  for(int i = 0; i < G.num_nodes; i++) {
    if(distance[i] == INT32_MAX)
      distance[i] = -1;
  }
  cout << "Shortest Distances Genereted! See in djta_dis.dot\n";
  ofstream fout("djta_dis.dot");
  fout << "digraph G {\n";
  for (int i = 0; i < G.num_nodes; i++) {
      for (int j = 0; j < G.num_nodes; j++) {
          if (G.City_Adjacency[i][j] > 0) {
              fout << "  \"" << i+1 << " (" << distance[i] << ")\" " <<  " -> \"" << j+1 << " (" << distance[j] << ")\" " << " [label=\"" << G.City_Adjacency[i][j] << "\"];\n";
          }
      }
  }
  fout << "}";
  fout.close();
  delete[] distance;
}

/*
************************* Bellman-Ford Algorithm *******************************
Algoritmo para encontrar o menor caminho entre um vértice de origem para todos os outros vértices do grafo.
-> Aceita arestas com peso negativo
-> Não é um algoritmo Guloso
-> Se há um ciclo com peso negativo retorna falso, indicando que não existe um caminho mais curto. Caso contrário, retorna o caminho mais curto.

Conceito: Relaxar uma aresta (A->B) consiste em verificar se a distância atual do vertice B é maior que a distância acumulada do vértice A mais o peso da aresta (A->B).
          Caso isso seja verdadeiro, a distância do vértice B é atualizada para dis(A) + peso(A->B); 

O algoritmo inicia com a definição da distância de cada vértice como infinita, exceto pelo vértice de origem, que tem distância 0. 
Em seguida, relaxa as arestas repetidamente, atualizando a distância mínima dos vértices até que nenhuma distância possa mais ser melhorada.

Complexidade = O(VE);
*/
void bellman_ford(Graph& G, int source) {
  int* distance = new int[G.num_nodes];
  // Setting distances
  for(int i = 0; i < G.num_nodes; i++)
    distance[i] = INT32_MAX;
  distance[source] = 0;

  // Relaxing all edges G num_nodes - 1 times
  for (int k = 0; k < G.num_nodes; k++) {
    for(int i = 0; i < G.num_nodes; i++) {
      for(int j = 0; j < G.num_nodes; j++) {
        if(G.City_Adjacency[i][j] != -1 && distance[i] != INT32_MAX) {
          if(distance[j] > distance[i] + G.City_Adjacency[i][j])
            distance[j] = distance[i] + G.City_Adjacency[i][j];
        }
      }
    }
  }

  // Checking for negative cycles
  for(int i = 0; i < G.num_nodes; i++) {
    for(int j = 0; j < G.num_nodes; j++) {
      if(G.City_Adjacency[i][j] != -1 && distance[i] != INT32_MAX) {
        if(distance[j] > distance[i] + G.City_Adjacency[i][j])
          cout << "Possui um ciclo negativo\n";
      }   
    }
  }
  // Printing Shortest Distances
  cout << "Shortest Distances:\n";
  for(int i = 0; i < G.num_nodes; i++) {
    cout << i+1 << ": " << distance[i] << endl;
  }
  delete[] distance;
}

/*
************************ MINIMUM SPANNING TREE (MST) ***************************************
-> Conceito de Grafos Não-Direcionados
Conceitos:
-> cut: É uma partição dos nós de um grafo em dois subconjunto não-vazios, tal que a soma dos dois
     subconjuntos gera o grafo original.
-> cutset: O cutset de um cut S consiste no conjunto de arestas que conecta o cut S com o outro subconjunto/cut.
            Ou seja, é o conjunto de arestas que conecta o cut S com o restante do grafo.
-> A interceção de um cutset e um ciclo em um grafo G possui um número par de arestas.

-> ÁRVORE GERADORA: Uma árvóre geradora de um grafo G é um subgrafo de G que:
    * Contém todos os nós de G;
    * Obviamente é uma árvore, ou seja, não possui ciclos;
    * Por implicação, possui V-1 arestas;
    * É CONEXA.
    * Remover qualquer aresta desconecta a árvore (Mimimally Connected)
    * Adicionar qualquer aresta forma um ciclo (Maximally Acyclic)

-> ÁRVORE GERADORA MiNÍMA (AGM/MST): É uma árvore geradora com a soma das arestas minimizadas. Ou seja, é a árvore geradora com a menor soma de arestas de um grafo G.

-> Ciclo Fundamental: Ao adicionar qualquer aresta em uma árvore geradora T irá se formar um ciclo C. 
                      Retirando QUALQUER aresta de C, não necessáriamente a adicionada no passo anterior, iremos ter uma nova árvore geradora.
-> Cutset Fudamental: Se T é uma árvore geradora de G, ao retirar uma aresta de T eu obtenho 2 componentes conectados. E ao adicionar qualquer
                      aresta eu gero outra árvore geradora.

**************************************** Algoritmo Guloso *************************************
--> Red Rule: Seja C um ciclo sem arestas vermelhas. Selecione a aresta de maior peso que não poussi cor pinte-a de vermelho.
--> Blue Rule:Seja D um cutset sem arestas azuis. Selecine a aresta de menor peso que não possui cor e pinte-a de azul.

-> Algoritmo:
  * Aplique a red rule e a blue rule até todas arestas estarem coloridas.
  * As arestas azuis formam um MST.
  * Caso n-1 arestas já estão pintadas de azul, o algoritmo pode ser interrompido.
  

*************************************** Algoritmo de Prim **************************************
Algoritmo para encontrar a MST de um grafo G.
-> Seja S um conjunto de vértices e T um conjunto de arestas.
-> Escolha um vértice qualquer de G e adicione em S
-> Realize o seguinte processo n-1 vezes:
   * Adicione à T a aresta E com o menor peso que possua apenas 1 conexão à vértices de S
   * Adicione em S o vértice de E que ainda não estava em S.
-> O grafo G'(S, T) é a MST de G.

A implementação do algorimto de Prim é muito semelhante a do Dijkstra. 



Complexidade O(mlog(n))
*/

// Funcção que seleciona o nó de G mais proximo da MST
int nearestNode(int* mstNodes, int* distance, int N) {
  int nearest_node;
  int min = INT32_MAX;
  for(int i = 0; i < N; i++) {
    if(!mstNodes[i] && distance[i] < min)
      min = distance[i], nearest_node = i;
  }
  return nearest_node;
}

void prim(Graph& G) {
  // Representa as arestas da MST
  int* mstEdges = new int[G.num_nodes];
  // Armazena os nós já presentes na MST
  int* mstNodes = new int[G.num_nodes];
  // Vetor de distancias de nós fora da MST até algum nó da MST
  int* distance = new int[G.num_nodes];

  for(int i = 0; i < G.num_nodes; i++) {
    mstNodes[i] = 0;
    distance[i] = INT32_MAX;
  }

  distance[0] = 0;
  mstEdges[0] = -1;

  // V-1 iterações
  for(int i = 0; i < G.num_nodes; i++) {
    // Selecionando o vértice de G mais próximo da MST
    int v = nearestNode(mstNodes, distance, G.num_nodes);
    mstNodes[v] = 1;
    for(int j = 0; j < G.num_nodes; j++) {
      if(G.City_Adjacency[v][j] != -1 && !mstNodes[j] && distance[j] > G.City_Adjacency[v][j]) {
        mstEdges[j] = v;
        distance[j] = G.City_Adjacency[v][j];
      }
    }
  }

  cout << "MST:\n";
  for(int i = 1; i < G.num_nodes; i++) {
    cout << mstEdges[i]+1 << "--" << i+1 << endl;
  }

  delete [] mstEdges;
  delete [] mstNodes;
  delete [] distance;
}

/*
********************************* Algoritmo de Kruskal *********************************
-> Dado um grafo G, ordene as arestas em ordem crescente de peso.
-> Enquanto não for considerado todas as arestas:
   * Adicione a aresta de menor peso, a não ser que a adição dela na MST forme um ciclo.
-> O Grafo resultante é uma MST

Complexidade: O(mlog(m))
*/

/*
******************************** Reverse Delete ***************************************
Segue a mesma lógica do algoritmo de Kruskal. Porém, ao invés de adicionar as arestas de menor peso as arestas de maior peso são removidas.
-> Dado um grafo G, ordene as arestas em ordem decrescente de peso.
-> Enquanto não for considerado todas as arestas:
   * Remova a aresta de maior peso, a não ser que a remoção dela desconecte o grafo.
-> O Grafo resultante é uma MST

Complexidade: O(mlog(m))
*/


/*
  ****************************** Fluxo em Redes **************************************
  É uma modelagem de um rede em um grafo direcionado G tal que:
    * Cada aresta e posusi um peso não negativo, sendo que o peso representa a capacidade de fluxo(passagem de dados) dessa aresta.
    * Um nó de origem s, tal que s é a fonte do fluxo.
    * Um nó de destino t, tal que t é o fim do fluxo. 
    * Em uma configuração/caminho de fluxo, medimos o quanto de fluxo está passando em uma determinada aresta utilizando uma função F(e), essa função
      simplesmete calcula o fluxo (o fluxo acumulado)  que está passando na aresta e.
    

    Condições para um fluxo:
    1- Condião de capacidade: para toda aresta e, 0 <= F(e) <= c(e). Ou seja, o fluxo de uma aresta deve estar entre 0 e sua capacidade máxima.
    2- Condição de conservação: para todo vertice v (exceto para os nós de origem e destino), a soma dos fluxos das arestas de entrada em v deve ser igual a 
     soma dos fluxos das arestas de saída de v.

    Em suma, temos que o fluxo dentro de uma configuração é constante. O valor do fluxo é igual à soma do fluxo das arestas de saída do nó de origem, e, obviamente,
    também é igual à soma do fluxo das arestas de entrada do nó de destino.


    Problema:
    Dado um grafo de fluxo de rede com uma fonte/origem que pode produzir fluxo ilimitado, qual o valor máximo possivel que o fluxo pode ter para chegar 
    ao nó de destino passando pelas arestas do grafo e sem exceder os limites de capacidade de fluxo de nenhuma aresta?
    
  
  O problema que buscamos solucionar é o seguinte: 

*/
int main(int argc, char const *argv[]) {
  int cities, paths, i, j, dis;

  cin >> cities >> paths;
  Graph g(cities);
  for (int k = 0; k < paths; k++) {
      cin >> i >> j >> dis;
      g.City_Adjacency[i - 1][j - 1] = dis; // -1 because the city range is 1 to N;
  }
  int *visited = new int[g.num_nodes];
  for(int i = 0; i < g.num_nodes; i++)
    visited[i] = 0;
  generate_dot(g.City_Adjacency, g.num_nodes, "map.dot");
  
  // cout << "DFS: \n";
  // DFS_recursiva(g, 0, visited);

  // cout << "BFS: \n";
  // BFS(g);
  // kosaraju(g);
  // cout << topologicalSorting(g) << endl;
  // Dijkstra(g, 0);
  // bellman_ford(g, 0);
  prim(g);
  delete[] visited;
  return 0;
}