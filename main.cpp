#include <iostream>
#include "graph.h"

void ejemplo_1() {
    // Crear el grafo
    utec::graph<char, int, int> g1;
    // Crear los vertices
    g1.push_vertex('A', 10);
    g1.push_vertex('B', 20);
    g1.push_vertex('C', 50);
    g1.push_vertex('D', 30);
    // Crear las aristas
    g1.push_edge({'A', 'B', 2});
    g1.push_edge({'A', 'C', 4});
    g1.push_edge({'B', 'C', 7});
    g1.push_edge({'C', 'D', 5});
    // Leer el valor de un vertice
    std::cout << g1.get_vertex('A') << std::endl;        // 10
    // Leer el peso de una arista
    std::cout << g1.get_edge('B', 'C') << std::endl;    // 7

//    g1.bfs('A', [](auto ))
}


int main() {
    ejemplo_1();
    return 0;
}
