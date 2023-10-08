#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

const int INF = numeric_limits<int>::max();

// Class to represent a weighted directed graph
class Graph {
public:
    int vertices;
    vector<vector<pair<int, int>>> adjacencyList;

    Graph(int V) : vertices(V), adjacencyList(V) {}

    // Add an edge to the graph
    void addEdge(int u, int v, int weight) {
        adjacencyList[u].push_back(make_pair(v, weight));
    }

    // Dijkstra's algorithm to find the shortest path
    void dijkstra(int start) {
        vector<int> dist(vertices, INF);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

        dist[start] = 0;
        pq.push(make_pair(0, start));

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (const auto& neighbor : adjacencyList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push(make_pair(dist[v], v));
                }
            }
        }

        // Print the shortest distances
        cout << "Shortest distances from vertex " << start << ":\n";
        for (int i = 0; i < vertices; ++i) {
            if (dist[i] == INF)
                cout << "Vertex " << i << ": Not reachable\n";
            else
                cout << "Vertex " << i << ": " << dist[i] << "\n";
        }
    }
};

int main() {
    // Create a graph
    Graph g(6);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 4, 3);
    g.addEdge(3, 5, 1);
    g.addEdge(4, 3, 2);
    g.addEdge(4, 5, 5);

    // Run Dijkstra's algorithm from vertex 0
    g.dijkstra(0);

    return 0;
}
