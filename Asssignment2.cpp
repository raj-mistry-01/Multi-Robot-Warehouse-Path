#include <bits/stdc++.h>
using namespace std;

struct Edge {
    int target;
    int weight;
};

struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto hash1 = hash<T1>()(p.first);
        auto hash2 = hash<T2>()(p.second);
        return hash1 ^ hash2;
    }
};

void addEdge(vector<vector<Edge>>& graph, int u, int v, int weight, const unordered_set<int>& blocked_nodes, const unordered_set<pair<int, int>, hash_pair>& blocked_edges) {
    if (blocked_nodes.find(v) == blocked_nodes.end() && blocked_edges.find({u, v}) == blocked_edges.end() && blocked_edges.find({v, u}) == blocked_edges.end()) {
        graph[u].push_back({v, weight});
    }
}

void generateDenseGraph(vector<vector<Edge>>& graph, int n, const unordered_set<int>& blocked_nodes, const unordered_set<pair<int, int>, hash_pair>& blocked_edges) {
    srand(time(0));
    int maxEdgesPerNode = 6;

    for (int u = 0; u < n; ++u) {
        int edgesAdded = 0;
        while (edgesAdded < maxEdgesPerNode) {
            int v = rand() % n;
            if (u != v) {
                int weight = rand() % 10 + 1;
                addEdge(graph, u, v, weight, blocked_nodes, blocked_edges);
                edgesAdded++;
            }
        }
    }
}

void generateDotFile(const vector<vector<Edge>>& graph, const vector<vector<int>>& paths, const vector<int>& source, const vector<int>& destination, const unordered_set<pair<int, int>, hash_pair>& blocked_edges, const string& filename) {
    ofstream file(filename);
    file << "digraph WarehouseGraph {" << endl;

    vector<string> colors = {"red", "blue", "green", "purple", "orange"};
    unordered_set<string> used_colors;

    // Color robot paths
    for (int i = 0; i < paths.size(); i++) {
        for (size_t j = 0; j < paths[i].size() - 1; j++) {
            int u = paths[i][j];
            int v = paths[i][j + 1];
            file << u << " -> " << v << " [color=" << colors[i % colors.size()] << ", fontcolor=" << colors[i % colors.size()] << ", penwidth=2];" << endl;
        }
        used_colors.insert(colors[i % colors.size()]);
    }

    // Determine an unused color for blocked edges
    string blocked_edge_color = "";
    for (const auto& color : colors) {
        if (used_colors.find(color) == used_colors.end()) {
            blocked_edge_color = color;
            break;
        }
    }

    // Color blocked edges
    for (const auto& blocked_edge : blocked_edges) {
        int u = blocked_edge.first;
        int v = blocked_edge.second;
        file << u << " -> " << v << " [color=" << blocked_edge_color << ", fontcolor=" << blocked_edge_color << ", penwidth=3, style=dashed];" << endl;
    }

    // Draw the nodes (sources and destinations)
    for (int i = 0; i < source.size(); i++) {
        file << source[i] << " [shape=circle, style=filled, color=" << colors[i % colors.size()] << ", fontcolor=white, label=\"S" << i << "\"];" << endl;
        file << destination[i] << " [shape=circle, style=filled, color=" << colors[i % colors.size()] << ", fontcolor=white, label=\"D" << i << "\"];" << endl;
    }

    // Draw the rest of the edges
    for (int u = 0; u < graph.size(); u++) {
        for (const auto& edge : graph[u]) {
            int v = edge.target;
            int weight = edge.weight;
            file << u << " -> " << v << " [label=\"" << weight << "\"];" << endl;
        }
    }

    file << "}" << endl;
}

vector<int> dijkstra(int source, int n, vector<vector<Edge>>& graph, unordered_set<pair<int, int>, hash_pair>& blocked_edges, vector<int>& parent) {
    vector<int> distance(n, INT_MAX);
    parent.assign(n, -1);
    distance[source] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});

    while (!pq.empty()) {
        int dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (dist > distance[u]) continue;

        for (auto& edge : graph[u]) {
            int v = edge.target;
            int weight = edge.weight;

            if (dist + weight < distance[v] && blocked_edges.find({u, v}) == blocked_edges.end()) {
                distance[v] = dist + weight;
                parent[v] = u;
                pq.push({distance[v], v});
            }
        }
    }

    return distance;
}

vector<int> getPath(int source, int target, const vector<int>& parent) {
    vector<int> path;
    for (int v = target; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

int main() {
    int n = 50;
    vector<vector<Edge>> graph(n);

    unordered_set<int> blocked_nodes = {4, 8, 10, 12, 15, 17, 19, 22, 24, 27, 29, 31, 33, 36, 38, 40, 42, 45, 47, 49};
    unordered_set<pair<int, int>, hash_pair> blocked_edges = {
        {1, 4}, {2, 5}, {3, 6}, {4, 7}, {5, 9}, {6, 11}, {7, 13}, {8, 14}, {9, 16},
        {10, 18}, {11, 17}, {12, 20}, {13, 21}, {14, 22}, {15, 23}, {16, 24}, {17, 25},
        {18, 26}, {19, 28}, {20, 29}, {21, 30}, {22, 31}, {23, 32}, {24, 33}, {25, 34},
        {26, 35}, {27, 36}, {28, 37}, {29, 38}, {30, 39}, {31, 0}, {32, 4}, {33, 8},
        {34, 12}, {35, 16}, {36, 20}, {37, 24}, {38, 28}, {39, 32}
    };

    generateDenseGraph(graph, n, blocked_nodes, blocked_edges);

    vector<int> sources = {0, 5, 10, 15, 20};
    vector<int> destinations = {40, 35, 30, 25, 45};
    vector<int> parent(n);
    vector<vector<int>> paths;

    for (int i = 0; i < sources.size(); i++) {
        vector<int> dist = dijkstra(sources[i], n, graph, blocked_edges, parent);
        vector<int> path = getPath(sources[i], destinations[i], parent);
        paths.push_back(path);
    }

    generateDotFile(graph, paths, sources, destinations, blocked_edges, "warehouse_paths2");

    cout << "Dot file generated with all robot paths highlighted!" << endl;

    return 0;
}
