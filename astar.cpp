// g++ -o astar.exe astar.cpp
// ./astar
// astar.exe
#include <iostream>
#include <bits/stdc++.h>
#include <windows.h>

#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

using namespace std;
using namespace chrono;

int nodesExpanded = 0, nodesGenerated = 1, skipCount = 0;
milliseconds duration = 0ms;

struct Metrics {
    string heuristic;
    long ET; // Execution time in milliseconds
    int NG;  // Nodes generated
    int NE;  // Nodes expanded
    int D;   // Depth of the tree
    double b_star; // Effective branching factor
};

// Struct to store the search result and associated metrics
struct SearchResult {
    vector<vector<vector<int>>> path;
    Metrics metrics;
};

struct Node {
    vector<vector<int>> state;
    int g; // Cost to reach this node
    int h; // estimated cost to goal
    int f; // Total cost (g + h)
    Node* parent;

    Node(vector<vector<int>> _state, int _g, int _h, Node* _parent = nullptr)
        : state(_state), g(_g), h(_h), parent(_parent) {
        f = g + h;
    }
};

struct vector_hash {
    // Overloads the function call operator to allow instances of vector_hash to be used as hash functions for vector<vector<int>>
    size_t operator()(const vector<vector<int>>& v) const {
        hash<int> hasher; 
        size_t seed = 0; 

        for (auto& i : v) {
            for (int j : i) {
                seed ^= hasher(j) + 0x9e3779b9 + (seed << 6) + (seed >> 2); // Combines the hash of the current element with the seed using bitwise operations and a constant to ensure a good distribution of hash values.
            }
        }

        return seed;
    }
};
void printState(vector<vector<int>> state);

int h1(vector<vector<int>> state, vector<vector<int>> goal){
    // Number of misplaced tiles
    int count = 0;
    for (size_t i = 0; i < state.size(); i++) {
        for (size_t j = 0; j < state[i].size(); j++) {
            if (state[i][j] != goal[i][j]) {
                count++;
            }
        }
    }
}

int h2(vector<vector<int>> state, vector<vector<int>> goal){
    // Sum of distances of tiles from their goal positions
    int sum = 0;
    for (size_t i = 0; i < state.size(); i++) {
        for (size_t j = 0; j < state[i].size(); j++) {
            int num = state[i][j];
            if (num != 0) {
                size_t goal_x, goal_y;
                for (size_t x = 0; x < goal.size(); x++) {
                    for (size_t y = 0; y < goal[x].size(); y++) {
                        if (goal[x][y] == num) {
                            goal_x = x;
                            goal_y = y;
                            break;
                        }
                    }
                }
                sum += abs(static_cast<int>(i) - static_cast<int>(goal_x)) + abs(static_cast<int>(j) - static_cast<int>(goal_y));
            }
        }
    }
}

// Sam's heuristic - Euclidean distance
int h3(vector<vector<int>> state, vector<vector<int>> goal);

// Landry's heuristic
int h4(vector<vector<int>> state, vector<vector<int>> goal);

vector<Node*> generateChildren(Node* current, const vector<vector<int>>& goal, string heuristic) {

    vector<Node*> children;
    int zero_x, zero_y;
    // Find the zero (empty space) in the puzzle
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (current->state[i][j] == 0) {
                zero_x = i;
                zero_y = j;
                break;
            }
        }
    }

    // Possible directions to move the empty space
    vector<pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (auto& dir : directions) {

        int new_x = zero_x + dir.first;
        int new_y = zero_y + dir.second;

        if (new_x >= 0 && new_x < 3 && new_y >= 0 && new_y < 3) {

            vector<vector<int>> new_state = current->state;
            swap(new_state[zero_x][zero_y], new_state[new_x][new_y]);

            int new_h = (heuristic == "h1") ? h1(new_state, goal) : h2(new_state, goal);

            // assign each child a f (n) -value. -- This is done in constructor of Node 
            Node* child = new Node(new_state, current->g + 1, new_h, current);
            children.push_back(child);
        }
    }
    return children;
}

// A* search algorithm
SearchResult  AStarSearch(const vector<vector<int>>& initial, const vector<vector<int>>& goal, string heuristic) {
    auto start = high_resolution_clock::now();

    SearchResult result;
    vector<vector<vector<int>>> path;
    Metrics metrics;

    try {
        priority_queue<pair<int, Node*>, vector<pair<int, Node*>>, greater<pair<int, Node*>>> open; // Make sure this takes the node with the lowest f value
        
        unordered_set<vector<vector<int>>, vector_hash> closed;
        vector<Node*> all_nodes;
        int nodesExpanded = 0, nodesGenerated = 1, skipCount = 0;

        int h_root = (heuristic == "h1") ? h1(initial, goal) : h2(initial, goal);
        Node* root = new Node(initial, 0, h_root);

        all_nodes.push_back(root);
        open.emplace(root->f, root);

        while (!open.empty()) {
            Node* current = open.top().second;
            open.pop();
            nodesExpanded++;

            // if X = goal then return the path from Start to X
            if (current->state == goal) {
                auto end = high_resolution_clock::now();
                long ET = duration_cast<milliseconds>(end - start).count();

                int D = current->g; // Depth of the tree
                double b_star = static_cast<double>(nodesGenerated) / D; // Effective branching factor

                // Populate metrics
                metrics.heuristic = heuristic;
                metrics.ET = ET;
                metrics.NG = nodesGenerated;
                metrics.NE = nodesExpanded;
                metrics.D = D;
                metrics.b_star = b_star;

                while (current != nullptr) {
                    path.push_back(current->state);
                    current = current->parent;
                }
                reverse(path.begin(), path.end());

                cout << "Path found for " << heuristic << endl;
                cout << "Nodes generated: " << nodesGenerated << endl;
                cout << "Nodes expanded: " << nodesExpanded << endl;
                cout << "Nodes skipped: " << skipCount << endl;

                // Clean up memory
                for (Node* node : all_nodes) delete node;

                result.path = path;
                result.metrics = metrics;
                return result;
            }

            // generate children of X.
            vector<Node*> children = generateChildren(current, goal, heuristic);

            for (Node* child : children) {

                // discard children of X if already on open or closed. 
                if (current->parent != nullptr && child->state == current->parent->state) {
                    skipCount++;
                    delete child;
                    continue;
                }

                if (closed.find(child->state) != closed.end()) {
                    skipCount++;
                    delete child;
                    continue;
                } 
                
                open.emplace(child->f, child);
                all_nodes.push_back(child);
                nodesGenerated++;
            }

            closed.insert(current->state);
        }

        // If no solution found
        cout << "No solution found for " << heuristic << endl;
        for (Node* node : all_nodes) delete node;
        
    } catch (const exception& e) {
        cerr << "An error occurred: " << e.what() << endl;
    }

    result.path = {};
    result.metrics = metrics;
    return result;
}

int main()
{

    // Enable virtual terminal processing on Windows 10 to handle ANSI escape sequences
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD consoleMode;
    GetConsoleMode(hConsole, &consoleMode);
    consoleMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hConsole, consoleMode);

    // ANSI escape codes
    cout << "\x1b[1;31m\n\nWelcome to A* Search Simulator\n\n\x1b[0m" << endl;
    
    vector<vector<int>> goal = {{1, 2, 3}, {8, 0, 4}, {7, 6, 5}};
    vector<vector<int>> init = {{2, 8, 3}, {1, 6, 4}, {0, 7, 5}};
    vector<vector<int>> init2 = {{2, 1, 6}, {4, 0, 8}, {7, 5, 3}};

    vector<SearchResult> results_init1;
    vector<SearchResult> results_init2;


    // First initial state runs
    cout << "Running h1...\n\n" << endl;
    SearchResult result = AStarSearch(init, goal, "h1");
    results_init1.push_back(result);
    for (auto& state : result.path) printState(state);

    cout << "Running h2...\n\n" << endl;
    result = AStarSearch(init, goal, "h2");
    results_init1.push_back(result);
    for (auto& state : result.path) printState(state);


    // Second initial state runs
    cout << "\n\nSecond Initial State Run\n\n" << endl;
    cout << "Running h1...\n\n" << endl;
    result = AStarSearch(init2, goal, "h1");
    results_init2.push_back(result);
    for (auto& state : result.path) printState(state);

    cout << "Running h2...\n\n" << endl;
    result = AStarSearch(init2, goal, "h2");
    results_init2.push_back(result);
    for (auto& state : result.path) printState(state);


    // Print tables for initial state #1
    cout << "\n\nTable 1\n" << endl;
    cout << "Initial State #1:\n" << endl;
    cout << "Heuristic Function\tET (ms)\tNG\tNE\tD\tb*" << endl;

    for (auto& res : results_init1) {
        cout << res.metrics.heuristic << "\t\t\t"
             << res.metrics.ET << "\t"
             << res.metrics.NG << "\t"
             << res.metrics.NE << "\t"
             << res.metrics.D << "\t"
             << res.metrics.b_star << endl;
    }


    // Print tables for initial state #2
    cout << "\n\nTable 2\n" << endl;
    cout << "Initial State #2:\n" << endl;
    cout << "Heuristic Function\tET (ms)\tNG\tNE\tD\tb*" << endl;

    for (auto& res : results_init2) {
        cout << res.metrics.heuristic << "\t\t\t"
             << res.metrics.ET << "\t"
             << res.metrics.NG << "\t"
             << res.metrics.NE << "\t"
             << res.metrics.D << "\t"
             << res.metrics.b_star << endl;
    }

    cout << "\nPress ENTER to continue...";
    cin.get();

    return 0;
}


void printState(vector<vector<int>> state) {

    for (int i = 0; i < state.size(); i++) {
        for (int j = 0; j < state[i].size(); j++) {
            cout << state[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}
