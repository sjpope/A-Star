// g++ -o setup.exe setup.cpp

#include <iostream>
#include <bits/stdc++.h>

using namespace std;

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

void printState(vector<vector<int>> state);

int h1(vector<vector<int>> state, vector<vector<int>> goal){
    // Number of misplaced tiles
    int count = 0;
    for (int i = 0; i < state.size(); i++) {
        for (int j = 0; j < state[i].size(); j++) {
            if (state[i][j] != goal[i][j]) {
                count++;
            }
        }
    }
}

int h2(vector<vector<int>> state, vector<vector<int>> goal){
    // Sum of distances of tiles from their goal positions
    int sum = 0;
    for (int i = 0; i < state.size(); i++) {
        for (int j = 0; j < state[i].size(); j++) {
            int num = state[i][j];
            if (num != 0) {
                int goal_x, goal_y;
                for (int x = 0; x < goal.size(); x++) {
                    for (int y = 0; y < goal[x].size(); y++) {
                        if (goal[x][y] == num) {
                            goal_x = x;
                            goal_y = y;
                            break;
                        }
                    }
                }
                sum += abs(i - goal_x) + abs(j - goal_y);
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

            Node* child = new Node(new_state, current->g + 1, new_h, current);
            children.push_back(child);
        }
    }
    return children;
}

// A* search algorithm
vector<vector<vector<int>>> AStarSearch(const vector<vector<int>>& initial, const vector<vector<int>>& goal, string heuristic) {

    priority_queue<pair<int, Node*>, vector<pair<int, Node*>>, greater<pair<int, Node*>>> open;
    vector<Node*> all_nodes; // To free memory later

    int h_root = (heuristic == "h1") ? h1(initial, goal) : h2(initial, goal);
    Node* root = new Node(initial, 0, h_root);

    all_nodes.push_back(root);
    open.emplace(root->f, root);

    while (!open.empty()) {
        Node* current = open.top().second;
        open.pop();

        if (current->state == goal) {

            vector<vector<vector<int>>> path;
            while (current != nullptr) {
                path.push_back(current->state);
                current = current->parent;
            }
            reverse(path.begin(), path.end());


            // Clean up memory
            for (Node* node : all_nodes) delete node;
            return path;
        }

        vector<Node*> children = generateChildren(current, goal, heuristic);
        for (Node* child : children) {
            open.emplace(child->f, child);
            all_nodes.push_back(child);
        }
    }

    // If no solution found
    for (Node* node : all_nodes) delete node;
    return {}; // Return empty path
}

int main()
{

    vector<vector<int>> goal = {{1, 2, 3}, {8, 0, 4}, {7, 6, 5}};

    vector<vector<int>> init = {{2, 8, 3}, {1, 6, 4}, {0, 7, 5}};
    vector<vector<int>> init2 = {{2, 1, 6}, {4, 0, 8}, {7, 5, 3}};


    vector<vector<vector<int>>> path = AStarSearch(init, goal, "h1");
    for (auto& state : path) printState(state);
    

    path = AStarSearch(init2, goal, "h2");
    for (auto& state : path) printState(state);


    /*
    For each run print the execution time (ET), the number of nodes generated (NG), 
    the number of nodes expanded (NE), depth of the tree (D), and the effective branching factor b* (NG/D).  
    Tabulate this data in two tables, one for each initial state.  
    A sample of such a table is given below. 
    
    Also, provide the total path for each run: */

    return 0;
}

// Need a closed list to avoid revisiting nodes

void printState(vector<vector<int>> state) {

    for (int i = 0; i < state.size(); i++) {
        for (int j = 0; j < state[i].size(); j++) {
            cout << state[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}
