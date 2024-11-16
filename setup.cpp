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
int h1(vector<vector<int>> state, vector<vector<int>> goal);
int h2(vector<vector<int>> state, vector<vector<int>> goal);

// Sam's heuristic - Euclidean distance
int h3(vector<vector<int>> state, vector<vector<int>> goal);

// Landry's heuristic
int h4(vector<vector<int>> state, vector<vector<int>> goal);


int main()
{

    vector<vector<int>> goal = {{1, 2, 3}, {8, 0, 4}, {7, 6, 5}};

    vector<vector<int>> init = {{2, 8, 3}, {1, 6, 4}, {0, 7, 5}};
    vector<vector<int>> init2 = {{2, 1, 6}, {4, 0, 8}, {7, 5, 3}};



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
