#include<iostream>
#include<vector>
#include<string>
#include<unordered_map>
#include<queue>

using namespace std;

void addEdge(vector<int> adj[], int u, int v) {

    adj[u].push_back(v);
}

void printGraph(vector<int> adj[], int V) {

    for (int i = 0; i < V; ++i) {
        cout << "Adjacency list of vertex " << i << ": ";
        for (int j = 0; j < adj[i].size(); ++j) {
            cout << adj[i][j] << " ";
        }
        cout << endl;
    }
}

int main(){

    int V = 5;
    vector<int> adj[V];
    addEdge(adj, 0, 1);
    addEdge(adj, 0, 4);
    addEdge(adj, 1, 2);
    addEdge(adj, 1, 3);
    addEdge(adj, 1, 4);
    addEdge(adj, 2, 3);
    addEdge(adj, 3, 4);
    printGraph(adj, V);


    //bfs

    queue<int> q;

    vector<int> result;
    vector<bool> visited ( 5, 0 );
    
    q.push(0);

    while( !q.empty() ){

        int cur = q.front();
        q.pop();

        for( const auto& node:adj[cur] ) if( !visited[node]) q.push(node);
        result.push_back( cur );
        visited[cur] = true;

    }

    for( const auto& num:result ) cout<< num <<" ";
    cout<<endl;

    return 0;

}