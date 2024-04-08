#include <math.h>

#include <algorithm>
#include <cfloat>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <regex>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <stack>
#include <cstdlib>

using namespace std;

unordered_map< string, vector<string>> edges;
unordered_map< string, unordered_map<string, int>> weight;

int CalculateDistance( string id1, string id2 ){

    return weight[id1][id2];
}

vector<string> GetNeighborIDs( string id ){

    return edges[id];
}

void bell_man(  std::string location1_name, std::string location2_name ){

    unordered_map< string, vector<string>> data = edges;

    std::vector<std::string> path;
    std::unordered_map< std::string, std::string > prev;
    std::unordered_map< std::string, double > d;

    std::string src = location1_name;
    std::string des = location2_name;

    for( const auto& pr:data ){

      d[pr.first] = INT_MAX;
    }

    d[src] = 0;

    //SPFA

    for( int i=0 ; i<data.size()-1 ; i++ ){

        set<string> update;

        if( i==0 ){

            for( const auto& nei:GetNeighborIDs(src) ){

                d[nei] = CalculateDistance( nei, src );
                update.insert(nei);
                prev[nei] = src;
            }
        }

        for( const auto& v:update ){

            for( const auto& nei:GetNeighborIDs(v) ){

                double alt = d[v]+CalculateDistance( nei, v );

                if( d[nei]>= alt ){

                    d[nei] = alt;
                    prev[nei] = v;
                    update.insert(nei);

                } 
                
            }

        }

        if( update.size() == 0 ) break;

    }


    string next = des;

    for( const auto& str:prev ) cout<<str.first<<":"<<str.second<<endl;

    while( next!=src ){

        path.push_back(next);
        cout<<next<<endl;
        next = prev[next];
    }

    cout<<next<<endl;

}


int main() { //bellman

edges["A"] = {"B", "C", "D"};
edges["B"] = {"A", "C", "E"};
edges["C"] = {"A", "B", "D", "E"};
edges["D"] = {"A", "C", "F", "G"};
edges["E"] = {"B", "C", "F"};
edges["F"] = {"D", "E", "G", "H"};
edges["G"] = {"D", "F", "H", "I"};
edges["H"] = {"F", "G", "I"};
edges["I"] = {"G", "H", "J"};
edges["J"] = {"I"};

weight["A"]["B"] = 4;
weight["A"]["C"] = 3;
weight["A"]["D"] = 1;

weight["B"]["A"] = 4;
weight["B"]["C"] = 2;
weight["B"]["E"] = 2;

weight["C"]["A"] = 3;
weight["C"]["B"] = 2;
weight["C"]["D"] = 4;
weight["C"]["E"] = 3;

weight["D"]["A"] = 1;
weight["D"]["C"] = 4;
weight["D"]["F"] = 5;
weight["D"]["G"] = 1;

weight["E"]["B"] = 2;
weight["E"]["C"] = 3;
weight["E"]["F"] = 2;

weight["F"]["D"] = 5;
weight["F"]["E"] = 2;
weight["F"]["G"] = 3;
weight["F"]["H"] = 2;

weight["G"]["D"] = 1;
weight["G"]["F"] = 3;
weight["G"]["H"] = 2;
weight["G"]["I"] = 4;

weight["H"]["F"] = 2;
weight["H"]["G"] = 2;
weight["H"]["I"] = 3;

weight["I"]["G"] = 4;
weight["I"]["H"] = 3;
weight["I"]["J"] = 5;

weight["J"]["I"] = 5;



    bell_man( "A", "J");
  
}

