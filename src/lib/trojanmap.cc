#include "trojanmap.h"
#include <cstdlib>

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------

/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id) { 

  if( data.count(id) == 0 ) return -1;
  else return data[id].lat;

}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id) {

  if( data.count(id) == 0 ) return -1;
  else return data[id].lon;

}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id) {

  if( data.count(id) == 0 ) return "";
  else return data[id].name;

}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {

  
  if( data.count(id) == 0 ) return {};
  else{

    std::vector<std::string> results;

    for( const auto& neighbor:data[id].neighbors ){

      results.push_back( neighbor );
    }

    return results;
  }
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name) {
  std::string res = "";

  for( auto pr:data){

    if( pr.second.name == name ) res = pr.first;
    else continue;

  }
  
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);

  if( name.size() == 0 ) return results;

  for( auto pr:data){

    std::string locationName = this->GetName( pr.first );

    if( locationName.size() != name.size() ) continue;
    else if( locationName == name ){

      results.first =  this->GetLat( pr.first );
      results.second = this->GetLon( pr.first );
    }
    
  }
  
  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {   //editdistance
  
  std::vector<std::vector<int>> record ( a.size(), std::vector<int>(b.size(),0) );
    
  for( int n=0 ; n<b.size() ; n++ ){
      
      for( int m=0 ; m<a.size() ; m++ ){
          
          int diff = 0;
          
          if( a[m]!=b[n] ) diff = 1;
          
          if( n==0 && m==0 ) record[m][n] = diff;
          else if( n==0 ) record[m][n] = record[m-1][n]+diff;
          else if( m==0 ) record[m][n] = record[m][n-1]+diff;
          else{
              
              int add = record[m][n-1];
              int re = record[m-1][n-1];
              int de = record[m-1][n];
              
              record[m][n] = std::min( add, std::min(re,de) )+diff;
              
          }
          
      }
  }
  
  return record[a.size()-1][ b.size()-1];
}

/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = ""; // Start with a dummy word

  std::string lcs;
  int min_dis = INT_MAX;

  for( const auto& pr:data ){

    if( pr.second.name == "") continue;

    int cur_dis = this->CalculateEditDistance( pr.second.name, name );

    if( cur_dis<min_dis ){

      min_dis = cur_dis;
      lcs = pr.second.name;

    }

  }
  
  return lcs;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {  //O(logn)
  std::vector<std::string> results;

  if( name.size() == 0 ) return results;

  auto node = trie;

  std::string prefix = "";

  for( int i=0 ; i<name.size() ; i++ ){

    char ch = name[i]>='A'&&name[i]<='Z'? name[i]+32:name[i];

    if( node->mp.count(ch) == 0 ) return results;
    else node = node->mp[ch];

    prefix+=node->data;

  }

  this->dfsHelper( node, prefix, results );

  return results;
}

/**
 * dfsHelper: a helper to do dfs for autocomplete
*/


void TrojanMap::dfsHelper( trieNode* node, std::string str, std::vector<std::string>& results ){

  if( node == nullptr ) return;
  else{

    if( node->isEnd == true ) results.push_back( str );
    for( auto pr:node->mp ){

      this->dfsHelper( pr.second, str+pr.second->data, results );
    }

  }

}



/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories() {

  std::unordered_set<std::string> categories;
  std::vector<std::string> results;

  for( const auto& mp:this->data ){

    if( mp.second.attributes.size() > 0 ){

      for( const auto& attr:mp.second.attributes ) categories.insert(attr);
    }
  }

  for( const auto& attr:categories ) results.push_back(attr);

  return results;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
  std::vector<std::string> res;

  for( const auto& mp:this->data ){

    for( const auto& attr:mp.second.attributes ){
      
      if( attr == category ) res.push_back( mp.first );
      break;
    }

  }

  if( res.size() <= 0 ) res.push_back("(-1,-1)");
  return res;
}

/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location) {

  std::vector<std::string> res;

  for( const auto& mp:this->data ){

    if( std::regex_match (mp.second.name, location) ) res.push_back( mp.first );
    else continue;
  }

  return res;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));

  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra( 
    std::string location1_name, std::string location2_name) {

  std::vector<std::string> path;
  std::string src = this->GetID(location1_name);
  std::string des = this->GetID(location2_name);
  std::unordered_map< std::string, std::string > prev;
  std::unordered_map< std::string, double> weight;
  std::unordered_map< std::string, bool> visited;

  std::priority_queue< std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater< std::pair<double, std::string>>> pq;

  std::unordered_map< std::string, std::unordered_map< std::string, double > > mp;


  pq.push( {0.0, src} );

  while( !pq.empty() ){

    std::string cur = pq.top().second;
    pq.pop();

    if( visited[cur] ) continue;
    else visited[cur] = true;

    if( cur == des )break;

    for( const auto& id:this->GetNeighborIDs(cur) ){

      if( !visited[id] ){   

        double alt = this->CalculateDistance( cur, id )+weight[cur];

        if( weight[id]>= alt || weight[id]==0 ){

          weight[id] = alt;
          prev[id] = cur;

        }

        pq.push( {weight[id], id} );

      }
    }
    
  }

  if( prev.count(des) == 0 ) return {};

  std::string next = prev[des];
  path.push_back(des);

  while( next!=src ){

    path.push_back(next);
    next = prev[next];
  }

  path.push_back(src);
  
  std::reverse( path.begin(), path.end() );
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(std::string location1_name, std::string location2_name) { //bellman

  std::vector<std::string> path;

  std::unordered_map< std::string, std::string > prev;
  std::unordered_map< std::string, double > d;


  std::string src = this->GetID(location1_name);
  std::string des = this->GetID(location2_name);

  for( const auto& pr:this->data ){

      d[pr.first] = 100;
  }

  d[src] = 0;

  std::set<std::string> out_update;

  for( int i=0 ; i<data.size()-1 ; i++ ){

    std::set<std::string> in_update = out_update;
    out_update = {};

    if( i==0 ){

      for( const auto& nei:GetNeighborIDs(src) ){

          d[nei] = CalculateDistance( nei, src );
          out_update.insert(nei);
          prev[nei] = src;
      }

    }

    for( const auto& v:in_update){

      for( const auto& nei:this->GetNeighborIDs(v) ){

        double alt = d[v]+this->CalculateDistance( nei, v );

        if( d[nei]> alt ){

            d[nei] = alt;
            prev[nei] = v;
            out_update.insert(nei);

        } 

      }

    }
    if( out_update.size() == 0 ) break;
  }

  if( prev.count(des) == 0 ) return {};

  std::string next = des;

  while( next!=src ){

      path.push_back(next);
      //std::cout<<next<<std::endl;
      next = prev[next];
  }

  path.push_back( src );

  std::reverse( path.begin(), path.end() );

  return path;
}

/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path, 
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance, 
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.

void greedyHelper( std::vector<int> cur, std::vector<std::vector<int>>& results, std::vector<bool> visited ){
    
    if( cur.size() == visited.size() ) results.push_back( cur );
    else{
        
        for( int i=0 ; i<visited.size() ; i++ ){
            
            if( visited[i] ) continue;
            else{
                
                cur.push_back(i);
                visited[i] = true;
                greedyHelper( cur, results, visited );
                cur.pop_back();
                visited[i] = false;
            }
        }
    }
}


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  std::vector<bool> visited( location_ids.size(), 0 );
  std::vector<std::vector<int>> results;
    
  greedyHelper( {}, results, visited );
  
  double min_dis = INT_MAX;

  for( const auto& vt:results ){

    double tmp_dis;
    std::vector<std::string> tmp_vt;
    for( int i=0 ; i<vt.size() ; i++ ) tmp_vt.push_back( location_ids[vt[i]] );
    tmp_vt.push_back( location_ids[vt[0]] );
    
    tmp_dis = this->CalculatePathLength( tmp_vt );

    if( tmp_dis<min_dis ){

      min_dis = tmp_dis;
      records.first = min_dis;
      records.second.push_back( tmp_vt );

    }
  }  

  return records;
}

void TrojanMap::backtrackingHelper( std::vector<int> cur, double& min_dis, double cur_dis, std::pair<double, std::vector<std::vector<int>>>& results, std::vector<bool> visited, std::vector<std::string> list ){
    
    if( cur.size() == visited.size() ) {
      
      double tmp_dis = cur_dis+this->CalculateDistance( list[cur.back()], list[cur[0]]);

      if( tmp_dis>=min_dis ) return;

      min_dis = tmp_dis;
      results.first = min_dis;

      cur.push_back(cur[0]);
      results.second.push_back(cur);


    }else{

      for( int i=0 ; i<visited.size() ; i++ ){
            
        if( visited[i] ) continue;
        else{
            
            double dis;
            if( cur.size() == 0 ) dis = 0;
            else dis = this->CalculateDistance( list[cur.back()], list[i]);

            if( cur_dis+dis > min_dis) continue;

            cur.push_back(i);

            visited[i] = true;
            this->backtrackingHelper( cur, min_dis, cur_dis+dis, results, visited, list );
            cur.pop_back();

            visited[i] = false;
        }
      }

    }
    
}



// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  std::vector<std::vector<int>> results;
  std::vector<bool> visited( location_ids.size(), 0 );
  std::pair<double, std::vector<std::vector<int>>> tmp_records;
  double min_dis = INT_MAX;
    
  this->backtrackingHelper( {}, min_dis, 0, tmp_records, visited, location_ids );
  

  records.first = tmp_records.first;

  for( const auto& vt:tmp_records.second ){

    std::vector<std::string> tmp;
    for( const auto& num:vt ) tmp.push_back( location_ids[num] );

    records.second.push_back(tmp);

  }


  return records;
}

std::vector<std::string> indexToString( std::vector<int>& input, std::vector<std::string> location_ids ){

  std::vector<std::string> output;
  for( const auto& num:input )output.push_back( location_ids[num] );
  return output;

}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;

  if( location_ids.size() == 1 ) return { 0, {{location_ids[0], location_ids[0]}} };

  std::srand( time(NULL) );

  int max_counter = 1000;

  std::vector<int> path;
  double min_dis = INT_MAX;

  for( int i=0 ; i<location_ids.size() ; i++) path.push_back(i);
  path.push_back(0);
  min_dis = this->CalculatePathLength( indexToString( path, location_ids ) );

  bool flag = 0;

  for( int i=0 ; i<max_counter ; i++ ){

    if( i == max_counter-1 && !flag )i=0;

    int tmp1 = rand()%location_ids.size();
    int tmp2 = rand()%location_ids.size();

    tmp1++;
    tmp2++;

    int start, end;

    if( tmp1 == tmp2 )continue;
    else{

      flag = 1;
      start = std::min( tmp1, tmp2 );
      end = std::max( tmp1, tmp2 );

      std::vector<int> tmp_path( path );

      std::reverse( tmp_path.begin()+start, tmp_path.begin()+end );

      std::vector<std::string> str_path = indexToString( tmp_path, location_ids );
      double tmp_dis = this->CalculatePathLength( str_path );

      if( tmp_dis<min_dis ){

        min_dis = tmp_dis;
        records.first = min_dis;
        path = tmp_path;
        records.second.push_back( str_path );
      }


    }

  }

  if( records.second.size() == 0 ){

    records.first = min_dis;
    location_ids.push_back( location_ids[0] );
    records.second.push_back( location_ids );
  }

  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open("/Users/tim/Documents/courses/EE538/spring2023_trojanmap-timjtchang/input/"+locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, word)) {
    location_names_from_csv.push_back(word);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open("/Users/tim/Documents/courses/EE538/spring2023_trojanmap-timjtchang/input/"+dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> dependency;
    while (getline(s, word, ',')) {
      dependency.push_back(word);
    }
    dependencies_from_csv.push_back(dependency);
  }
  fin.close();
  return dependencies_from_csv;
}

void topology_helper( std::unordered_map< std::string, std::vector< std::string> >& edges, std::unordered_map< std::string, bool >& visited, 
                      std::string cur, std::vector<std::string>& result ){
    
    visited[cur] = true;
    for( const auto& child:edges[cur] ){

      if( visited[child] ) continue;

      topology_helper( edges, visited, child, result );

    }

    result.push_back( cur );
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan( 
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies) {
  

  std::unordered_map< std::string, std::vector< std::string> > edges;
  std::unordered_map< std::string, bool > visited;

  for( const auto& vt:dependencies )edges[vt[0]].push_back(vt[1]);
  std::vector<std::string> result;

  for( const auto& root:locations){

    if( visited[root] ) continue;
    topology_helper( edges, visited, root, result );

  }

  std::reverse( result.begin(), result.end() );

  return result;     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) { 
  // l, r, u, d

  double lon = this->data[id].lon;
  double lat = this->data[id].lat;

  if( lon>=square[0] && lon<=square[1] && lat<=square[2] && lat>=square[3] )return true;

  else return false;
  
}


/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;

  for( const auto& v:data ){

    if( this->inSquare(v.first, square) ) subgraph.push_back(v.first);

  }

  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */

bool TrojanMap::cycleHelper( std::unordered_map<std::string, bool>& visited, 
                              std::unordered_map<std::string, bool>& viewed,
                                std::vector<double> & square, std::string cur, std::string prev ){ //hello

  if( viewed[cur] ) return true;
  else if( this->GetNeighborIDs(cur).size() == 0 || visited[cur] ) return false;
  else{

    visited[cur] = true;
    viewed[cur] = true;

    for( const auto& nei:this->GetNeighborIDs(cur) ){

      if( nei == cur || !this->inSquare(nei, square) || nei == prev ) continue;
      else if( cycleHelper( visited, viewed, square, nei, cur ) )return true; 

    }
    viewed[cur] = false;

  }

  return false;

}
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {

  std::unordered_map< std::string, bool > visited;
  std::unordered_map< std::string, bool > viewed;

  int count = 0;
  for( const auto& v:subgraph ){

    if( visited[v]) continue;
    else{
      
      bool result = this->cycleHelper( visited, viewed, square, v, "" );

      if( result ) return true;
    }

    count++;

  }

  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;

  std::queue<std::string> q;
  std::unordered_map< std::string, bool > visited;
  std::unordered_map< std::string, double > record;

  int counter = 0;

  q.push( this->GetID(name) );
  visited[ this->GetID(name) ] = true;

  std::string start = this->GetID(name) ;

  while( !q.empty() ){

    if( counter == k ) break;

    std::string cur = q.front();
    q.pop();

    for( const auto& str:this->GetNeighborIDs(cur) ){

      if( visited[str] || this->CalculateDistance(str,start)>r) continue;
      
      q.push(str);
      visited[str] = true;
      
      if( this->data[str].attributes.find( attributesName ) != this->data[str].attributes.end() ){

        record[str] = this->CalculateDistance(str,start);
        counter++;
      }

    }

  }

  while( record.size()!=0 ){

    std::string hold;
    double dis = INT_MAX;

    for( const auto& pr:record ){

      if( pr.second < dis ){

        hold = pr.first;
        dis = pr.second;

      } 

    }
    
    res.push_back( hold );
    record.erase( hold );
  }


  return res;
}


/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */

void pathHelper( std::vector<std::vector<int>>& result, std::vector<int> cur, std::vector<bool> visited ){

  if( cur.size() == visited.size() ){

    result.push_back(cur);
    return;

  }else{

    for( int i=0 ; i<visited.size() ; i++ ){

      if( visited[i] ) continue;
      else{

        cur.push_back(i);
        visited[i] = true;
        pathHelper( result, cur, visited );
        cur.pop_back();
        visited[i] = false;
      }

    }

  }
}
std::vector<std::string> TrojanMap::TrojanPath(
      std::vector<std::string> &location_names) {

    
    std::vector<std::vector<int>> path;
    std::vector<bool> visited( location_names.size(), false );
    pathHelper( path, {}, visited );

    double min_dis = INT_MAX;
    std::vector<std::string> res;

    std::vector<std::vector<int>> record( location_names.size(), std::vector<int>( location_names.size(), 0));
    std::unordered_map<int, std::pair<double, std::vector<std::string>>> record_data;

    for( auto& vt:path ){

      std::vector<std::string> str_vt = indexToString( vt, location_names );

      double tmp_dis = 0;
      std::vector<std::string> tmp_res;
      tmp_res.push_back( "hello");

      for( int i=0 ; i<vt.size()-1 ; i++ ){

        tmp_res.pop_back();

        if(  !record[vt[i]][vt[i+1]] ){
          
          record[vt[i]][vt[i+1]] = record_data.size()+1;
          
          std::vector<std::string> dij_path = this->CalculateShortestPath_Dijkstra( str_vt[i], str_vt[i+1] );
          double dij_dis =  this->CalculatePathLength(dij_path);

          record_data[ record[vt[i]][vt[i+1]] ] = { dij_dis, dij_path };
        }

        std::vector<std::string> tmp =  record_data[ record[vt[i]][vt[i+1]] ].second;
        tmp_dis += record_data[ record[vt[i]][vt[i+1]] ].first;

        for( const auto& str:tmp ) tmp_res.push_back(str);

      }

      if( tmp_dis<min_dis ){

        min_dis = tmp_dis;
        res = tmp_res;
      }

    }

    return res;
}


std::vector<std::string> TrojanMap::queryHelper( std::string& name1, std::string& name2, double& gas ){

  std::vector<std::string> path;
  std::string src = this->GetID(name1);
  std::string des = this->GetID(name2);
  std::unordered_map< std::string, std::string > prev;
  std::unordered_map< std::string, double> weight;
  std::unordered_map< std::string, bool> visited;

  std::priority_queue< std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater< std::pair<double, std::string>>> pq;

  std::unordered_map< std::string, std::unordered_map< std::string, double > > mp;


  pq.push( {0.0, src} );

  while( !pq.empty() ){

    std::string cur = pq.top().second;
    pq.pop();

    if( visited[cur] ) continue;
    else visited[cur] = true;

    if( cur == des )break;

    for( const auto& id:this->GetNeighborIDs(cur) ){

      if( this->CalculateDistance( cur, id )>gas ) continue;

      if( !visited[id] ){   

        double alt = this->CalculateDistance( cur, id )+weight[cur];

        if( weight[id]>= alt || weight[id]==0 ){

          weight[id] = alt;
          prev[id] = cur;

        }

        pq.push( {weight[id], id} );

      }
    }
    
  }


  if( prev.count(des) == 0 ) return {};
  else{

    std::string next = prev[des];
    path.push_back(des);

    while( next!=src ){

      path.push_back(next);
      next = prev[next];
    }

    path.push_back(src);
  
    std::reverse( path.begin(), path.end() );

    return path;

  }

  
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans(q.size());

    this->queries_path = {};

    for( int i=0 ; i<q.size() ; i++ ){
      
      std::pair<double, std::vector<std::string>> pr = q[i];

      std::vector<std::string> path = this->queryHelper( pr.second[0], pr.second[1], pr.first );

      this->queries_path.push_back(path);

      bool flag = true;
      
      if( path.size() == 0 ) flag =false;

      ans[i] = flag;

    }
    

    return ans;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();

}

void TrojanMap::err( std::string msg ){

  std::cout<<"-----ERR-------"<<std::endl;
  std::cout<<msg<<std::endl;
  std::cout<<"---------------"<<std::endl;

  throw;

}

void TrojanMap::buildTrie(){

  this->trie = new trieNode();

  for( auto pr:data ){

    std::string name = GetName( pr.first );
    
    auto node = trie;

    for( auto ch:name ){

      char lowerCase = ch>='A'&&ch<='Z'? ch+32:ch;

      if( node->mp.count(lowerCase) == 0 ){

        node->mp[lowerCase] = new trieNode();
        node->mp[lowerCase]->data = ch;
      } 
      node = node->mp[lowerCase];

    }

    node->isEnd = true;

  }

}

