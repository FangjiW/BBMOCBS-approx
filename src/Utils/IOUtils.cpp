#include <fstream>
#include <random>
#include <chrono>

#include "Utils/IOUtils.h"


void split_string(std::string string, std::string delimiter, std::vector<std::string> &results)
{
    size_t first_delimiter;

    while ((first_delimiter = string.find_first_of(delimiter)) != string.npos) {
        if (first_delimiter > 0) {
            results.push_back(string.substr(0, first_delimiter));
        }
        string = string.substr(first_delimiter + 1);
    }

    if (string.length() > 0) {
        results.push_back(string);
    }
}

bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size){
  size_t          max_node_num = 0;
  for (auto gr_file: gr_files){
    std::ifstream file(gr_file.c_str());
    
    if (file.is_open() == false){
      std::cerr << "cannot open the gr file " << gr_file << std::endl;
      return false;
    }

    std::string line;
    int idx_edge = 0;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        std::string type = decomposed_line[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
          if (idx_edge < (int)edges_out.size() - 1){
            if (
                (stoul(decomposed_line[1]) != edges_out[idx_edge].source) ||
                (stoul(decomposed_line[2]) != edges_out[idx_edge].target)) {
              // arc_sign src dest should be same in both files
              std::cerr << "file inconsistency" << std::endl;
              return false;
            }
            edges_out[idx_edge].cost.push_back(std::stoul(decomposed_line[3]));
          }else{
            Edge e(std::stoul(decomposed_line[1]),
                   std::stoul(decomposed_line[2]),
                   {std::stoul(decomposed_line[3])});
            edges_out.push_back(e);
            max_node_num = std::max({max_node_num, e.source, e.target});
          }
        }
        idx_edge ++;
    }
    file.close();
  }
  graph_size = max_node_num;
  return true;
}

bool load_gr_files(std::string gr_file1, std::string gr_file2, std::vector<Edge> &edges_out, size_t &graph_size) {
    size_t          max_node_num = 0;
    std::ifstream   file1(gr_file1.c_str());
    std::ifstream   file2(gr_file2.c_str());

    if ((file1.is_open() == false) || (file2.is_open() == false)) {
        return false;
    }

    std::string line1, line2;
    while ((file1.eof() == false) && (file2.eof() == false)) {
        std::getline(file1, line1);
        std::getline(file2, line2);

        if ((line1 == "") || (line2 == "")) {
            break;
        }

        std::vector<std::string> decomposed_line1, decomposed_line2;
        split_string(line1, " ", decomposed_line1);
        split_string(line2, " ", decomposed_line2);

        std::string type = decomposed_line1[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if ((decomposed_line1[0] != decomposed_line2[0]) ||
            (decomposed_line1[1] != decomposed_line2[1]) ||
            (decomposed_line1[2] != decomposed_line2[2])) {
            // arc_sign src dest should be same in both files
            return false;
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
            Edge e(std::stoul(decomposed_line1[1]),
                   std::stoul(decomposed_line1[2]),
                   {std::stoul(decomposed_line1[3]), std::stoul(decomposed_line2[3])});
            edges_out.push_back(e);
            max_node_num = std::max({max_node_num, e.source, e.target});
        }
    }
    graph_size = max_node_num;
    return true;
}

bool load_txt_file(std::string txt_file, std::vector<Edge> &edges_out, size_t &graph_size) {
    bool            first_line = true;
    size_t          max_node_num = 0;
    std::ifstream   file(txt_file.c_str());

    if (file.is_open() == false) {
        return false;
    }

    std::string line;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        if (first_line) {
            first_line = false;
            continue;
        }
        Edge e(std::stoul(decomposed_line[0]),
               std::stoul(decomposed_line[1]),
               {std::stoul(decomposed_line[2]), std::stoul(decomposed_line[3])});
        edges_out.push_back(e);
        max_node_num = std::max({max_node_num, e.source, e.target});
    }
    graph_size = max_node_num;
    return true;
}


bool load_queries(std::string query_file, std::vector<std::pair<size_t, size_t>> &queries_out) {
    std::ifstream   file(query_file.c_str());

    if (file.is_open() == false) {
        return false;
    }

    std::string line;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        } else if (line[0] == '#') {
            continue; // Commented out queries
        }

        std::vector<std::string> decomposed_line;
        split_string(line, ",", decomposed_line);

        std::pair<size_t, size_t> query = {std::stoul(decomposed_line[0]), std::stoul(decomposed_line[1])};
        queries_out.push_back(query);
    }
    return true;
}


void PreProcessor::read_map(std::string map_file_name, Map& map, std::unordered_map<size_t, std::vector<int>>& id2coord)
{
    std::ifstream Input(map_file_name);
    if (!Input.is_open()) {
        std::cout << "Error: Unable to open map file." << std::endl;
        exit(1);
    }



    // Read map information
    std::string temp;
    int height, width;
    std::getline(Input, temp);
    Input >> temp >> height >> temp >> width;
    std::getline(Input, temp);
    std::getline(Input, temp);

    if (height <= 0 || width <= 0) {
        std::cout << "Error: Invalid map format." << std::endl;
        exit(1);
    }

    // Create a Map object to store the map data
    map = Map(width, height);

    // Read the map data from the file and store it in the Map object
    for (int x = 0; x < height; ++x) {
        std::string line;
        std::getline(Input, line);

        for (int y = 0; y < width; ++y) {
            if (line[y] == '@') {
                map.setVal(x, y, -1);
                map.setID(x, y, -1);
            } else if (line[y] == '.') {
                map.setVal(x, y, 0);
                map.setID(x, y, map.graph_size);
                id2coord.insert(std::make_pair(map.graph_size, std::vector<int>({x, y})));
                map.graph_size ++; // Increment graph_size for each '.'
            } else {
                std::cout << x << y << " " << line[y];
                getchar();
                std::cout << "Error: Invalid character in map file." << std::endl;
                exit(1);
            }
        }
    }

    Input.close();
}

void PreProcessor::read_config(std::string config_file, Map map, int agent_num, std::vector<std::pair<size_t, size_t>>&  start_end)
{
    // std::cout << config_file[config_file.length()-1];
    // getchar();
    if(config_file[config_file.length()-1] == 'g'){
        std::cout << "here";
        std::ifstream Input(config_file);
        std::string a; double b; int c;

        if(!Input.is_open()){
            std::cout << "Error: Unable to open configure file." << std::endl;
            exit(1);
        }
        // Input >> a >> c;
        for(int i = 0; i < agent_num; i ++){
            size_t x1, x2, y1, y2;
            // Input >> c >> a >> a >> a >> y1 >> x1 >> y2 >> x2  >> b;
            // std::cout << x1 << y1 << x2 << y2;
            // getchar();
            Input >> x1 >> y1 >> x2 >> y2;
            start_end.push_back(std::make_pair(map.getID(x1, y1), map.getID(x2, y2)));
        }

        Input.close();
        return;
    }
    std::ifstream Input(config_file);
    std::string a; double b; int c;

    if(!Input.is_open()){
        std::cout << "Error: Unable to open configure file." << std::endl;
        exit(1);
    }
    Input >> a >> c;
    for(int i = 0; i < agent_num; i ++){
        size_t x1, x2, y1, y2;
        Input >> c >> a >> a >> a >> y1 >> x1 >> y2 >> x2  >> b;
        // std::cout << x1 << y1 << x2 << y2;
        // getchar();
        // Input >> x1 >> y1 >> x2 >> y2;
        start_end.push_back(std::make_pair(map.getID(x1, y1), map.getID(x2, y2)));
    }

    Input.close();
}

// void PreProcessor::read_cost(Map map, Map cost_map, Edge& edge, std::unordered_map<std::vector<size_t>, size_t> coord2id)
// {
//     for(size_t i = 0; i < map.getWidth(); i ++){

//     }
// }

void PreProcessor::generate_cost(Map map, std::vector<Edge>& edges)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    Map cost_map1(map.width, map.height);
    Map cost_map2(map.width, map.height);

    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(1, 2);

    // Read the map data from the file and store it in the Map object
    for(int x = 0; x < map.height; x ++) {
        for(int y = 0; y < map.width; y ++){
            int randomNumber1 = distribution(generator);
            int randomNumber2 = distribution(generator);
            cost_map1.setVal(x, y, randomNumber1);
            cost_map2.setVal(x, y, randomNumber2);
        }
    }

    for(int x = 0; x < map.height; x ++) {
        for(int y = 0; y < map.width; y ++){
            if(map.getVal(x, y) == -1){
                continue;
            }
            edges.push_back(Edge(map.getID(x, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));        //  add remain place action

            if(y < map.width-1 && map.getVal(x, y+1) == 0){
                edges.push_back(Edge(map.getID(x, y+1), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(y > 0 && map.getVal(x, y-1) == 0){
                edges.push_back(Edge(map.getID(x, y-1), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(x < map.height-1 && map.getVal(x+1, y) == 0){
                edges.push_back(Edge(map.getID(x+1, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(x > 0 && map.getVal(x-1, y) == 0){
                edges.push_back(Edge(map.getID(x-1, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
        }
    }
}

void PreProcessor::read_cost(std::string cost_file, Map map, std::vector<Edge>& edges)
{
    std::ifstream Input(cost_file);
    if(!Input.is_open()){
        std::cout << "Error: Unable to open cost file." << std::endl;
        exit(1);
    }
    Map cost_map1(map.width, map.height);
    Map cost_map2(map.width, map.height);

    // Read the map data from the file and store it in the Map object
    for(int x = 0; x < map.height; x ++) {
        for(int y = 0; y < map.width; y ++){
            size_t cost1;
            Input >> cost1;
            // std::cout << cost1 << std::endl;
            cost_map1.setVal(x, y, cost1);
        }
    }
    for(int x = 0; x < map.height; x ++) {
        for(int y = 0; y < map.width; y ++){
            int cost2;
            Input >> cost2;
            cost_map2.setVal(y, x, cost2);
        }
    }

    for(int x = 0; x < map.height; x ++) {
        for(int y = 0; y < map.width; y ++){
            if(map.getVal(x, y) == -1){
                continue;
            }
            edges.push_back(Edge(map.getID(x, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));        //  add remain place action

            if(y < map.width-1 && map.getVal(x, y+1) == 0){
                edges.push_back(Edge(map.getID(x, y+1), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(y > 0 && map.getVal(x, y-1) == 0){
                edges.push_back(Edge(map.getID(x, y-1), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(x < map.height-1 && map.getVal(x+1, y) == 0){
                edges.push_back(Edge(map.getID(x+1, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
            if(x > 0 && map.getVal(x-1, y) == 0){
                edges.push_back(Edge(map.getID(x-1, y), map.getID(x, y), std::vector<size_t>(
                    {cost_map1.getVal(x, y), cost_map2.getVal(x, y)})));
            }
        }
    }

    Input.close();
}