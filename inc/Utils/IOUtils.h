#ifndef UTILS_IO_UTILS_H
#define UTILS_IO_UTILS_H

#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include "Utils/Definitions.h"
#include "Utils/MAP.h"

bool load_gr_files(std::string gr_file1, std::string gr_file2, std::vector<Edge> &edges, size_t &graph_size);
bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges, size_t &graph_size);
bool load_queries(std::string query_file, std::vector<std::pair<size_t, size_t>> &queries_out);

class PreProcessor
{
public:
    void read_map(std::string map_file_name, Map& map, std::unordered_map<size_t, std::vector<int>>& id2coord);
    void read_config(std::string config_file_name, Map map, int agent_num, std::vector<std::pair<size_t, size_t>>&  start_end);
    void read_cost(std::string cost_file_name, Map map, std::vector<Edge>& edges);
    void generate_cost(Map map, std::vector<Edge>& edges, int dim);
};

#endif //UTILS_IO_UTILS_H
