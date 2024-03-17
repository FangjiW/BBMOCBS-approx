#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "ShortestPathHeuristic.h"
#include "Definitions.h"
#include "IOUtils.h"
#include "LowLevel/Utils/Logger.h"
#include "LowLevel/BOAStar.h"
// #include "PPA.h"
// #include "SingleCriteria.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, Heuristic& heuristic, size_t source, 
    size_t target, LSolver l_solver, double eps, MergingStrategy ms, LoggerPtr logger, unsigned int time_limit, PathSet& solution_ids, 
    CostSet& solution_apex_costs, CostSet& solution_real_costs, VertexConstraint& vertex_constraints, 
    EdgeConstraint& edge_constraints, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map, int turn_dim, int turn_cost);

// void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, 
//     std::string algorithm, MergingStrategy ms, LoggerPtr logger, 
//     double Leps_merge, double Leps_prune, int time_limit, PathSet& solution_ids, CostSet& solution_apex_costs, 
//     CostSet& solution_real_costs, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints, CAT& cat, 
//     std::unordered_map<int, int>& conflict_num_map, int turn_mode, int turn_cost);
                
void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, 
            std::string algorithm, MergingStrategy ms, LoggerPtr logger, double eps, int time_limit, 
            VertexConstraint& vertex_constraints);


