#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "BOAStar.h"
// #include "PPA.h"
// #include "SingleCriteria.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, 
    size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, 
    double Leps_merge, double Leps_prune, unsigned int time_limit, PathSet& solution_ids, 
    CostSet& solution_apex_costs, CostSet& solution_real_costs, IndividualConstraintSet& indiv_constraint_set);

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, 
    std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, 
    double Leps_merge, double Leps_prune, int time_limit, PathSet& solution_ids, CostSet& solution_apex_costs, 
    CostSet& solution_real_costs, IndividualConstraintSet& indiv_constraint_set);
                
void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, 
            std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit, 
            IndividualConstraintSet& indiv_constraint_set);


