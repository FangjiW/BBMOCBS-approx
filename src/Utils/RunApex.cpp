#include "Utils/RunApex.h"
#include "BOAStar.h"

#include<stack>


const std::string resource_path = "resources/";
const std::string output_path = "output/";
std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, 
    size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, 
    double Leps_merge, double Leps_prune, unsigned int time_limit, PathSet& solution_ids, 
    CostSet& solution_apex_costs, CostSet& solution_real_costs, IndividualConstraintSet& indiv_constraint_set) {
    // Compute heuristic
    // std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);  // can run outside and only once
    // sp_heuristic.set_all_to_zero();
    // std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solution;
    int num_exp, num_gen;
    auto runtime = std::clock();

    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        // Pair<double> eps_pair({eps, eps});
        // solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({0, 0});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "NAMOAdr"){
        EPS eps_vec (graph.get_num_of_objectives(), 0);
        solver = std::make_unique<NAMOAdr>(graph, eps_vec, logger);
    }else if (algorithm == "Apex"){
        EPS eps_prune (graph.get_num_of_objectives(), Leps_prune);
        EPS eps_merge (graph.get_num_of_objectives(), Leps_merge);
        solver = std::make_unique<ApexSearch>(graph, eps_merge, eps_prune, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }

    // auto start =std::clock();
    (*solver)(solution_ids, solution_apex_costs, solution_real_costs, source, target, heuristic, 
            indiv_constraint_set, time_limit);
    // runtime = std::clock() - start;

    // std::cout << "LowLevel" << "  Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;
    // num_exp = solver->get_num_expansion();
    // num_gen = solver->get_num_generation();
//     // for (auto sol: solution){
//     //     std::cout << *sol << std::endl;
//     // }
//     // for (auto sol: solution){
//     //     std::cout << sol->path_node->g[0] << ", " << sol->path_node->g[1] << std::endl;
//     // }

// // getchar();
//     // for(auto ele : solution_real_costs){
//     //     std::cout << ele.second.at(0) << ", " << ele.second.at(1) << std::endl;
//     // }
//     // getchar();
    // output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
    //        << source << "\t" << target << "\t"
    //        << num_gen << "\t"
    //        << num_exp << "\t"
    //        << solution.size() << "\t"
    //        << (double) runtime / CLOCKS_PER_SEC
    //        << std::endl;

    // std::cout << "-----End Single Example-----" << std::endl;
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, 
    std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, 
    double Leps_merge, double Leps_prune, int time_limit, PathSet& solution_ids, CostSet& solution_apex_costs, 
    CostSet& solution_real_costs, IndividualConstraintSet& indiv_constraint_set) {
    
    AdjacencyMatrix graph(graph_size, edges);   // can run outside and only once
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, Leps_merge, Leps_prune, time_limit, solution_ids, solution_apex_costs, solution_real_costs, indiv_constraint_set);
 }

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit, IndividualConstraintSet& indiv_constraint_set) {
    // std::ofstream stats;
    // stats.open(output_path + output_file, std::fstream::app);


    // std::vector<std::pair<size_t, size_t>> queries;
    // if (load_queries(query_file, queries) == false) {
    //     std::cout << "Failed to load queries file" << std::endl;
    //     return;
    // }

    // // Build graphs
    // AdjacencyMatrix graph(graph_size, edges);
    // AdjacencyMatrix inv_graph(graph_size, edges, true);

    // size_t query_count = 0;
    // for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

    //     query_count++;
    //     std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
    //     size_t source = iter->first;
    //     size_t target = iter->second;

    //     single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, indiv_constraint_set);
    // }

}
