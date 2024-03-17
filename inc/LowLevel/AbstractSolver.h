#pragma once

#include "Definitions.h"
#include "LowLevel/Utils/Logger.h"

class AbstractSolver {
protected:
    const AdjacencyMatrix   &adj_matrix;
    // Pair<double>            eps;
    EPS eps_prune;
    EPS eps_merge;
    int turn_mode;
    int turn_cost;

    size_t num_expansion = 0;
    size_t num_generation= 0;

    virtual void init_search(){
        num_expansion = 0;
        num_generation = 0;
    }

    const LoggerPtr         logger;
    void start_logging(size_t source, size_t target);
    
    void end_logging(SolutionSet &solutions, bool succ);


public:
    virtual std::string get_solver_name() = 0;

    size_t get_num_expansion(){return num_expansion;}
    size_t get_num_generation(){return num_generation;}

    virtual void operator()(PathSet& solution_ids, CostSet& solution_apex_costs, CostSet& solution_real_costs, 
        size_t source, size_t target, Heuristic &heuristic, VertexConstraint& vertex_constraints, 
        EdgeConstraint& edge_constrains, unsigned int time_limit, VertexCAT& vertex_cat, EdgeCAT& edge_cat, std::unordered_map<int, int>& conflict_num_map) = 0;
        
    AbstractSolver(const AdjacencyMatrix &adj_matrix, EPS eps_prune, int turn_mode, int turn_cost, const LoggerPtr logger): adj_matrix(adj_matrix), eps_prune(eps_prune), logger(logger), turn_mode(turn_mode), turn_cost(turn_cost) {}
    AbstractSolver(const AdjacencyMatrix &adj_matrix, EPS eps_merge, EPS eps_prune, int turn_mode, int turn_cost, const LoggerPtr logger): adj_matrix(adj_matrix), eps_merge(eps_merge), eps_prune(eps_prune), logger(logger), turn_mode(turn_mode), turn_cost(turn_cost) {}
    virtual ~AbstractSolver(){}
};

inline bool is_constraint(NodePtr node, VertexConstraint& vertex_constraints, EdgeConstraint& edge_constraints)
{
    if(vertex_constraints.count(node->t)){
        for(int i = 0; i < vertex_constraints[node->t].size(); i ++){
            if(node->id == vertex_constraints[node->t].at(i)){
                return true;
            }
        }
    }
    if(edge_constraints.count(node->t-1) && edge_constraints[node->t-1].count(node->parent->id)){
        for(int i = 0; i < edge_constraints[node->t-1][node->parent->id].size(); i++){
            if(node->id == edge_constraints[node->t-1][node->parent->id].at(i)){
                return true;
            }
        }
    }
    return false;
}