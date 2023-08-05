// #ifndef BI_CRITERIA_PPA_H
// #define BI_CRITERIA_PPA_H

// #include "Utils/Definitions.h"
// #include "Utils/MapQueue.h"
// #include "AbstractSolver.h"

// class PPA : public AbstractSolver {
// protected:
//     Pair<double> eps_pair;

//     virtual void insert(PathPairPtr &pp, PPQueue &queue);
//     virtual void merge_to_solutions(const PathPairPtr &pp, PPSolutionSet &solutions);

//     size_t num_missed_merge = 0;
//     std::vector<std::vector<PathPairPtr>> expanded;
//     void init_search();

//     std::vector<PathPairPtr> min_g2;
//     PPSolutionSet pp_solutions;

//     size_t target;
//     virtual inline bool is_dominated(PathPairPtr pp);

// public:
//     virtual std::string get_solver_name() {return "PPA*"; }

//     PPA(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger=nullptr);
//     virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
// };

// #endif //BI_CRITERIA_PPA_H
