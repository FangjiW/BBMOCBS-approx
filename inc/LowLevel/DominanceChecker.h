#pragma once

#include "Definitions.h"

class DominanceChecker {
protected:
    EPS eps;
public:
    virtual ~DominanceChecker(){};
    DominanceChecker(EPS eps):eps(eps){};

    virtual bool is_dominated(ApexPathPairPtr node) = 0;

    virtual void add_node(ApexPathPairPtr ap) = 0;
};


class SolutionCheck: public DominanceChecker {
    ApexPathPairPtr last_solution = nullptr;
    std::list<ApexPathPairPtr> solutions;
public:

    SolutionCheck(EPS eps):DominanceChecker(eps){};

    virtual bool is_dominated(ApexPathPairPtr node);

    virtual void add_node(ApexPathPairPtr ap){ last_solution = ap;};
};

class SolutionCheckLinear: public DominanceChecker {
    std::list<ApexPathPairPtr> solutions;

public:

    SolutionCheckLinear(EPS eps):DominanceChecker(eps){};

    virtual bool is_dominated(ApexPathPairPtr node);

    virtual void add_node(ApexPathPairPtr ap);
};


class LocalCheck: public DominanceChecker {

protected:
    std::vector<std::unordered_map<size_t, size_t>>  min_g2;

public:

    LocalCheck(EPS eps, size_t graph_size):DominanceChecker(eps), min_g2(graph_size + 1) {};

    virtual bool is_dominated(ApexPathPairPtr node);

    virtual void add_node(ApexPathPairPtr ap);
};

class LocalCheckLinear: public DominanceChecker {

protected:
    std::vector<std::unordered_map<size_t, std::list<ApexPathPairPtr>>> min_g2;
    bool if_turn;

public:

    LocalCheckLinear(EPS eps, size_t graph_size, bool if_turn):DominanceChecker(eps), min_g2(graph_size + 1), if_turn(if_turn) {};

    virtual bool is_dominated(ApexPathPairPtr node);

    virtual void add_node(ApexPathPairPtr ap);
};

class GCL {

protected:
    std::vector<std::list<NodePtr>> gcl;

public:

    GCL(size_t graph_size):gcl(graph_size + 1) {};

    virtual bool is_dominated(NodePtr node);

    virtual void add_node(NodePtr node);
};


