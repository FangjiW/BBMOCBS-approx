#pragma once

#include <iostream>

#include "Definitions.h"

class ConflictResolver
{
public:
    ConflictResolver(){};
    std::tuple<int, int, std::vector<size_t>, size_t> DetectConflict(JointPathPair &joint_path, std::vector<PathSet>& indiv_paths_list); // 0 or (i, j, node_id, t)

    void AddConstraint(std::vector<VertexConstraint>& vertex_constraints, size_t agent_id, size_t node_id, size_t time);

    void AddConstraint(std::vector<EdgeConstraint>& edge_constraints, size_t agent_id, size_t source, size_t target, size_t time);
};