#pragma once

#include "Definitions.h"


template <typename T>
bool less_than_pair(const T &a, const T &b)
{
    auto vec1 = a.first, vec2 = b.first;
    for(int i = 0; i < vec1.size(); i ++){
        if(vec1.at(i) != vec2.at(i)){
            return vec1.at(i) < vec2.at(i);
        }
    }
    return true;
}


struct less_than_vector
{
    bool operator()(const CostVector &a, const CostVector &b){
        for(int i = 0; i < a.size(); i++){
            if(a.at(i) != b.at(i)){
                return a.at(i) < b.at(i);
            }
        }
        return true;
    }
};


inline CostVector vector_min(CostVector& a, CostVector& b){
    CostVector  min_vector(a.size());
    for(int i = 0; i < a.size(); i++){
        min_vector.at(i) = a.at(i) <= b.at(i) ? a.at(i) : b.at(i);
    }
    return min_vector;
}

inline double calculate_BF(CostVector& a, CostVector& b){
    double eps = 0;
    for(int i = 0; i < a.size(); i++){
        eps = double(b.at(i)+0.001)/double(a.at(i)) - 1 > eps ? double(b.at(i)+0.001)/double(a.at(i)) - 1 : eps;
    }
    return eps;
}