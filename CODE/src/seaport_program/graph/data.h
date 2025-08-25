#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <map>

#include <unordered_map>
#include <numeric>

using index_type = std::vector<int>;

struct index_hash
{
    std::size_t operator()(index_type const &i) const noexcept
    {
        auto const hash_combine = [](auto seed, auto x)
        {
            return std::hash<int>()(x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
        return std::accumulate(i.begin() + 1, i.end(), i[0], hash_combine);
    }
};

template <typename T>
using sparse_array = std::unordered_map<index_type, T, index_hash>;

struct Pairwise
{
    int demand;
    SCIP_CONS *path_constraint;
    std::vector<std::pair<std::vector<int>, SCIP_VAR *>> path_with_variable;
};

struct Aircraft
{
    std::string name;
    double range;
    int seat_capacity;
    bool is_electric;
    double D1;
    double D2;
    double D3;
    double D4;
    double F1;
    double F2;
    double F3;
    std::vector<int> compatible_airports;
};