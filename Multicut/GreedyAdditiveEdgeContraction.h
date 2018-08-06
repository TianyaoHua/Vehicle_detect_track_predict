#pragma once
// #include "stdafx.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iterator>
#include <iostream>
using namespace std;
class Edge;
void remove_node(unordered_map<int, unordered_map<int, double> >& hyperG, int node);
void merge(unordered_map<int, unordered_map<int, double> >& hyperG,
	unordered_map<int, unordered_set<int> >& components, priority_queue<Edge>& Q, int a, int b, int newindex);
void printG(const unordered_map<int, unordered_map<int, double> >& G);
void printC(const unordered_map<int, unordered_set<int> >& components);
unordered_map<int, unordered_set<int> > GDEC(unordered_map<int, unordered_map<int, double> >& G, /*unsigned int upperbound,*/ unsigned int lowerbound);