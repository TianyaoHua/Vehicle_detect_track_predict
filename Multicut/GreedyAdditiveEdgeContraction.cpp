// #include "stdafx.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iterator>
#include <iostream>
using namespace std;
class Edge{
public:
	int a;
	int b;
	double w;

	Edge(int a, int b, double w) {
		/*if (a < b)
			swap(a, b);*/
		this->a = a;
		this->b = b;
		this->w = w;
	}
	bool operator< (const Edge& rhs) const {
		return rhs.w > w; //attention! this is because we want to use the reverse order!
	}
};

void remove_node(unordered_map<int, unordered_map<int, double> >& hyperG, int node) {
	hyperG.erase(node);
	for (auto& Node : hyperG) {
		Node.second.erase(node);
	}
}

void merge(unordered_map<int, unordered_map<int, double> >& hyperG,
	       unordered_map<int, unordered_set<int> >& components, 
	       priority_queue<Edge>& Q, int a, int b, int newindex) {
	
	pair<int, unordered_map<int, double> > newpoint{newindex,{}};
	
	for (const auto& adj_a : hyperG[a]) {
		if (adj_a.first != b) {
			newpoint.second[adj_a.first] += adj_a.second;
		}
	}
	//for (const auto& node : hyperG) {                                      //if single, check other nodes. because hyperG[a]'s info not complete. because single!
	//	if(node.first > a && node.second.find(a) != node.second.end())
	//		newpoint.second[node.first] += node.second.at(a);
	//}

	for (const auto& adj_b : hyperG[b]) {
		if (adj_b.first != a) {
			newpoint.second[adj_b.first] += adj_b.second;
		}
	}
	/*for (const auto node : hyperG) {
		if (node.first > b && node.first != a && node.second.find(b) != node.second.end())
			newpoint.second[node.first] += node.second.at(b);
	}*/
	for (const auto& node : newpoint.second) {
		hyperG[node.first][newindex] = node.second; //if single connected, no need to update others.
		if(node.second > 0)
			Q.push({ newindex, node.first, node.second });
	}

	remove_node(hyperG, a);
	remove_node(hyperG, b);
	hyperG.insert(newpoint);
	components[newindex] = unordered_set<int>(make_move_iterator(components[a].begin()), make_move_iterator(components[a].end()));
	components[newindex].insert(make_move_iterator(components[b].begin()), make_move_iterator(components[b].end()));
	components.erase(a);
	components.erase(b);
}

void printG(const unordered_map<int, unordered_map<int, double> >& G) {
	for (auto node : G) {
		cout << node.first << ":" << " ";
		for (auto adj : node.second) {
			cout << "(" << adj.first << "," << adj.second << ")" << " ";
		}
		cout << endl;
	}
}

void printC(const unordered_map<int, unordered_set<int> >& components) {
	for (auto hypernode : components) {
		cout << hypernode.first<<"(";
		for (auto node : hypernode.second) {
			cout << node <<" ";
		}
		cout << ")" << endl;
		cout << endl << endl;
	}
	cout << endl;
	cout << components.size() << endl;
}

unordered_map<int, unordered_set<int> > GDEC(unordered_map<int, unordered_map<int, double> >& G, /*unsigned int upperbound,*/ 
											 unsigned int lowerbound) {
	
	unordered_map<int, unordered_map<int, double> > hyperG(G); //change G to singel connected.(from higher to smaller)
	unordered_map<int, unordered_set<int> >components;
	priority_queue<Edge> Q;
	
	int newindex = hyperG.size();
	for (const auto &u : G) {
		components[u.first] = { u.first };
		for (const auto &v : G[u.first]) {
			Edge e(u.first, v.first, v.second);
			Q.push(e);
		}
	}
	while (components.size() > lowerbound && !Q.empty()) {
		Edge e = Q.top();
		Q.pop();
		//cout << "edge: " << e.a << " "<<e.b << " " << e.w << " " << endl;
		if (hyperG.find(e.a) == hyperG.end() || hyperG[e.a].find(e.b) == hyperG[e.a].end())
			continue;
		merge(hyperG, components, Q, e.a, e.b, ++newindex);
		//printC(components);
		//cout << endl << endl << endl;
		//cout << components.size();
	}
	return components;
}