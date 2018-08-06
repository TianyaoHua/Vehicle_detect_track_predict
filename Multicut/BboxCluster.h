#pragma once
#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "similarity.hpp"

using namespace std;

// struct Point
// {
// 	float x;
// 	float y;
// };
// class CTrack {
// public:
// 	CTrack() {}
// 	std::vector<float> mxs, mys;
// 	std::vector<float> mus, mvs;
// 	std::vector< std::vector<float> > mcolors;
// 	int startFrame;
// 	int endFrame;
// 	inline float squaredAvg();
// };

// class BBox {
// public:
// 	int t, contourLength;
// 	float x, y, w, h, conf;
// 	std::vector<Point> contour;
// 	BBox() {}
// };

typedef vector<vector<int> >::size_type node; //node is represented in int
typedef vector<CTrack>::size_type track_index;//track index is represented by the location in ctracks vector
typedef vector<BBox>::size_type bbox_index;//bbox index is also represented by the lobation in bboxes vector

double IOU(const BBox& bbox1, const BBox& bbox2, const bbox_index bbox1_index, const bbox_index bbox2_index, const vector<CTrack>& ctracks,
	const unordered_map<int, vector<int> >& block_flow_map);

void DFS(vector<vector<double> >&G, vector<double>& label, int u);

vector<int> SCC(vector<vector<double> >&G);

pair< unordered_map<int, unordered_map<int, vector<int> > >, vector<vector<double> > > bbox_cluster(const vector<BBox>& bboxes, const vector<CTrack>& ctracks, double thres);

unordered_map<int, vector<int> > Block_Flow_Map(const vector<BBox>& bboxes, const vector<CTrack>& ctracks);