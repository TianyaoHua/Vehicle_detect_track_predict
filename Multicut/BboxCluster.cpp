#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "BboxCluster.h"

using namespace std;


// float CTrack::squaredAvg() {
// 	float avgU = 0;
// 	float avgV = 0;
// 	for (int i = 0; i<mus.size(); i++) {
// 		avgU += mus[i];
// 		avgV += mvs[i];
// 	}
// 	avgU /= mus.size();
// 	avgV /= mvs.size();

// 	return avgU * avgU + avgV * avgV;
// }

double IOU(const BBox& bbox1, const BBox& bbox2, const bbox_index bbox1_index, const bbox_index bbox2_index, const vector<CTrack>& ctracks, 
	const unordered_map<int, vector<int> >& block_flow_map) {
	if (bbox1.t == bbox2.t)
		return 0; //if two bboxes are in the same frame, they have to represent two different things.
	unsigned int n_flows_common = 0; //the number of common flows.
	//growing match. index in block_flow_map is strictly Monotonically increasing.
	track_index bbox1_track_index = 0, bbox2_track_index = 0;
	track_index n_bbox1_track = block_flow_map.at(bbox1_index).size();
	track_index n_bbox2_track = block_flow_map.at(bbox2_index).size();
	while (bbox1_track_index < n_bbox1_track && bbox2_track_index < n_bbox2_track) {
		if (block_flow_map.at(bbox1_index).at(bbox1_track_index) == block_flow_map.at(bbox2_index).at(bbox2_track_index)) {
			n_flows_common += 1;
			bbox1_track_index += 1;
			bbox2_track_index += 1;
		}
		else if (block_flow_map.at(bbox1_index).at(bbox1_track_index) > block_flow_map.at(bbox2_index).at(bbox2_track_index)) {
			bbox2_track_index += 1;
		}
		else {
			bbox1_track_index += 1;
		}
	}
	return static_cast<double>(n_flows_common) / (n_bbox1_track+ n_bbox2_track - n_flows_common);
}

void DFS(vector<vector<double> >&G, vector<int>& label, int u) {
	for (node adj = 0; adj < G[u].size(); ++adj) {
		if (G[u][adj] && (!label[adj])) { //if G[u][adj] is 0, i and j are disconnected. Otherwise it is connected.
		//if (!label[adj]) {
			label[adj] = label[u];
			DFS(G, label, adj);
		}
	}
}

vector<int> SCC(vector<vector<double> >&G) {
	//the G is undirected graph. Omit the first DFS.
	vector<int> label(G.size());
	int current_label = 1;
	for (node u = 0; u < G.size(); ++u) {
		if (!label[u]) {
			label[u] = ++current_label;
			DFS(G, label, u);
		}
	}
	return label;
}

unordered_map<int, vector<int> > Block_Flow_Map(const vector<BBox>& bboxes, const vector<CTrack>& ctracks) {
	unordered_map<int, vector<int> >block_flow_map;
	for (bbox_index j = 0; j < bboxes.size(); ++j) {
		block_flow_map[j] = {};
		BBox bbox = bboxes[j];
		//cout << "j:" << j << endl;
		for (track_index i = 0; i < ctracks.size(); ++i) {
			//cout << "i:" << i << endl;
			if (ctracks[i].startFrame <= bboxes[j].t && bboxes[j].t < ctracks[i].endFrame) {
				auto mx = ctracks[i].mxs[bbox.t - ctracks[i].startFrame];
				auto my = ctracks[i].mys[bbox.t - ctracks[i].startFrame];
				double  in_bbox = cv::pointPolygonTest(bbox.contour, cv::Point2f(mx, my), false);
				if (in_bbox == 1)
					block_flow_map[j].push_back(i); //the jth bboex contains the ith ctrack
			}
		}
	}
	return block_flow_map;
}

pair< unordered_map<int, unordered_map<int, vector<int> > >, vector<vector<double> > > bbox_cluster(const vector<BBox>& bboxes, const vector<CTrack>& ctracks, double thres) {
	vector<BBox>::size_type N = bboxes.size();
	vector<vector<double> > G(N, vector<double>(N));  //G[i][j] is the iou between ith and jth bbox. 
	unordered_map<int, vector<int> >block_flow_map;

	cout << "prepare the bbox_tracks map...";
	cout << "ctracks.size()" << ctracks.size() << endl;
	//prepare the bbox_tracks map.

	block_flow_map = Block_Flow_Map(bboxes, ctracks);
	cout << "[DONE] prepare the bbox_tracks map..." << endl;

	//Calculate the iou between bboxes.
	for (node i = 0; i < N; ++i) {
		for (node j = i + 1; j < N; ++j) { //only calculate half of them. The other half is symetric.
			auto iou = IOU(bboxes[i], bboxes[j], i, j, ctracks, block_flow_map);
			G[i][j] = (iou > thres ? iou : 0); // if iou greater than threshould, keep it. otherwise set it to be 0, meaning i and j are disconneted.
			G[j][i] = G[i][j];
		}
	}

	cout << "[DONE] generate IOU..." << endl;


	auto label = SCC(G); // the label of each bbox.

	cout << "[DONE] cluster by IOU..." << endl;


	unordered_map<int, unordered_map<int, vector<int> > > cluster;
	for (node u = 0; u < label.size(); ++u) {
		cluster[label[u]][bboxes[u].t].push_back(u);
	}
	return {cluster, G};
}