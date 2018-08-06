//
//  calculateDistance.cpp
//  
//
//  Created by Tingyu Mao on 7/8/18.
//

#include "similarity.hpp"
#include "GreedyAdditiveEdgeContraction.h"
#include "Hungarian.h"
#include "BboxCluster.h"
#include "FileUtils.h"

#include <sstream>
#include <iomanip>
#include <time.h>
#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

using namespace std;

// function for point trajectory to point trajectory distance calculation.
//void addDistanceEdge(Similarity& distanceMetric, vector<BBox>& mBBoxs, vector<CTrack>& mTracks, 
//                     vector<float>& mVariances, 
//                     unordered_map<int, unordered_map<int, double> >& distMap) {
//    int numEdge = 0;
//
//    // add pt to pt edge (low level, pt: point trajectory).
//    cout << "Adding low level edges..." << endl;
//    for(int i=0; i+1<mTracks.size(); i++){
//        CTrack ta = mTracks[i];
//
//        // if(i%100 == 0 && i>0) cout << "i is " << i << ". #Edge is " << numEdge << endl;
//
//        for(int j=i+1; j<mTracks.size(); j++){
//            float p = 0.0;
//            CTrack tb = mTracks[j];
//            
//            p = distanceMetric.PointFlowDistance(ta, tb, mVariances);
//
//            // Insert p=distance(i, j) to map[i] and map[j]
//            if(p < 1 && p > 0) {
//                float c = -log(p/(1-p));
//                distMap[i][j] = c;
//                distMap[j][i] = c;
//                numEdge += 1;
//                // cout << p << " " << c << endl;
//            }
//        }
//    }
//
//    cout << "Number of low-level edges is " << numEdge << endl;
//
//    // add bbox to bbox edge (high level).
//    numEdge = 0;
//    cout << "Adding high level edges..." << endl;
//    int bbox_init_index = mTracks.size();
//
//    for(int i=0; i+1<mBBoxs.size(); i++) {
//        BBox b1 = mBBoxs[i];
//        for(int j=i+1; j<mBBoxs.size(); j++) {
//            BBox b2 = mBBoxs[j];
//            if(b1.t == b2.t) {
//                float p = distanceMetric.DetectionDistance(b1, b2);
//                if(p < 1 && p > 0) {
//                    // insert edge
//                    float c = -log(p/(1-p));
//                    distMap[i + bbox_init_index][j + bbox_init_index] = c;
//                    distMap[j + bbox_init_index][i + bbox_init_index] = c;
//                    numEdge += 1;
//                    // cout << p << " " << c << endl;
//                }
//            }
//        }
//    }
//
//    cout << "Number of high-level edges is " << numEdge << endl;
//
//    // add bbox to pt edge (pairwise level).
//    numEdge = 0;
//    cout << "Adding pairwise edges..." << endl;
//    for(int i=0; i<mBBoxs.size(); i++) {
//        BBox bbox = mBBoxs[i];
//        for(int j=0; j<mTracks.size(); j++) {
//            CTrack pt = mTracks[j];
//            if(bbox.t >= pt.startFrame && bbox.t <= pt.endFrame) {
//                float p = distanceMetric.DetectionPointFlowDistance(pt, bbox);
//                if(p < 1 && p > 0) {
//                    // insert edge
//                    float c = -log(p/(1-p));
//                    distMap[i + bbox_init_index][j] = c;
//                    distMap[j][i + bbox_init_index] = c;
//                    numEdge += 1;
//                    // cout << p << " " << c << endl;
//                }
//            }
//        }
//    }
//
//    cout << "Number of pairwise edges is " << numEdge << endl;
//
//}

/////////////////////////////////////////
// Frame by Frame Hungarian Algorithm
/////////////////////////////////////////
void frameByframeAssociation(unordered_map<int, vector<int> >& boxes, 
                             vector< vector<double> >& IOU, 
                             unordered_map<int, int>& bboxAssignment, vector<BBox>& mBBoxs, vector<CTrack>& mTracks) {
    // Initialzie the hungarian algorithm solver
    static HungarianAlgorithm HungAlgo;
    // initialize row indexes and column indexes.
    vector<int> frameIndexes;

	//delete extra bbox
	for (auto frameBoxes : boxes) {
		frameIndexes.push_back(frameBoxes.first);
		if (frameBoxes.second.size() > 1) {
			frameBoxes.second.erase(frameBoxes.second.begin() + 1, frameBoxes.second.end());
		}//needs to be corrected!
	}
    sort(frameIndexes.begin(), frameIndexes.end());


    // declare row and column indexes
    vector<int> rowIndexes(boxes[frameIndexes[0]]); // t = 1st frame
    vector<int> colIndexes;

    // Initialize bbox_assignment (output)
    for(auto frameBoxes: boxes) {
        for(auto iter=frameBoxes.second.begin(); iter!=frameBoxes.second.end(); ++iter) {
            bboxAssignment[*iter] = *iter;
        }
    }
    // loop over frames
    for(int t=1; t<frameIndexes.size(); t++) {

        // update column elements by new frame bboxes (the frame after column frame).
		colIndexes = boxes[frameIndexes[t]]; // t = 2nd frame
        // initialize assignment
        vector<int> assignment;
        // initialize cost matrix
        int m = rowIndexes.size();
        int n = colIndexes.size();

        // declare cost matrix
        vector< vector<double> > costMatrix(m, vector<double>(n));
        for(int i = 0; i < rowIndexes.size(); ++i) {
            for(int j = 0; j < colIndexes.size(); ++j) {
                // insert IOU[ rowIndexes[i] ][ colIndexes[j] ];
				costMatrix[i][j] = 1 - IOU[rowIndexes[i]][colIndexes[j]];
                // cout << "Node " << rowIndexes[i]  << " and " << colIndexes[j] << ": " << 1 - costMatrix[i][j] << endl;
            }
        }


        // solve association by hungarian
        HungAlgo.Solve(costMatrix, assignment);

        // update rowIndexes, colIndexes
        for(int iter = 0; iter < assignment.size(); ++iter) {

            if(assignment[iter] == -1)
                // no need to update the row
                continue;

            // update column bounding boxes assignment
            int matchColIdx = colIndexes[assignment[iter]];
			bboxAssignment[matchColIdx] = bboxAssignment[rowIndexes[iter]];
            // update row element by matched column elements.
            rowIndexes[iter] = matchColIdx;
            // mark column elements as matched (negative).
            colIndexes[assignment[iter]] += 1; // avoid "0" bbox.
            colIndexes[assignment[iter]] *= -1; 
        }

        // insert new row element from unmatched column elements.
        for(auto colIdx = colIndexes.begin(); colIdx != colIndexes.end(); ++colIdx) {
            if(*colIdx >= 0) {
                int originalColIdx = *colIdx;
                rowIndexes.push_back(originalColIdx);
            }
        }
    }

	//add missing bbox
	for (auto iter = frameIndexes.begin() + 1; iter != frameIndexes.end(); ++iter) {
		if (*iter - *(iter - 1) > 1) {
			auto previous_bbox_index = boxes[*(iter - 1)][0];
			vector<BBox> previous_bbox = {mBBoxs[previous_bbox_index]};
			auto flows_in_previous_bbox = Block_Flow_Map(previous_bbox, mTracks)[0];
			for (auto t = *(iter - 1) + 1; t < *(iter); ++t) {
				vector<cv::Point2f> flow_in_t_frame;
				for (auto flow_index : flows_in_previous_bbox) {
					auto flow = mTracks[flow_index];
					if (flow.startFrame <= t && t <= flow.endFrame) { // [) or []???
						flow_in_t_frame.emplace_back(flow.mxs[t - flow.startFrame], flow.mys[t - flow.startFrame]); //embrace;
					}
				}
				vector<cv::Point2f> generated_mask;
				cv::convexHull(flow_in_t_frame, generated_mask);
				auto box_ = cv::boundingRect(generated_mask);
				BBox generated_bbox;
				generated_bbox.t = t;
				generated_bbox.contourLength = generated_mask.size();
				generated_bbox.contour = (generated_mask);
				generated_bbox.h = box_.height;
				generated_bbox.w = box_.width;
				generated_bbox.x = box_.x + box_.width/2;
				generated_bbox.y = box_.y + box_.height/2;
				generated_bbox.conf = 0.7;
				mBBoxs.push_back(generated_bbox);
				bboxAssignment[mBBoxs.size() - 1] = bboxAssignment[previous_bbox_index];

				//for debug
				//stringstream ss;
				//ss << setw(3) << setfill('0') << t;
				//string imageFile = string("C:\\My_Projects\\ProjectsWithZoran\\video_on_intersection\\multicut_test\\Loc0_4_ppm") + "\\Loc0_4_" + ss.str() + ".ppm";
				//auto image = cv::imread(imageFile);
				//for (auto flow_point : flow_in_t_frame) {
				//	cv::circle(image, flow_point, 1, cv::Scalar(100, 255, 100), -1);
				//}
				//cout << "masklength" << generated_mask.size();

				//vector<cv::Point> contour;
				//for (auto point : generated_mask) {
				//	contour.emplace_back((int)point.x, (int)point.y);
				//}
				//const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
				//int npts = cv::Mat(contour).rows;
				//cv::polylines(image, &pts, &npts, 1, true, cv::Scalar(255,0,0), 3, CV_AA, 0);

				//cv::Point p1, p2;
				//p1.x = generated_bbox.x - generated_bbox.w / 2;
				//p1.y = generated_bbox.y - generated_bbox.h / 2;
				//p2.x = generated_bbox.x + generated_bbox.w / 2;
				//p2.y = generated_bbox.y + generated_bbox.h / 2;
				//cout << p1.x << " "<< p1.y <<" "<< p2.x << " " << p2.y << endl;
				//cv::rectangle(image, p1, p2, {255, 0, 0}, 2);
				//cv::namedWindow("Debug window", cv::WINDOW_AUTOSIZE); // Create a window for display.
				//cv::imshow("Debug window", image); // Show our image inside it.
		
				//int keyBoardInput = cv::waitKey(0);
			
			}
		}
	}
}
/////////////////////////////////////////////
// check the number of bboxes in frames of a specific cluster 
/////////////////////////////////////////////
void check_number(unordered_map<int, vector<int> >& boxes) {
	for (auto box : boxes) {
		cout << box.first << ',';
		for (auto index : box.second) {
			cout << index << ',';
		}
		cout << "   ";
	}
	cout << endl << endl << endl;
}

/////////////////////////////////////////////
// Main Function
/////////////////////////////////////////////
int main(int argc, char** args) {
    
    if(argc < 8)
    {
        cout << args[0] << " <start frame> " << " <end frame> " << " <IOUThreshold> " 
             << " <detection file> " << " <tracking file> " << "<variance file> " << " <image directory> " << endl;
        exit(0);
    }

    int startFrame = atoi(args[1]);
    int endFrame = atoi(args[2]);

    float IOUThreshold = atof(args[3]);
    
    string detectFile = args[4];
    string trackFile = args[5];
    string varFile = args[6];
    string imageDir = args[7];

    vector<BBox> mBBoxs;

    int mSequenceLength;
    int mTracksNum;
    vector<CTrack> mTracks;
    vector<float> mVariances;

    // Set up timer.
    clock_t timer = clock();
    // Read variance file.
    readVariances(varFile, mVariances);
    timer = clock() - timer;
    cout << "Read mVariances.dat takes " << ((float)timer)/CLOCKS_PER_SEC << " seconds." << endl;
    // Read detection dat file.
    readDetections(detectFile, startFrame, endFrame, mBBoxs);
    timer = clock() - timer;
    cout << "Read Detection dat takes " << ((float)timer)/CLOCKS_PER_SEC << " seconds." << endl;
    // Read tracking dat file.
    readTracks(trackFile, startFrame, endFrame, mSequenceLength, mTracksNum, mTracks);
    timer = clock() - timer;
    cout << "Read Tracks dat takes " << ((float)timer)/CLOCKS_PER_SEC << " seconds." << endl;

    cout << "Successfully read variance, detection and track files." << endl;
    cout << "Length of variance: " << mVariances.size() << endl;
    cout << "Total number of bounding boxes: " << mBBoxs.size() << endl;
    cout << "Total number of trajectories: " << mTracks.size() << endl;

    // Declare distance map.
    // unordered_map<int, unordered_map<int, double> > distMap;

    // Define the similarity metric
    // Similarity similarity = Similarity();
    
    // Calculate the distance between point trajectories, bboxs, point trajectory & bbox.
    // addDistanceEdge(similarity, mBBoxs, mTracks, mVariances, distMap);

    // cout << "Number of nodes in DistMap: " << distMap.size() << endl;

    // Greedy Additive Edge Contraction
    // cout << endl;
    // cout << "Start Greedy Additive Edge Contraction..." << endl;
    // auto resMap = GDEC(distMap, lowBound);
    // printC(resMap);


    ///////////////////////////////////////////////////////////////
    // flow IOU
	cout << "calculating cluster" << endl;
	cout << mBBoxs[0].x << " " << mBBoxs[0].y<<endl;
    auto result = bbox_cluster(mBBoxs, mTracks, IOUThreshold);
    // unordered_map<int, unordered_map<int, vector<int> > > Clusters;
    // vector<vector<double> > IOU;
    auto Clusters = result.first;
    auto IOU = result.second;

    cout << "Number of cluster: " << Clusters.size() << endl;
    cout << "[DONE] Generate intial clusters by Flow IOU hard-thresholding." << endl;

    unordered_map<int, int> bboxAssignment;
    for(auto clusterBBoxes: Clusters) {
        // if clusterBBoxes contains less than 2 frames, then continue;
        if(clusterBBoxes.second.size() < 2)
            continue;
		//check_number(clusterBBoxes.second);
        frameByframeAssociation(clusterBBoxes.second, IOU, bboxAssignment, mBBoxs, mTracks);
    }
    ////////////////////////////////////////////////////////////////

    // for(auto bbox: bboxAssignment) {
    //     cout << "Cluster: " << bbox.first << " BBox: " << bbox.second << endl;
    // }

    cout << "[DONE] Frame-by-frame association." << endl;

    // Visualize
    // generate colors
    default_random_engine randomEngine;
    uniform_int_distribution<int> distribution(105, 255);
    unordered_map<int, cv::Scalar > colorMap;
    for(auto result:bboxAssignment) {
        cv::Scalar colork = cv::Scalar(distribution(randomEngine), distribution(randomEngine), distribution(randomEngine));
        colorMap[result.second] = colork;
    }

    int bbox_init_index = mTracks.size();
    cv::Mat image;
    int frame = startFrame;
    while(frame<endFrame) {
        stringstream ss;
        ss << setw(3) << setfill('0') << frame;
        string imageFile = imageDir + "/Loc0_4_" + ss.str() + ".ppm";
        image = cv::imread(imageFile);

        if( image.empty() )                      // Check for invalid input
        {
            cout <<  "Could not open or find the image " << imageFile << endl ;
            return -1;
        }

        // draw point flow
        for(auto flow: mTracks) {
            if(flow.startFrame <= frame && flow.endFrame > frame) {
                cv::circle(image, cv::Point(flow.mxs[frame - flow.startFrame], flow.mys[frame - flow.startFrame]), 1, cv::Scalar(100, 255, 100), -1);
            }
        }

        // draw bounding boxes
        for(auto result:bboxAssignment){
            BBox bbox = mBBoxs[result.first];
            if(bbox.t == frame) {
                cv::Point p1, p2;
                p1.x = bbox.x - bbox.w/2;
                p1.y = bbox.y - bbox.h/2;
                p2.x = bbox.x + bbox.w/2;
                p2.y = bbox.y + bbox.h/2;
                cv::rectangle(image, p1, p2, colorMap[result.second], 2);

                cv::putText(image, //target image
                            to_string(result.second) + ", " + to_string(result.first), //text
                            p1, //top-left position
                            cv::FONT_HERSHEY_DUPLEX,
                            0.5,
                            cv::Scalar(0, 0, 255), //font color
                            1);
            }
        }
        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE ); // Create a window for display.
        cv::imshow( "Display window", image ); // Show our image inside it.
        int keyBoardInput = cv::waitKey(0);
        if(keyBoardInput == 27) {
            frame++;
        } else {
            frame--;
            frame = max(frame, startFrame);
        }
    }
	const int* const* p;
    return 0;
}
