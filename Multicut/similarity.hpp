//
//  similarity.hpp
//  
//
//  Created by Tingyu Mao on 7/8/18.
//

#ifndef similarity_hpp
#define similarity_hpp

#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <limits>
#include <math.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <algorithm>
#include <ctime>
//#include "CTensor.h"
//#include "CFilter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>

#define INF 10000
 
struct Point
{
    float x;
    float y;
};

class CTrack {
public:
    CTrack() {}
    std::vector<float> mxs,mys;
    std::vector<float> mus,mvs;
    std::vector< std::vector<float> > mcolors;
    int startFrame;
    int endFrame;
    inline float squaredAvg();
};

float CTrack::squaredAvg() {
    float avgU = 0;
    float avgV = 0;
    for(int i=0; i<mus.size(); i++){
        avgU += mus[i];
        avgV += mvs[i];
    }
    avgU /= mus.size();
    avgV /= mvs.size();

    return avgU*avgU + avgV*avgV;
}

class BBox {
public:
    int t, contourLength;
    float x, y, w, h, conf;
    std::vector<cv::Point2f> contour;
    BBox() {}
};

class Similarity {
    float theta0_hat = 6;
    float theta0 = 2;
    float theta1 = -1; //-0.02;
    float theta2 = -4;
    float theta3 = -0.02;

public:
    Similarity() {}
    float PointFlowDistance(CTrack& t1, CTrack& t2, std::vector<float>& mVariances);
    //float DetectionPointFlowDistance(CTrack& pt, BBox& bbox);
    float DetectionDistance(BBox& b1, BBox& b2);
    float ContourTemplate(float x, float y, std::vector<Point>& contour);
    float IOU(BBox& b1, BBox& b2);
    float norm2(float x, float y) { return sqrt(x*x + y*y); }

    bool IsInsidePolygon(float x, float y, std::vector<Point>& contour);
    bool onSegment(Point p, Point q, Point r);
    int orientation(Point p, Point q, Point r);
    bool doIntersect(Point p1, Point q1, Point p2, Point q2);
    bool DebugIsInsidePolygon(float x, float y, std::vector< Point >& polygon);
};

#endif
