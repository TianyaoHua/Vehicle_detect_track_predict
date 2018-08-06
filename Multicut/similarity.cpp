//
//  similarity.cpp
//  
//
//  Created by Tingyu Mao on 7/8/18.
//

#include "similarity.hpp"

using namespace std;

float Similarity::PointFlowDistance(CTrack& ta, CTrack& tb, vector<float>& mVariances) {

    float p = 0.0;
    // Calculate distance represented by p.
    int overlapStartFrame = max(ta.startFrame, tb.startFrame);
    int overlapEndFrame = min(ta.endFrame, tb.endFrame);
    // if ta and tb have at least 2 frames in common
    if(overlapStartFrame < overlapEndFrame) {
        float d_motion_max = 0.0;
        float d_spatial = 0.0;
        float d_color = 0.0;
        for(int k=overlapStartFrame; k<overlapEndFrame+1; k++){
            // calculate motion distance
            float uta = ta.mus[k-ta.startFrame];
            float vta = ta.mvs[k-ta.startFrame];
            float utb = tb.mus[k-tb.startFrame];
            float vtb = tb.mvs[k-tb.startFrame];
            
            float d_motion = norm2(uta - utb, vta - vtb) / mVariances[k];
            d_motion_max = (d_motion_max>d_motion)? d_motion_max: d_motion;
            
            // calculate spatial distance
            float xta = ta.mxs[k-ta.startFrame];
            float yta = ta.mys[k-ta.startFrame];
            float xtb = tb.mxs[k-tb.startFrame];
            float ytb = tb.mys[k-tb.startFrame];
            
            d_spatial += norm2(xta - xtb, yta - ytb);
            
            // calculate color distance
            vector<float> colora = ta.mcolors[k-ta.startFrame];
            vector<float> colorb = tb.mcolors[k-tb.startFrame];
            float tmp=0.0;
            for(int i=0; i<colora.size(); i++){
                tmp += (colora[i] - colorb[i])*(colora[i] - colorb[i]);
            }
            d_color += sqrt(tmp);
        }

        d_spatial /= (overlapEndFrame - overlapStartFrame + 1);
        d_color /= (overlapEndFrame - overlapStartFrame + 1);
        
        // Compute z(A, B) by nonlinear function
        float z = max(theta0_hat + theta1*d_motion_max + theta2*d_spatial + theta3*d_color, theta0 + theta1*d_motion_max);

        // cout << "# z = " << z << " d_motion = " << d_motion_max << "d_spatial = " << d_spatial << "d_color = " << d_color << endl;

        p = 1.0 / (1 + exp(-1*z));
    }
    // else if the gap between ta and tb is within 5 frames
    else if(overlapEndFrame + 5 >= overlapStartFrame) {
        int dgap = overlapStartFrame - overlapEndFrame;

        float xEnd = (ta.endFrame <= tb.startFrame) ? ta.mxs.back() : tb.mxs.back();
        float yEnd = (ta.endFrame <= tb.startFrame) ? ta.mys.back() : tb.mys.back();
        float xStart = (ta.endFrame <= tb.startFrame) ? tb.mxs[0] : ta.mxs[0];
        float yStart = (ta.endFrame <= tb.startFrame) ? tb.mys[0] : ta.mys[0];
        
        float uEnd = (ta.endFrame <= tb.startFrame) ? ta.mus.back() : tb.mus.back();
        float vEnd = (ta.endFrame <= tb.startFrame) ? ta.mvs.back() : tb.mvs.back();
        
        float uStart = (ta.endFrame <= tb.startFrame) ? tb.mus[0] : ta.mus[0];
        float vStart = (ta.endFrame <= tb.startFrame) ? tb.mvs[0] : ta.mvs[0];

        float rba = norm2(xEnd + dgap*uEnd - xStart, yEnd + dgap*vEnd - yStart); //sqrt((xEnd + dgap*uEnd - xStart)*(xEnd + dgap*uEnd - xStart) + (yEnd + dgap*vEnd - yStart)*(yEnd + dgap*vEnd - yStart));
        float rab = norm2(xStart - dgap*uStart - xEnd, yStart - dgap*vStart - yEnd); //sqrt((xStart - dgap*uStart - xEnd)*(xStart - dgap*uStart - xEnd) + (yStart - dgap*vStart - yEnd)*(yStart - dgap*vStart - yEnd));

        float d_async = max(rab, rba);
        float z = d_async<20 ? 2-0.1*d_async : 0.0;
        p = 1.0 / (1 + exp(-1*z));

        // cout << "# rab = " << rab << ", rba = " << rba << endl;

    }
    return 1-p;
}

//
//float Similarity::DetectionPointFlowDistance(CTrack& pt, BBox& bbox) {
//
//    float sigma = 1.5; // threshold for d_spatial
//
//    int kFrame = bbox.t - pt.startFrame;
//    float px = pt.mxs[kFrame];
//    float py = pt.mys[kFrame];
//
//    float bx = bbox.x;
//    float by = bbox.y;
//    float bw = bbox.w;
//    float bh = bbox.h;
//	vector<cv::Point2f>& bc = bbox.contour;
//
//    float d_sp = 2*norm2( (bx - px)/bw, (by - py)/bh ); //(bx - px)*(bx - px)/(bw*bw) + (by - py)*(by - py)/(bh*bh);
//
//    float T_high = bbox.conf * ContourTemplate(px, py, bc);
//    // if(IsInsidePolygon(px, py, bc))
//    //     T_high = bbox.conf;
//
//    float pe = 0.5;
//    if(T_high > 0.5) {
//        pe = 1 - T_high;
//
//        if(d_sp > sigma){
//            cout << "Large dsp between bbox and pt: " << d_sp << endl;
//            cout << bx << " " << by << " " << bw << " " << bh << endl;
//            cout << px << " " << py << endl;
//            // DebugIsInsidePolygon(px, py, bc);
//        }
//    }
//
//    if(d_sp > sigma) {
//        pe = 1;
//    }
//
//    return pe;
//}


float Similarity::DetectionDistance(BBox& b1, BBox& b2) {

    float iou = IOU(b1, b2);
    float d_sp = 2*norm2( (b1.x - b2.x)/(b1.w + b2.w), (b1.y - b2.y)/(b1.h + b2.h) ); //(b1x - b2x)*(b1x - b2x)/( (b1w + b2w)*(b1w + b2w) ) + (b1y - b2y)*(b1y - b2y)/( (b1h + b2h)*(b1h + b2h) );

    float pe = 0.5;
    if(iou > 0.7)
        pe = 1 - 1/(1 + exp(20 * (0.7 - iou)));
    if(d_sp > 1.2)
        pe = 1/(1 + exp(5 * (1.2 - d_sp)));

    return pe;
}


// Check if a point is inside a polygon.
// Remember that the polygon must be a convex polygon.
bool Similarity::IsInsidePolygon(float x, float y, vector< Point >& polygon) {

    int n = polygon.size();
    Point p = {x, y};

    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;
 
    // Create a point for line segment from p to infinite
    Point extreme = {INF, p.y};
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
               return onSegment(polygon[i], p, polygon[next]);
 
            count++;
        }
        i = next;
    } while (i != 0);
 
    // Return true if count is odd, false otherwise
    return (count%2 == 1);

}

// Debug
bool Similarity::DebugIsInsidePolygon(float x, float y, vector< Point >& polygon) {

    int n = polygon.size();
    Point p = {x, y};

    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;
 
    // Create a point for line segment from p to infinite
    Point extreme = {INF, p.y};
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            cout << polygon[i].x << ", " << polygon[i].y << "." << polygon[next].x << ", " << polygon[next].y << endl;

            if (orientation(polygon[i], p, polygon[next]) == 0)
               return onSegment(polygon[i], p, polygon[next]);
 
            count++;
        }
        i = next;
    } while (i != 0);
 
    // Return true if count is odd, false otherwise
    return (count%2 == 1);

}

float Similarity::ContourTemplate(float x, float y, vector< Point >& contour) {

    int n = contour.size();
    float nx = 0, ny = 0;
    for(int i=0; i<n; i++) {
        float cx = contour[i].x;
        float cy = contour[i].y;

        float norm = norm2(cx - x, cy - y);
        nx += (cx - x) / norm;
        ny += (cy - y) / norm;
    }

    return 1 - norm2(nx/n, ny/n);
}


float Similarity::IOU(BBox& b1, BBox& b2) {

    // IOU = overlap / union
    float overlapW = min(b1.x + b1.w/2, b2.x + b2.w/2) - max(b1.x - b1.w/2, b2.x - b2.w/2);
    float overlapH = min(b1.y + b1.h/2, b2.y + b2.h/2) - max(b1.y - b1.h/2, b2.y - b2.h/2);

    float overlapArea = 0.0;
    if(overlapW > 0 && overlapH > 0)
        overlapArea = overlapW * overlapH;

    float unionArea = b1.w * b1.h + b2.w * b2.h - overlapArea;

    return overlapArea/unionArea;
}


// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool Similarity::onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int Similarity::orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool Similarity::doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}

////////////////////////////////////////////
// Following are obsolete code for backup.
////////////////////////////////////////////

// float Similarity::PointFlowDistance(CTrack& ta, CTrack& tb, vector<float>& mVariances) {

//     float p = 0.0;
//     // Calculate distance represented by p.
//     int overlapStartFrame = max(ta.startFrame, tb.startFrame);
//     int overlapEndFrame = min(ta.endFrame, tb.endFrame);
//     // if ta and tb have at least 2 frames in common
//     if(overlapStartFrame < overlapEndFrame) {
//         float d_motion_max = 0.0;
//         float d_spatial = 0.0;
//         float d_color = 0.0;
//         for(int k=overlapStartFrame; k<overlapEndFrame+1; k++){
//             // calculate motion distance
//             float uta = ta.mus[k-ta.startFrame];
//             float vta = ta.mvs[k-ta.startFrame];
//             float utb = tb.mus[k-tb.startFrame];
//             float vtb = tb.mvs[k-tb.startFrame];
            
//             float d_motion = norm2(uta - utb, vta - vtb) / mVariances[k]; //sqrt((uta - utb)*(uta - utb) + (vta - vtb)*(vta - vtb))/mVariances[k];
//             d_motion_max = (d_motion_max>d_motion)? d_motion_max: d_motion;
            
//             // calculate spatial distance
//             float xta = ta.mxs[k-ta.startFrame];
//             float yta = ta.mys[k-ta.startFrame];
//             float xtb = tb.mxs[k-tb.startFrame];
//             float ytb = tb.mys[k-tb.startFrame];
            
//             d_spatial += norm2(xta - xtb, yta - ytb); //sqrt((xta - xtb)*(xta - xtb) + (yta - ytb)*(yta - ytb));
            
//             // calculate color distance
//             vector<float> colora = ta.mcolors[k-ta.startFrame];
//             vector<float> colorb = tb.mcolors[k-tb.startFrame];
//             float tmp=0.0;
//             for(int i=0; i<colora.size(); i++){
//                 tmp += (colora[i] - colorb[i])*(colora[i] - colorb[i]);
//             }
//             d_color += sqrt(tmp);
//         }

//         d_spatial /= (overlapEndFrame - overlapStartFrame + 1);
//         d_color /= (overlapEndFrame - overlapStartFrame + 1);
        
//         // Compute z(A, B) by nonlinear function
//         float z = max(theta0_hat + theta1*d_motion_max + theta2*d_spatial + theta3*d_color, theta0 + theta1*d_motion_max);
//         p = 1.0 / (1 + exp(-1*z));
//     }
//     // else if the gap between ta and tb is within 5 frames
//     else if(overlapEndFrame + 5 >= overlapStartFrame) {
//         int dgap = overlapStartFrame - overlapEndFrame;
//         if(ta.endFrame <= tb.startFrame) {
//             float xEnd = ta.mxs.back();
//             float yEnd = ta.mys.back();
//             float xStart = tb.mxs[0];
//             float yStart = tb.mys[0];
            
//             float uEnd = ta.mus.back();
//             float vEnd = ta.mvs.back();
            
//             float uStart = tb.mus[0];
//             float vStart = tb.mvs[0];

//             float rab = norm2(xEnd + dgap*uEnd - xStart, yEnd + dgap*vEnd - yStart); //sqrt((xEnd + dgap*uEnd - xStart)*(xEnd + dgap*uEnd - xStart) + (yEnd + dgap*vEnd - yStart)*(yEnd + dgap*vEnd - yStart));
//             float rba = norm2(xStart - dgap*uStart - xEnd, yStart - dgap*vStart - yEnd); //sqrt((xStart - dgap*uStart - xEnd)*(xStart - dgap*uStart - xEnd) + (yStart - dgap*vStart - yEnd)*(yStart - dgap*vStart - yEnd));

//             float d_async = max(rab, rba);
//             float z = d_async<20 ? 2-0.1*d_async : 0.0;
//             p = 1.0 / (1 + exp(-1*z));
            
//         } else if (tb.endFrame <= ta.startFrame) {
//             float xEnd = tb.mxs.back();
//             float yEnd = tb.mys.back();
//             float xStart = ta.mxs[0];
//             float yStart = ta.mys[0];
            
//             float uEnd = tb.mus.back();
//             float vEnd = tb.mvs.back();
            
//             float uStart = ta.mus[0];
//             float vStart = ta.mvs[0];

//             float rab = norm2(xEnd + dgap*uEnd - xStart, yEnd + dgap*vEnd - yStart); //sqrt((xEnd + dgap*uEnd - xStart)*(xEnd + dgap*uEnd - xStart) + (yEnd + dgap*vEnd - yStart)*(yEnd + dgap*vEnd - yStart));
//             float rba = norm2(xStart - dgap*uStart - xEnd, yStart - dgap*vStart - yEnd); //sqrt((xStart - dgap*uStart - xEnd)*(xStart - dgap*uStart - xEnd) + (yStart - dgap*vStart - yEnd)*(yStart - dgap*vStart - yEnd));

//             float d_async = max(rab, rba);
//             float z = d_async<20 ? 2-0.1*d_async : 0.0;
//             p = 1.0 / (1 + exp(-1*z));
//         } else {
//             cout << "hhh" << endl;
//             cout << ta.startFrame << "," << ta.endFrame << endl;
//             cout << tb.startFrame << "," << tb.endFrame << endl;
//         }

//     }

//     return p;

// }



