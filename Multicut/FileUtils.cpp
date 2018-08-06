#include "FileUtils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// Optimization 1: 
// read large file optimization: using istream.getline and char*. 
// Compared with the old getline methods, it is about 1.5 times faster.

//vector<float> line2VectorFloat(char* cstr) {
//    vector<float> result;
//    char *token = strtok(cstr, " ");
//
//    while(token != NULL) {
//        result.push_back(atof(token));
//        token = strtok(NULL, " ");
//    }
//	
//    return result;
//
//}

vector<float> line2VectorFloat(char* cstr) {
	char delim = ' ';
	stringstream ss(cstr);
	string item;
	vector<float> result;
	while (getline(ss, item, delim)) {
		result.push_back(stof(item));
	}
	return result;
}

void line2LabelAndLength(char* line, int& mLabel, int& mTrackLength){
    
    vector<float> result = line2VectorFloat(line);
    mLabel = result[0];
    mTrackLength = result[1];
}

void line2XYColorFrame(char* line, float& x, float& y, float& u, float& v, vector<float>& color, int& frame){

    vector<float> result = line2VectorFloat(line);
    x = result[0];
    y = result[1];
    u = result[2];
    v = result[3];
    color.push_back(result[4]);
    color.push_back(result[5]);
    color.push_back(result[6]);
    frame = result[7];

}

void line2BBox(char* line, BBox& bbox){

    float ratiox = 1200./1920.;
    float ratioy = 675./1080.;

    vector<float> result = line2VectorFloat(line);
    bbox.t = result[0];
    bbox.x = ratiox * result[1];
    bbox.y = ratioy * result[2];
    bbox.w = ratiox * result[3];
    bbox.h = ratioy * result[4];
    bbox.contourLength = result[5];
    bbox.conf = result[6];
    
}

void readTracks(string fileName, int startFrame, int endFrame, int& mSequenceLength, int& mTracksNum, vector<CTrack>& mTracks) {
    
    const int MAX_LENGTH = 524288;
	char* line = new char[MAX_LENGTH];
    //string line;
    
    ifstream inFile;
    inFile.open(fileName);
    
    if(!inFile.is_open())
        cout << "Error opening " << fileName << endl;
    
    int cnt=0;
    inFile.getline(line, MAX_LENGTH);
    mSequenceLength = atoi(line);
    inFile.getline(line, MAX_LENGTH);
    mTracksNum = atoi(line);
    
    for(int i=0; i<mTracksNum; i++){
        int mLabel, mTrackLength;
        inFile.getline(line, MAX_LENGTH);
        line2LabelAndLength(line, mLabel, mTrackLength);
        
        // read the line containing the original point.
        inFile.getline(line, MAX_LENGTH);
        bool skip = false;
        vector<float> originalPointInfo = line2VectorFloat(line);
        if(originalPointInfo[2] + mTrackLength - 2 < startFrame)
            skip = true;
        
        CTrack oneTrack;
        for(int j=0; j<mTrackLength-1; j++){
            inFile.getline(line, MAX_LENGTH);

            if(skip) continue;

            float mx, my, mu, mv;
            vector<float> mcolor;
            int mframe;

            line2XYColorFrame(line, mx, my, mu, mv, mcolor, mframe);
            
            oneTrack.mxs.push_back(mx);
            oneTrack.mys.push_back(my);
            oneTrack.mus.push_back(mu);
            oneTrack.mvs.push_back(mv);
            oneTrack.mcolors.push_back(mcolor);
            
            if(j == 0){
                oneTrack.startFrame = mframe;
                oneTrack.endFrame = mframe + mTrackLength - 2;
            }
        }
        // keep valid (has at least two frame) and moving trajectories.
        // Also, point trajectory should have overlapped interval with [startFrame, endFrame].
        int overlapStartFrame = max(oneTrack.startFrame, startFrame);
        int overlapEndFrame = min(oneTrack.endFrame, endFrame);
        if(mTrackLength-2 > 0 && oneTrack.squaredAvg() > 1e-3 && overlapStartFrame < overlapEndFrame)
            mTracks.push_back(oneTrack);

        if(oneTrack.startFrame > endFrame) {
            cout << "End frame is " << endFrame << endl;
            cout << "Early stop at Track " << i << " at frame " << oneTrack.startFrame << endl;
            break;
        }
    }
    
}

void readVariances(string fileName, vector<float>& mVariances) {

	const int MAX_LENGTH = 524288;
	char* line = new char[MAX_LENGTH];
    
    mVariances.push_back(0.0);

    ifstream inFile;
    inFile.open(fileName);
    
    if(!inFile.is_open())
        cout << "Error opening " << fileName << endl;
    
    for(; inFile.getline(line, MAX_LENGTH); ){
        mVariances.push_back(atof(line));
    }
    mVariances.pop_back();
}

void readDetections(string fileName, int startFrame, int endFrame, vector<BBox>& mBBoxs) {

    ifstream inFile;
    inFile.open(fileName);

    if(!inFile.is_open())
        cout << "Error opening " << fileName << endl;

    const int MAX_LENGTH = 524288;
	char* line = new char[MAX_LENGTH];

    for(; inFile.getline(line, MAX_LENGTH); ){

        BBox oneBBox;
        line2BBox(line, oneBBox);
        for(int i=0; i<oneBBox.contourLength; i++) {
            // get contour lines
            inFile.getline(line, MAX_LENGTH);
            // read contour x and y
            vector<float> result = line2VectorFloat(line);
            float ratiox = 1200./1920.;
            float ratioy = 675./1080.;
            
            cv::Point2f ci{ratiox*result[0], ratioy*result[1]};

            oneBBox.contour.push_back(ci);
        }

        // only keep valid bounding boxes.
        if(oneBBox.t >= startFrame && oneBBox.t < endFrame)
            mBBoxs.push_back(oneBBox);
    }

}
