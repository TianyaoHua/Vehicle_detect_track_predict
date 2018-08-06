#include "similarity.hpp"

#include <sstream>
#include <iomanip>

using namespace std;

vector<float> line2VectorFloat(string line);
void line2LabelAndLength(string line, int& mLabel, int& mTrackLength);
void line2XYColorFrame(string line, float& x, float& y, float& u, float& v, vector<float>& color, int& frame);
void line2BBox(string line, BBox& bbox);

void readTracks(string fileName, int startFrame, int endFrame, int& mSequenceLength, int& mTracksNum, vector<CTrack>& mTracks);
void readVariances(string fileName, vector<float>& mVariances);
void readDetections(string fileName, int startFrame, int endFrame, vector<BBox>& mBBoxs);
