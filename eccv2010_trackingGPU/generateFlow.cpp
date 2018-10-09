#include <string>
#include <vector>
#include <algorithm>
#include <ctime>
#include "CTensor.h"
#include "CFilter.h"
#include "ldof.h"

class CTrack {
public:
  CTrack() {mStopped = false; mLabel = -1;}
  std::vector<float> mx,my;             // current position of the track
  std::vector<float> mu,mv;             // current flow of the track
  std::vector< std::vector<float> > mcolor;  // current LAB color of the track
  int mox,moy;                          // original starting point of the track
  int mLabel;                           // assignment to a region (ignored for tracking but makes the tracking files compatible to other tools I have)
  bool mStopped;                        // tracking stopped due to occlusion, etc.
  int mSetupTime;                       // Time when track was created
};

CVector<std::string> mInput;
std::string mFilename;
std::string mInputDir;
std::string mResultDir;
std::vector<CTrack> mTracks;
int mStartFrame;
int mStep;
int mSequenceLength;
int mXSize,mYSize;
CMatrix<float> mColorCode;
std::ofstream mStatus;

std::vector<float> mVariance;

// calculate standard deviation
float calStd(CMatrix<float> matrix){
    int wholeSize = matrix.size();
    float avg = matrix.avg();
    float std = 0;
    for(int i=0; i<wholeSize; i++){
        std += (matrix.data()[i]-avg)*(matrix.data()[i]-avg);
    }
    return std/wholeSize;
}

// buildColorCode
void buildColorCode() {
  mColorCode.setSize(3,15000);
  for (int i = 0; i < 256; i++) {
    mColorCode(0,i) = 0;
    mColorCode(1,i) = i;
    mColorCode(2,i) = 255;
  }
  for (int i = 0; i < 256; i++) {
    mColorCode(0,i+256) = 0;
    mColorCode(1,i+256) = 255;
    mColorCode(2,i+256) = 255-i;
  }
  for (int i = 0; i < 256; i++) {
    mColorCode(0,i+512) = i;
    mColorCode(1,i+512) = 255;
    mColorCode(2,i+512) = 0;
  }
  for (int i = 0; i < 256; i++) {
    mColorCode(0,i+768) = 255;
    mColorCode(1,i+768) = 255-i;
    mColorCode(2,i+768) = 0;
  }
  for (int i = 0; i < 256; i++) {
    mColorCode(0,i+1024) = 255;
    mColorCode(1,i+1024) = 0;
    mColorCode(2,i+1024) = i;
  }
  for (int i = 1280; i < mColorCode.ySize(); i++) {
    mColorCode(0,i) = 255;
    mColorCode(1,i) = 0;
    mColorCode(2,i) = 255;
  }
}

// readMiddlebury
bool readMiddlebury(const char* aFilename, CTensor<float>& aFlow) {
  FILE *stream = fopen(aFilename, "rb");
  if (stream == 0) {
    std::cout << "Could not open " << aFilename << std::endl;
    return false;
  }
  float help;
  int dummy;
  dummy = fread(&help,sizeof(float),1,stream);
  int aXSize,aYSize;
  dummy = fread(&aXSize,sizeof(int),1,stream);
  dummy = fread(&aYSize,sizeof(int),1,stream);
  aFlow.setSize(aXSize,aYSize,2);
  for (int y = 0; y < aFlow.ySize(); y++)
    for (int x = 0; x < aFlow.xSize(); x++) {
      dummy = fread(&aFlow(x,y,0),sizeof(float),1,stream);
      dummy = fread(&aFlow(x,y,1),sizeof(float),1,stream);
    }
  fclose(stream);
  return true;
}

// writeMiddlebury
bool writeMiddlebury(const char* aFilename, CTensor<float>& aFlow) {
  FILE *stream = fopen(aFilename, "wb");
  if (stream == 0) {
    std::cout << "Could not open " << aFilename << std::endl;
    return false;
  }
  float help=202021.25;
  int dummy;
  dummy = fwrite(&help,sizeof(float),1,stream);
  int aXSize = aFlow.xSize();
  int aYSize = aFlow.ySize();
  fwrite(&aXSize,sizeof(int),1,stream);
  fwrite(&aYSize,sizeof(int),1,stream);
  //aFlow.setSize(aXSize,aYSize,2);
  for (int y = 0; y < aFlow.ySize(); y++)
    for (int x = 0; x < aFlow.xSize(); x++) {
      fwrite(&aFlow(x,y,0),sizeof(float),1,stream);
      fwrite(&aFlow(x,y,1),sizeof(float),1,stream);
    }
  fclose(stream);
  return true;
}

class CPoint {
public:
  CPoint() {}
  float x,y,frame;
};

class CSimpleTrack {
public:
  CSimpleTrack() {}
  int mLabel;
  CVector<CPoint> mPoints;
};

// write Variance
void writeVarience() {
    char buffer[50];
    sprintf(buffer,"Variance%d.dat",mSequenceLength);
    std::ofstream aFile((mResultDir+mFilename+buffer).c_str());
    for(int i=0; i<mSequenceLength-1; i++){
        aFile << mVariance[i] << std::endl;
    }
}

// Code fragment from Pedro Felzenszwalb  ---------------
// http://people.cs.uchicago.edu/~pff/dt/
void dt(CVector<float>& f, CVector<float>& d, int n) {
  d.setSize(n);
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -10e20;
  z[1] = 10e20;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+q*q)-(f(v[k])+v[k]*v[k]))/(2*(q-v[k]));
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+q*q)-(f(v[k])+v[k]*v[k]))/(2*(q-v[k]));
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = 10e20;
  }
  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    int help = q-v[k];
    d(q) = help*help + f(v[k]);
  }
  delete[] v;
  delete[] z;
}


int main(int argc, char** args) {

    if(argc < 5)
    {
        std::cout<<args[0]<<" <bmf file> <start frame number (starts from 0)> <Sequence length> <subsampling factor (1 for dense, >1 for subsampled - integer only) > "<<std::endl;
        exit(0);
    }
    clock_t fileopstart,fileop=0;
     CVector<float>color = CVector<float>(3);
  // Determine input directory
  std::string s = args[1];
  s.erase(s.find_last_of("/")+1,s.length());
  mInputDir = s;
  s = args[1];
  s.erase(0,s.find_last_of('.'));
  // Read image sequence in bmf format
  if (s == ".bmf" || s == ".BMF") {
    int aImageCount,aViewCount;
    std::ifstream aStream(args[1]);
    aStream >> aImageCount;
    aStream >> aViewCount;
    mInput.setSize(aImageCount);
    for (int i = 0; i < aImageCount; i++) {
      std::string s;
      aStream >> s;
      mInput(i) = mInputDir+s;
    }
  }
  else {
    std::cout << "Must pass a bmf file as input" << std::endl;
    return -1;
  }
  // Determine image/sequence name
  s = args[1];
  s.erase(0,s.find_last_of("/")+1);
  s.erase(s.find_first_of('.'));
  mFilename = s;
  // Make directory for results
  mResultDir = mInputDir + mFilename + "Results";
  std::string s2 = "mkdir " + mResultDir;
  int dummy = system(s2.c_str());
  mResultDir += "/";
  buildColorCode();
  // Set beginning and end of tracking
  mStartFrame = atoi(args[2]);
  mSequenceLength = atoi(args[3]);
  mStep = atoi(args[4]);
  char buffer[100];
  sprintf(buffer,"Status%d.txt",mSequenceLength);
  mStatus.open((mResultDir+buffer).c_str());
  clock_t start = clock();
  // Load first image
  CTensor<float>* aImage1 = new CTensor<float>;
  CTensor<float>* aImage2;
  fileopstart = clock();
  std::cout<<mInput(mStartFrame).c_str()<<std::endl;
  aImage1->readFromPPM(mInput(mStartFrame).c_str());
  std::cout<<mInput(mStartFrame).c_str()<<std::endl;
  fileop += clock() - fileopstart;
  mXSize = aImage1->xSize();
  mYSize = aImage1->ySize();
  mStatus << "Tracks are being computed..." << std::endl;
  mTracks.clear();
  CMatrix<float> aCorners;
  CMatrix<float> aCovered(mXSize,mYSize);
  int aSize = mXSize*mYSize;
  // Smooth first image (can be removed from optical flow computation then)
  NFilter::recursiveSmoothX(*aImage1,0.8f);
  NFilter::recursiveSmoothY(*aImage1,0.8f);
  // Tracking
  for (int t = 0; t < mSequenceLength-1; t++) {
    // Load next image
    aImage2 = new CTensor<float>;
    fileopstart = clock();
    aImage2->readFromPPM(mInput(mStartFrame+t+1).c_str());
    std::cout<<mInput(mStartFrame+t+1).c_str()<<std::endl;
    fileop += clock() - fileopstart;
    NFilter::recursiveSmoothX(*aImage2,0.8f);
    NFilter::recursiveSmoothY(*aImage2,0.8f);

    // Compute bidirectional LDOF or read from file when available
    ///* Replace this part by calling the GPU implementation
    CTensor<float> aForward,aBackward;
    sprintf(buffer,"ForwardFlow%03d.flo",t+mStartFrame);
    //sprintf(buffer,"tennis%03dForward.flo",t+mStartFrame);
    std::string aName1 = mResultDir+buffer;
    sprintf(buffer,"BackwardFlow%03d.flo",t+mStartFrame);
    //sprintf(buffer,"tennis%03dBackward.flo",t+mStartFrame);
    std::string aName2 = mResultDir+buffer;
    if (readMiddlebury(aName1.c_str(),aForward) == false || readMiddlebury(aName2.c_str(),aBackward) == false) 
    {
      mStatus << "Compute optical flow. Frame" << t << std::endl;
      ldof(*aImage1,*aImage2,aForward,aBackward);
      writeMiddlebury(aName1.c_str(), aForward);
      writeMiddlebury(aName2.c_str(), aBackward);
    }
    
    // Calculate variance of flow
    CMatrix<float> flowU(mXSize, mYSize), flowV(mXSize, mYSize);
    aForward.getMatrix(flowU, 0);
    aForward.getMatrix(flowV, 1);
      
    float sigmaU = calStd(flowU);
    float sigmaV = calStd(flowU);
    
    mVariance.push_back(sqrt(sigmaU + sigmaV));
    // Prepare for next image
    delete aImage1; aImage1 = aImage2;
  }
  delete aImage1;
  
  // Write variance to file
  writeVarience();
    
  clock_t finish = clock();
  mStatus << (double(finish)-double(start)-(double)fileop)/CLOCKS_PER_SEC << std::endl;
  mStatus.close();
  return 0;
}

