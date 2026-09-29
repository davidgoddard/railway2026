#include <cassert>
#include <cstdio>
#include <algorithm>
#include <random>
#include "../../Camera_Module/CaptureWindow.h"

int main() {
  CaptureWindowPlanner empty;assert(!empty.finish().cropped());
  CaptureWindowPlanner single;single.add(328,364,13);
  auto crop=single.finish();assert(crop.width==96 && crop.height==96);
  assert(CaptureWindowPlanner::valid(crop));
  CaptureWindowPlanner corners;
  corners.add(5,5,3);corners.add(634,5,3);corners.add(5,474,3);corners.add(634,474,3);
  assert(!corners.finish().cropped());
  CaptureWindowPlanner invalid;invalid.add(640,480,10);assert(!invalid.finish().cropped());
  std::mt19937 rng(42);
  for(int trial=0;trial<10000;++trial) {
    CaptureWindowPlanner planner;
    int left=640,top=480,right=-1,bottom=-1;
    for(unsigned i=0,count=1+rng()%4;i<count;++i) {
      const int x=rng()%640,y=rng()%480,radius=3+rng()%48;
      planner.add(x,y,radius);
      left=std::min(left,std::max(0,x-radius-2));right=std::max(right,std::min(639,x+radius+2));
      top=std::min(top,std::max(0,y-radius-2));bottom=std::max(bottom,std::min(479,y+radius+2));
    }
    crop=planner.finish();assert(CaptureWindowPlanner::valid(crop));
    assert(crop.x<=left && crop.y<=top && crop.x+crop.width>right && crop.y+crop.height>bottom);
    // Verify minimal eligible preset independently by enumerating every aligned origin.
    for(unsigned i=0;i<crop.preset;++i) {
      const auto &size=CAPTURE_WINDOW_SIZES[i];
      bool fits=false;
      for(int y=0;y+size.height<=480;y+=16) for(int x=0;x+size.width<=640;x+=16)
        fits|=x<=left && y<=top && x+size.width>right && y+size.height>bottom;
      assert(!fits);
    }
  }
  std::puts("Capture window coverage, margins, alignment, minimality and corner fallback passed");
}
