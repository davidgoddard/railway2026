#pragma once
#include <stdint.h>
#include <stddef.h>

// VGA-coordinate window. Only presets whose dimensions preserve the 800:640
// scale exactly through OV2640's four-pixel DSP size registers are eligible.
struct CaptureWindow {
  uint16_t x=0,y=0,width=640,height=480;
  uint8_t preset=8;
  bool cropped() const { return width!=640 || height!=480; }
};
struct CaptureWindowSize { uint16_t width,height; };
constexpr CaptureWindowSize CAPTURE_WINDOW_SIZES[]={
  {96,96},{128,128},{176,144},{240,176},{240,240},
  {320,240},{320,320},{480,320},{640,480}
};

class CaptureWindowPlanner {
 public:
  void add(uint16_t x,uint16_t y,uint8_t radius) {
    if(x>=640 || y>=480 || !radius) { invalid_=true;return; }
    // One pixel for shifted comparisons plus one for the 3x3 gradient.
    const int extent=(int)radius+2;
    const int l=maximum(0,(int)x-extent),t=maximum(0,(int)y-extent);
    const int r=minimum(639,(int)x+extent),b=minimum(479,(int)y+extent);
    left_=minimum(left_,l);top_=minimum(top_,t);
    right_=maximum(right_,r);bottom_=maximum(bottom_,b);++count_;
  }
  CaptureWindow finish() const {
    if(!count_ || invalid_) return {};
    for(uint8_t i=0;i<sizeof(CAPTURE_WINDOW_SIZES)/sizeof(CAPTURE_WINDOW_SIZES[0]);++i) {
      const auto &size=CAPTURE_WINDOW_SIZES[i];
      const int x=origin(left_,right_,size.width,640);
      const int y=origin(top_,bottom_,size.height,480);
      if(x>=0 && y>=0) return {(uint16_t)x,(uint16_t)y,size.width,size.height,i};
    }
    return {};
  }
  static bool valid(const CaptureWindow &window) {
    if(window.preset>=sizeof(CAPTURE_WINDOW_SIZES)/sizeof(CAPTURE_WINDOW_SIZES[0])) return false;
    const auto &size=CAPTURE_WINDOW_SIZES[window.preset];
    return window.width==size.width && window.height==size.height &&
      window.x%16==0 && window.y%16==0 &&
      window.x+window.width<=640 && window.y+window.height<=480;
  }
 private:
  int left_=640,top_=480,right_=-1,bottom_=-1;
  unsigned count_=0;
  bool invalid_=false;
  static int minimum(int a,int b) { return a<b?a:b; }
  static int maximum(int a,int b) { return a>b?a:b; }
  static int origin(int first,int last,int size,int limit) {
    // Any origin in this interval includes the entire padded sensor extent.
    const int low=maximum(0,last+1-size),high=minimum(first,limit-size);
    const int alignedLow=(low+15)/16*16,alignedHigh=high/16*16;
    if(high<0 || alignedLow>alignedHigh) return -1;
    const int centred=maximum(0,(first+last+1-size)/2)/16*16;
    return maximum(alignedLow,minimum(centred,alignedHigh));
  }
};
