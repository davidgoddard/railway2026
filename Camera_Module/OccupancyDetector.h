#pragma once
// Shared grayscale occupancy detector. No camera driver, pin map, or radio API.
// CellRuntime and GroupRuntime are the persisted protocol/runtime records.
class OccupancyDetector {
 public:
  // Provisional saturation limit; tune from frames captured on the layout.
  static constexpr uint8_t FRAME_CLIP_PERCENT=20;
  using StateCallback = void (*)(uint32_t id, bool grouped, uint8_t state, uint16_t score);
  void bind(const uint8_t *pixels, uint16_t width, uint16_t height,
            CellRuntime *cells, uint16_t cellCount, GroupRuntime *groups,
            uint16_t groupCount, StateCallback callback) {
    pixels_=pixels; width_=width; height_=height; cells_=cells;
    cellCount_=cellCount; groups_=groups; groupCount_=groupCount; callback_=callback;
  }
  void begin() {
    for(int degree=0;degree<=20;++degree)
      tanQ8[degree]=(uint16_t)lroundf(tanf(degree*0.01745329252f)*256);
  }
  void calibrateCell(CellRuntime &cell);
  void analyseAllCells();
  uint16_t inspectCell(CellRuntime &cell,Feature &live,uint16_t buckets[SPATIAL_BUCKETS]);
 private:
  const uint8_t *pixels_=nullptr;
  uint16_t width_=0,height_=0,cellCount_=0,groupCount_=0;
  CellRuntime *cells_=nullptr;
  GroupRuntime *groups_=nullptr;
  StateCallback callback_=nullptr;
  uint16_t tanQ8[21]={};
  uint32_t analysisNumber_=0;
  void emit(uint32_t id,bool grouped,uint8_t state,uint16_t score) {
    if(callback_) callback_(id,grouped,state,score);
  }
  void gradientAt(const uint8_t *pixels,int p,int &gx,int &gy);
  bool insideCell(const CellConfig &c,int x,int y);
  void bounds(const CellConfig &c,int &x0,int &x1,int &y0,int &y1);
  uint8_t directionBucket(const CellRuntime &cell,int gx,int gy);
  uint8_t spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y);
  Feature analyse(CellRuntime &cell,bool referenceMode,uint16_t buckets[SPATIAL_BUCKETS],
                  int offsetX=0,int offsetY=0);
  float histogramDistance(const Feature &a,const Feature &b);
  float projectionDistance(const CellRuntime &cell,const Feature &live,const uint16_t *buckets);
  uint16_t compareCell(CellRuntime &cell,const Feature &live,const uint16_t *buckets);
  void updateGroups();
};

void OccupancyDetector::gradientAt(const uint8_t *pixels,int p,int &gx,int &gy) {
  // 3x3 Scharr is more rotationally consistent than Sobel for these small
  // cells. Its coefficients produce four times Sobel's response, so return
  // a rounded quarter-scale result to preserve configured contrast floors.
  const int rawX=-3*pixels[p-width_-1]+3*pixels[p-width_+1]
                 -10*pixels[p-1]+10*pixels[p+1]
                 -3*pixels[p+width_-1]+3*pixels[p+width_+1];
  const int rawY=-3*pixels[p-width_-1]-10*pixels[p-width_]-3*pixels[p-width_+1]
                 +3*pixels[p+width_-1]+10*pixels[p+width_]+3*pixels[p+width_+1];
  gx=rawX<0?-((-rawX+2)/4):(rawX+2)/4;
  gy=rawY<0?-((-rawY+2)/4):(rawY+2)/4;
}
bool OccupancyDetector::insideCell(const CellConfig &c,int x,int y) {
  if(x<=0 || x>=width_-1 || y<=0 || y>=height_-1) return false;
  const int dx=x-(int)c.x,dy=y-(int)c.y;
  return c.shape==0 ? dx*dx+dy*dy<=c.radius*c.radius
                    : abs(dx)<c.radius && abs(dy)<c.radius;
}
void OccupancyDetector::bounds(const CellConfig &c,int &x0,int &x1,int &y0,int &y1) {
  x0=max(1,(int)c.x-c.radius);x1=min((int)width_-2,(int)c.x+c.radius);
  y0=max(1,(int)c.y-c.radius);y1=min((int)height_-2,(int)c.y+c.radius);
}
uint8_t OccupancyDetector::directionBucket(const CellRuntime &cell,int gx,int gy) {
  (void)cell;
  float angle=atan2f((float)gy,(float)gx)*57.2957795f+90.0f;
  if(angle<0) angle+=180;
  if(angle>=180) angle-=180;
  angle+=7.5f;if(angle>=180) angle-=180;
  return min((int)(angle/15.0f),(int)FIXED_DIRECTIONS-1);
}
uint8_t OccupancyDetector::spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y) {
  // directionX/Y is the edge normal. Project the sample position onto it,
  // then retain only three broad bands to avoid exact pixel matching.
  const int32_t rhoQ8=(x-(int)cell.config.x)*cell.directionX[peak]
                    +(y-(int)cell.config.y)*cell.directionY[peak];
  const int32_t limitQ8=(int32_t)cell.config.radius*256/3;
  const uint8_t band=rhoQ8 < -limitQ8 ? 0 : rhoQ8 > limitQ8 ? 2 : 1;
  return peak*POSITION_BANDS+band;
}
Feature OccupancyDetector::analyse(CellRuntime &cell,bool referenceMode,uint16_t buckets[SPATIAL_BUCKETS],
                                   int offsetX,int offsetY) {
  Feature f={};
  memset(buckets,0,(SPATIAL_BUCKETS)*sizeof(uint16_t));
  int x0,x1,y0,y1;bounds(cell.config,x0,x1,y0,y1);
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
    if(!insideCell(cell.config,x,y)) continue;
    const int sx=x+offsetX,sy=y+offsetY;
    if(sx<=0 || sx>=width_-1 || sy<=0 || sy>=height_-1) continue;
    const int p=sy*width_+sx;
    int gx,gy;gradientAt(pixels_,p,gx,gy);
    const int magnitude=abs(gx)+abs(gy);
    ++f.samples;f.graySum+=pixels_[p];
    if(pixels_[p]>=250) ++f.whitePixels;
    f.maxGradient=max(f.maxGradient,(uint16_t)magnitude);
  }
  if(f.maxGradient<cell.config.contrastFloor) return f;
  // Use the saved reference cutoff for live frames. A changing bright pixel
  // must not move the cutoff for every edge in an otherwise unchanged cell.
  const int minimum=max((int)cell.config.contrastFloor,
    (int)(referenceMode?f.maxGradient:cell.reference.maxGradient)/5);
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
    if(!insideCell(cell.config,x,y)) continue;
    const int sx=x+offsetX,sy=y+offsetY;
    if(sx<=0 || sx>=width_-1 || sy<=0 || sy>=height_-1) continue;
    int gx,gy;gradientAt(pixels_,sy*width_+sx,gx,gy);
    if(abs(gx)+abs(gy)<minimum) continue;
    ++f.edges;
    float angle=atan2f((float)gy,(float)gx)*57.2957795f;
    if(angle<0) angle+=180;
    if(angle>=180) angle-=180;
    const int bin=min((int)(angle/5),BINS-1);
    ++f.hist[bin];f.angleSum[bin]+=angle;
    const uint8_t direction=directionBucket(cell,gx,gy);
    ++buckets[spatialBucket(cell,direction,x,y)];
  }
  f.textured=f.edges>=8;
  if(referenceMode && f.textured) {
    f.peakCount=FIXED_DIRECTIONS;
    for(uint8_t direction=0;direction<FIXED_DIRECTIONS;++direction) {
      const float physicalAngle=direction*15.0f;
      f.peakAngle[direction]=fmodf(physicalAngle+90.0f,180.0f);
      uint32_t count=0;
      for(uint8_t band=0;band<POSITION_BANDS;++band)
        count+=buckets[direction*POSITION_BANDS+band];
      f.peakShare[direction]=(float)count/f.edges;
    }
  }
  return f;
}
float OccupancyDetector::histogramDistance(const Feature &a,const Feature &b) {
  if(!a.edges || !b.edges) return 1;
  float sum=0;
  for(int i=0;i<BINS;++i) {
    const int am=a.hist[(i+BINS-1)%BINS]+2*a.hist[i]+a.hist[(i+1)%BINS];
    const int bm=b.hist[(i+BINS-1)%BINS]+2*b.hist[i]+b.hist[(i+1)%BINS];
    sum+=fabsf((float)am/(4*a.edges)-(float)bm/(4*b.edges));
  }
  return min(1.0f,0.5f*sum);
}
float OccupancyDetector::projectionDistance(const CellRuntime &cell,const Feature &live,const uint16_t *buckets) {
  if(!cell.reference.edges || !live.edges) return 1;
  float sum=0;
  for(int i=0;i<SPATIAL_BUCKETS;++i)
    sum+=fabsf((float)cell.referenceBuckets[i]/cell.reference.edges
             -(float)buckets[i]/live.edges);
  return min(1.0f,0.5f*sum);
}
void OccupancyDetector::calibrateCell(CellRuntime &cell) {
  memset(cell.referenceBuckets,0,sizeof(cell.referenceBuckets));
  for(uint8_t direction=0;direction<FIXED_DIRECTIONS;++direction) {
    const float physicalAngle=direction*15.0f;
    const float gradientRadians=fmodf(physicalAngle+90.0f,180.0f)*0.01745329252f;
    cell.directionX[direction]=(int16_t)lroundf(cosf(gradientRadians)*256);
    cell.directionY[direction]=(int16_t)lroundf(sinf(gradientRadians)*256);
  }
  cell.reference=analyse(cell,true,cell.referenceBuckets);
  cell.state=CLEAR;cell.enterCount=cell.clearCount=0;cell.scorePermille=0;
  DEBUGF("base id=%lu group=%lu centre=(%u,%u) r=%u edges=%u angles=%u",
    (unsigned long)cell.config.id,(unsigned long)cell.config.groupId,
    cell.config.x,cell.config.y,cell.config.radius,cell.reference.edges,cell.reference.peakCount);
#if CAMERA_DEBUG_SERIAL
  for(int i=0;i<cell.reference.peakCount;++i) DEBUGF(" %.1fdeg",cell.reference.peakAngle[i]);
  DEBUGF("\n");
#endif
}

uint16_t OccupancyDetector::compareCell(CellRuntime &cell,const Feature &live,const uint16_t *buckets) {
  // Angle histograms from fewer than 16 edges are too sparse to compare reliably.
  // Treat a weak, unoriented reference as clear until the live patch has real detail.
  if(!cell.reference.peakCount && cell.reference.edges<16 && live.edges<16) return 0;
  float score=0;
  if(!cell.reference.textured && !live.textured) score=0;
  else if(cell.reference.textured!=live.textured) score=1;
  else {
    score=projectionDistance(cell,live,buckets);
    const float edgeDensity=fabsf((float)cell.reference.edges-live.edges)
      /max((float)cell.reference.edges,(float)live.edges);
    score=max(score,edgeDensity);
  }
  return (uint16_t)constrain((int)lroundf(score*1000),0,1000);
}
uint16_t OccupancyDetector::inspectCell(CellRuntime &cell,Feature &live,uint16_t buckets[SPATIAL_BUCKETS]) {
  live=analyse(cell,false,buckets);
  uint16_t score=compareCell(cell,live,buckets);
  const uint16_t toleranceTarget=cell.state==OCCUPIED
    ? (uint16_t)(cell.config.thresholdPermille*0.7f)
    : cell.config.thresholdPermille;
  for(int dy=-1;dy<=1 && score>=toleranceTarget;++dy) {
    for(int dx=-1;dx<=1 && score>=toleranceTarget;++dx) {
      if(!dx && !dy) continue;
      uint16_t shiftedBuckets[SPATIAL_BUCKETS]={};
      const Feature shifted=analyse(cell,false,shiftedBuckets,dx,dy);
      const uint16_t shiftedScore=compareCell(cell,shifted,shiftedBuckets);
      if(shiftedScore<score) {
        score=shiftedScore;live=shifted;
        memcpy(buckets,shiftedBuckets,sizeof(shiftedBuckets));
      }
    }
  }
  return score;
}
void OccupancyDetector::updateGroups() {
  for(uint16_t g=0;g<groupCount_;++g) {
    bool occupied=false,unknown=false;
    uint16_t score=0;
    for(uint16_t i=0;i<cellCount_;++i) if(cells_[i].config.groupId==groups_[g].id) {
      if(cells_[i].state==OCCUPIED) occupied=true;
      if(cells_[i].state==UNKNOWN) unknown=true;
      score=max(score,cells_[i].scorePermille);
    }
    const uint8_t next=occupied?OCCUPIED:unknown?UNKNOWN:CLEAR;
    if(next!=groups_[g].state) {
      groups_[g].state=next;
      emit(groups_[g].id,true,next,score);
    }
  }
}
void OccupancyDetector::analyseAllCells() {
  ++analysisNumber_;
  // The camera source supplies one grayscale byte per pixel. Count saturation
  // across the whole frame once, before comparing any individual sensor.
  const uint32_t total=(uint32_t)width_*height_;
  uint32_t clipped=0;
  for(uint32_t p=0;p<total;++p) if(pixels_[p]>=250) ++clipped;
  if(total && clipped*100>=total*FRAME_CLIP_PERCENT) {
    bool changed=false;
    for(uint16_t i=0;i<cellCount_;++i) {
      CellRuntime &cell=cells_[i];
      cell.enterCount=cell.clearCount=0;
      cell.scorePermille=0;
      if(cell.state!=UNKNOWN) {
        cell.state=UNKNOWN;
        emit(cell.config.id,false,UNKNOWN,0);
        changed=true;
      }
    }
    if(changed || analysisNumber_%16==0) DEBUGF(
      "unreliable frame clipped=%lu/%lu (%lu%%) threshold=%u%%\n",
      (unsigned long)clipped,(unsigned long)total,
      (unsigned long)(100UL*clipped/total),FRAME_CLIP_PERCENT);
    updateGroups();
    return;
  }
  for(uint16_t i=0;i<cellCount_;++i) {
    CellRuntime &cell=cells_[i];
    uint16_t buckets[SPATIAL_BUCKETS]={};
    const Feature live=analyse(cell,false,buckets);
    cell.scorePermille=compareCell(cell,live,buckets);
    // A one-pixel image shift can replace several edge samples in a small
    // cell. Search neighbouring image positions only when the current score
    // could enter occupancy or prevent an occupied cell from clearing.
    const uint16_t toleranceTarget=cell.state==OCCUPIED
      ? (uint16_t)(cell.config.thresholdPermille*0.7f)
      : cell.config.thresholdPermille;
    if(cell.scorePermille>=toleranceTarget) {
      for(int dy=-1;dy<=1 && cell.scorePermille>=toleranceTarget;++dy) {
        for(int dx=-1;dx<=1 && cell.scorePermille>=toleranceTarget;++dx) {
          if(!dx && !dy) continue;
          uint16_t shiftedBuckets[SPATIAL_BUCKETS]={};
          const Feature shifted=analyse(cell,false,shiftedBuckets,dx,dy);
          cell.scorePermille=min(cell.scorePermille,compareCell(cell,shifted,shiftedBuckets));
        }
      }
    }
    const uint8_t old=cell.state;
    if(cell.scorePermille>=cell.config.thresholdPermille) {
      cell.enterCount=min((int)cell.enterCount+1,255);
      cell.clearCount=0;
      if(cell.enterCount>=cell.config.enterFrames) cell.state=OCCUPIED;
    } else if(cell.state==UNKNOWN ||
              cell.scorePermille<(uint16_t)(cell.config.thresholdPermille*0.7f)) {
      cell.clearCount=min((int)cell.clearCount+1,255);
      cell.enterCount=0;
      if(cell.clearCount>=cell.config.clearFrames) cell.state=CLEAR;
    }
    if(old!=cell.state) {
      DEBUGF("trigger id=%lu group=%lu %s score=%u angles=%u edges=%u/%u max=%u/%u\n",
        (unsigned long)cell.config.id,(unsigned long)cell.config.groupId,
        cell.state==OCCUPIED?"occupied":"clear",cell.scorePermille,
        cell.reference.peakCount,live.edges,cell.reference.edges,
        live.maxGradient,cell.reference.maxGradient);
      emit(cell.config.id,false,cell.state,cell.scorePermille);
    }
  }
  updateGroups();
}
