#pragma once
// Shared grayscale occupancy detector. No camera driver, pin map, or radio API.
// CellRuntime and GroupRuntime are the persisted protocol/runtime records.
class OccupancyDetector {
 public:
  // Provisional saturation limit; tune from frames captured on the layout.
  static constexpr uint8_t FRAME_CLIP_PERCENT=20;
  using StateCallback = void (*)(uint32_t id, bool grouped, uint16_t score);
  void bind(const uint8_t *pixels, uint16_t width, uint16_t height,
            CellRuntime *cells, uint16_t cellCount, GroupRuntime *groups,
            uint16_t groupCount, StateCallback callback) {
    pixels_=pixels; width_=width; height_=height; cells_=cells;
    cellCount_=cellCount; groups_=groups; groupCount_=groupCount; callback_=callback;
    if(workSignature_ && workSignature_!=configurationSignature()) releaseWorkMap();
  }
  void begin() {
    for(uint8_t direction=0;direction<FIXED_DIRECTIONS;++direction) {
      const float radians=(direction*15.0f+90.0f)*0.01745329252f;
      directionX_[direction]=(int16_t)lroundf(cosf(radians)*256);
      directionY_[direction]=(int16_t)lroundf(sinf(radians)*256);
      classifierX_[direction]=(int16_t)lroundf(cosf(radians)*16384);
      classifierY_[direction]=(int16_t)lroundf(sinf(radians)*16384);
    }
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
  struct WorkPixel { uint16_t x,y;uint32_t firstMember; };
  WorkPixel *workPixels_=nullptr;
  uint16_t *workMembers_=nullptr;
  Feature *liveFeatures_=nullptr;
  uint16_t *liveBuckets_=nullptr;
  uint16_t *liveCutoffs_=nullptr;
  uint32_t workPixelCount_=0,workMemberCount_=0,workSignature_=0;
  int16_t directionX_[FIXED_DIRECTIONS]={},directionY_[FIXED_DIRECTIONS]={};
  int16_t classifierX_[FIXED_DIRECTIONS]={},classifierY_[FIXED_DIRECTIONS]={};
  uint32_t analysisNumber_=0;
  void emit(uint32_t id,bool grouped,uint16_t score) {
    if(callback_) callback_(id,grouped,score);
  }
  void gradientAt(const uint8_t *pixels,int p,int &gx,int &gy);
  bool insideCell(const CellConfig &c,int x,int y);
  void bounds(const CellConfig &c,int &x0,int &x1,int &y0,int &y1);
  uint8_t directionBucket(int gx,int gy);
  uint8_t spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y);
  Feature analyse(CellRuntime &cell,bool referenceMode,uint16_t buckets[SPATIAL_BUCKETS],
                  int offsetX=0,int offsetY=0);
  float projectionDistance(const CellRuntime &cell,const Feature &live,const uint16_t *buckets);
  uint16_t compareCell(CellRuntime &cell,const Feature &live,const uint16_t *buckets);
  void updateGroups();
  uint32_t configurationSignature() const;
  bool prepareWorkMap();
  void releaseWorkMap();
};

void OccupancyDetector::gradientAt(const uint8_t *pixels,int p,int &gx,int &gy) {
  // 3x3 Scharr is more rotationally consistent than Sobel for these small
  // cells. Its coefficients produce four times Sobel's response, so return
  // a rounded quarter-scale result to preserve configured contrast floors.
  const int a=pixels[p-width_-1],b=pixels[p-width_],c=pixels[p-width_+1];
  const int d=pixels[p-1],f=pixels[p+1];
  const int g=pixels[p+width_-1],h=pixels[p+width_],i=pixels[p+width_+1];
  const int rawX=3*((c-a)+(i-g))+10*(f-d);
  const int rawY=3*((g-a)+(i-c))+10*(h-b);
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
uint8_t OccupancyDetector::directionBucket(int gx,int gy) {
  uint8_t best=0;
  int32_t bestProjection=-1;
  for(uint8_t direction=0;direction<FIXED_DIRECTIONS;++direction) {
    int32_t projection=(int32_t)gx*classifierX_[direction]+(int32_t)gy*classifierY_[direction];
    if(projection<0) projection=-projection;
    if(projection>bestProjection) { bestProjection=projection;best=direction; }
  }
  return best;
}
uint8_t OccupancyDetector::spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y) {
  // directionX/Y is the edge normal. Project the sample position onto it,
  // then retain only three broad bands to avoid exact pixel matching.
  const int32_t rhoQ8=(x-(int)cell.config.x)*directionX_[peak]
                    +(y-(int)cell.config.y)*directionY_[peak];
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
    const uint8_t direction=directionBucket(gx,gy);
    ++buckets[spatialBucket(cell,direction,x,y)];
  }
  f.textured=f.edges>=8;
  return f;
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
  cell.reference=analyse(cell,true,cell.referenceBuckets);
  cell.state=CLEAR;cell.enterCount=cell.clearCount=0;cell.scorePermille=0;
  DEBUGF("base id=%lu group=%lu centre=(%u,%u) r=%u edges=%u angles=%u",
    (unsigned long)cell.config.id,(unsigned long)cell.config.groupId,
    cell.config.x,cell.config.y,cell.config.radius,cell.reference.edges,
    cell.reference.textured?FIXED_DIRECTIONS:0);
#if CAMERA_DEBUG_SERIAL
  for(int i=0;i<(cell.reference.textured?FIXED_DIRECTIONS:0);++i) DEBUGF(" %ddeg",i*15);
  DEBUGF("\n");
#endif
}

uint16_t OccupancyDetector::compareCell(CellRuntime &cell,const Feature &live,const uint16_t *buckets) {
  // Direction buckets from very few edges are too sparse to compare reliably.
  // Treat a weak reference as clear until the live patch has real detail.
  if(!cell.reference.textured && cell.reference.edges<16 && live.edges<16) return 0;
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

uint32_t OccupancyDetector::configurationSignature() const {
  uint32_t hash=2166136261UL;
  const uint16_t dimensions[]={width_,height_,cellCount_};
  const uint8_t *dimensionBytes=(const uint8_t *)dimensions;
  for(size_t i=0;i<sizeof(dimensions);++i)
    hash=(hash^dimensionBytes[i])*16777619UL;
  for(uint16_t cell=0;cell<cellCount_;++cell) {
    const uint8_t *bytes=(const uint8_t *)&cells_[cell].config;
    for(size_t i=0;i<sizeof(CellConfig);++i) hash=(hash^bytes[i])*16777619UL;
  }
  return hash?hash:1;
}

void OccupancyDetector::releaseWorkMap() {
  free(workPixels_);free(workMembers_);free(liveFeatures_);free(liveBuckets_);free(liveCutoffs_);
  workPixels_=nullptr;workMembers_=nullptr;liveFeatures_=nullptr;liveBuckets_=nullptr;liveCutoffs_=nullptr;
  workPixelCount_=workMemberCount_=workSignature_=0;
}

bool OccupancyDetector::prepareWorkMap() {
  const uint32_t signature=configurationSignature();
  if(workSignature_==signature) return true;
  releaseWorkMap();
  if(!cellCount_) { workSignature_=signature;return true; }

  const size_t frameSize=(size_t)width_*height_;
  uint8_t *mask=(uint8_t *)ps_malloc(frameSize);
  if(!mask) return false;
  memset(mask,0,frameSize);

  uint32_t uniquePixels=0,totalMembers=0;
  for(uint16_t cell=0;cell<cellCount_;++cell) {
    int x0,x1,y0,y1;bounds(cells_[cell].config,x0,x1,y0,y1);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      if(!insideCell(cells_[cell].config,x,y)) continue;
      const uint32_t pixel=(uint32_t)y*width_+x;
      if(!mask[pixel]) { mask[pixel]=1;++uniquePixels; }
      ++totalMembers;
    }
  }

  workPixels_=(WorkPixel *)ps_malloc((size_t)uniquePixels*sizeof(WorkPixel));
  workMembers_=(uint16_t *)ps_malloc((size_t)totalMembers*sizeof(uint16_t));
  liveFeatures_=(Feature *)ps_malloc((size_t)cellCount_*sizeof(Feature));
  liveBuckets_=(uint16_t *)ps_malloc((size_t)cellCount_*SPATIAL_BUCKETS*sizeof(uint16_t));
  liveCutoffs_=(uint16_t *)ps_malloc((size_t)cellCount_*sizeof(uint16_t));
  if((uniquePixels && !workPixels_) ||
     (totalMembers && !workMembers_) || !liveFeatures_ || !liveBuckets_ || !liveCutoffs_) {
    free(mask);releaseWorkMap();return false;
  }
  uint32_t nextPixel=0;
  for(uint32_t pixel=0;pixel<frameSize;++pixel) if(mask[pixel]) {
    workPixels_[nextPixel].x=pixel%width_;workPixels_[nextPixel].y=pixel/width_;
    workPixels_[nextPixel].firstMember=0;
    ++nextPixel;
  }
  free(mask);
  auto findPixel=[&](uint32_t pixel) {
    uint32_t low=0,high=uniquePixels;
    while(low<high) {
      const uint32_t middle=low+(high-low)/2;
      const uint32_t middlePixel=(uint32_t)workPixels_[middle].y*width_+workPixels_[middle].x;
      if(middlePixel<pixel) low=middle+1;else high=middle;
    }
    return low;
  };
  for(uint16_t cell=0;cell<cellCount_;++cell) {
    int x0,x1,y0,y1;bounds(cells_[cell].config,x0,x1,y0,y1);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x)
      if(insideCell(cells_[cell].config,x,y))
        ++workPixels_[findPixel((uint32_t)y*width_+x)].firstMember;
  }
  uint32_t nextMember=0;
  for(uint32_t pixel=0;pixel<uniquePixels;++pixel) {
    nextMember+=workPixels_[pixel].firstMember;
    workPixels_[pixel].firstMember=nextMember;
  }
  for(uint16_t cell=0;cell<cellCount_;++cell) {
    int x0,x1,y0,y1;bounds(cells_[cell].config,x0,x1,y0,y1);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      if(!insideCell(cells_[cell].config,x,y)) continue;
      const uint32_t index=findPixel((uint32_t)y*width_+x);
      workMembers_[--workPixels_[index].firstMember]=cell;
    }
  }
  workPixelCount_=uniquePixels;workMemberCount_=totalMembers;workSignature_=signature;
  return true;
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
      emit(groups_[g].id,true,score);
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
        emit(cell.config.id,false,0);
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
  const bool sharedPass=prepareWorkMap();
  if(sharedPass && cellCount_) {
    memset(liveFeatures_,0,(size_t)cellCount_*sizeof(Feature));
    memset(liveBuckets_,0,(size_t)cellCount_*SPATIAL_BUCKETS*sizeof(uint16_t));
    uint16_t lowestCutoff=UINT16_MAX;
    for(uint16_t cell=0;cell<cellCount_;++cell) {
      liveCutoffs_[cell]=(uint16_t)max((int)cells_[cell].config.contrastFloor,
        (int)cells_[cell].reference.maxGradient/5);
      lowestCutoff=min(lowestCutoff,liveCutoffs_[cell]);
    }
    for(uint32_t work=0;work<workPixelCount_;++work) {
      const WorkPixel &item=workPixels_[work];
      int gx,gy;gradientAt(pixels_,(uint32_t)item.y*width_+item.x,gx,gy);
      const uint16_t magnitude=(uint16_t)(abs(gx)+abs(gy));
      const bool possiblyAnEdge=magnitude>=lowestCutoff;
      const uint8_t direction=possiblyAnEdge?directionBucket(gx,gy):0;
      const uint32_t memberEnd=work+1<workPixelCount_
        ? workPixels_[work+1].firstMember : workMemberCount_;
      for(uint32_t member=item.firstMember;member<memberEnd;++member) {
        const uint16_t cellIndex=workMembers_[member];
        CellRuntime &cell=cells_[cellIndex];Feature &live=liveFeatures_[cellIndex];
        live.maxGradient=max(live.maxGradient,magnitude);
        if(!possiblyAnEdge || magnitude<liveCutoffs_[cellIndex]) continue;
        ++live.edges;
        ++liveBuckets_[(size_t)cellIndex*SPATIAL_BUCKETS+
          spatialBucket(cell,direction,item.x,item.y)];
      }
    }
    for(uint16_t cell=0;cell<cellCount_;++cell)
      liveFeatures_[cell].textured=liveFeatures_[cell].edges>=8;
  }

  for(uint16_t i=0;i<cellCount_;++i) {
    CellRuntime &cell=cells_[i];
    uint16_t fallbackBuckets[SPATIAL_BUCKETS]={};
    const Feature live=sharedPass?liveFeatures_[i]:analyse(cell,false,fallbackBuckets);
    const uint16_t *buckets=sharedPass
      ? liveBuckets_+(size_t)i*SPATIAL_BUCKETS : fallbackBuckets;
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
        cell.reference.textured?FIXED_DIRECTIONS:0,live.edges,cell.reference.edges,
        live.maxGradient,cell.reference.maxGradient);
      emit(cell.config.id,false,cell.scorePermille);
    }
  }
  updateGroups();
}
