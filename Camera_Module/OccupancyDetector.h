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
  bool nearDirection(const CellRuntime &cell,int peak,int gx,int gy,int tolerance);
  uint8_t directionBucket(const CellRuntime &cell,int gx,int gy);
  uint8_t spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y);
  void selectPeaks(Feature &f);
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
bool OccupancyDetector::nearDirection(const CellRuntime &cell,int peak,int gx,int gy,int tolerance) {
  const int32_t dot=abs(gx*cell.directionX[peak]+gy*cell.directionY[peak]);
  const int32_t cross=abs(gy*cell.directionX[peak]-gx*cell.directionY[peak]);
  return cross*256<=dot*tanQ8[constrain(tolerance,0,20)];
}
uint8_t OccupancyDetector::directionBucket(const CellRuntime &cell,int gx,int gy) {
  int32_t bestDot=-1;uint8_t best=MAX_PEAKS;
  for(uint8_t peak=0;peak<cell.reference.peakCount;++peak) {
    if(!nearDirection(cell,peak,gx,gy,cell.config.angleTolerance)) continue;
    const int32_t dot=abs(gx*cell.directionX[peak]+gy*cell.directionY[peak]);
    if(dot>bestDot) { bestDot=dot;best=peak; }
  }
  return best;
}
uint8_t OccupancyDetector::spatialBucket(const CellRuntime &cell,uint8_t peak,int x,int y) {
  if(peak==MAX_PEAKS) return SPATIAL_BUCKETS-1;
  // directionX/Y is the edge normal. Project the sample position onto it,
  // then retain only three broad bands to avoid exact pixel matching.
  const int32_t rhoQ8=(x-(int)cell.config.x)*cell.directionX[peak]
                    +(y-(int)cell.config.y)*cell.directionY[peak];
  const int32_t limitQ8=(int32_t)cell.config.radius*256/3;
  const uint8_t band=rhoQ8 < -limitQ8 ? 0 : rhoQ8 > limitQ8 ? 2 : 1;
  return peak*POSITION_BANDS+band;
}
void OccupancyDetector::selectPeaks(Feature &f) {
  uint32_t support[BINS]={};
  for(int i=0;i<BINS;++i) support[i]=f.hist[(i+BINS-1)%BINS]+f.hist[i]+f.hist[(i+1)%BINS];
  uint64_t selected=0;uint32_t primary=0;int primaryBin=-1;
  auto append=[&](int best,uint32_t count) {
    const float centre=best*5.0f+2.5f;
    float total=0;
    for(int delta=-1;delta<=1;++delta) {
      const int bin=(best+delta+BINS)%BINS;
      if(!f.hist[bin]) continue;
      float mean=f.angleSum[bin]/f.hist[bin];
      while(mean-centre>90) mean-=180;
      while(mean-centre< -90) mean+=180;
      total+=mean*f.hist[bin];
    }
    float refined=total/count;
    if(refined<0) refined+=180;
    if(refined>=180) refined-=180;
    bool sameFamily=false;
    for(uint8_t existing=0;existing<f.peakCount;++existing) {
      float separation=fabsf(refined-f.peakAngle[existing]);
      separation=min(separation,180.0f-separation);
      if(separation<20.0f) { sameFamily=true;break; }
    }
    if(sameFamily || f.peakCount>=MAX_PEAKS) return false;
    selected|=1ULL<<best;
    f.peakAngle[f.peakCount]=refined;
    f.peakShare[f.peakCount]=(float)count/f.edges;
    ++f.peakCount;
    return true;
  };
  for(int bin=0;bin<BINS;++bin) if(support[bin]>primary) { primary=support[bin];primaryBin=bin; }
  if(primaryBin<0 || primary<3 || primary*20<3*f.edges || !append(primaryBin,primary)) return;

  // Track sleepers are expected to be perpendicular to the dominant rail
  // family. Search that neighbourhood explicitly before applying generic
  // secondary-family thresholds. Four samples and 3% support suppress
  // isolated noise while allowing a visually clear but weaker sleeper family.
  int perpendicular=-1;uint32_t perpendicularCount=0;
  const int opposite=(primaryBin+BINS/2)%BINS;
  for(int delta=-3;delta<=3;++delta) {
    const int bin=(opposite+delta+BINS)%BINS;
    if(support[bin]>perpendicularCount) { perpendicular=bin;perpendicularCount=support[bin]; }
  }
  if(perpendicular>=0 && perpendicularCount>=4 && perpendicularCount*100>=3*f.edges)
    append(perpendicular,perpendicularCount);

  while(f.peakCount<MAX_PEAKS) {
    int best=-1;uint32_t count=0;
    for(int bin=0;bin<BINS;++bin) {
      bool close=false;
      for(int delta=-4;delta<=4;++delta)
        if(selected&(1ULL<<((bin+delta+BINS)%BINS))) close=true;
      if(!close && support[bin]>count) { best=bin;count=support[bin]; }
    }
    if(best<0 || count<3 || count*20<f.edges || count*5<primary) break;
    if(!append(best,count)) selected|=1ULL<<best;
  }
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
    if(!referenceMode && cell.reference.peakCount) {
      const uint8_t peak=directionBucket(cell,gx,gy);
      ++buckets[spatialBucket(cell,peak,x,y)];
    } else {
      float angle=atan2f((float)gy,(float)gx)*57.2957795f;
      if(angle<0) angle+=180;
      if(angle>=180) angle-=180;
      const int bin=min((int)(angle/5),BINS-1);
      ++f.hist[bin];f.angleSum[bin]+=angle;
    }
  }
  f.textured=f.edges>=8;
  if(referenceMode && f.textured) selectPeaks(f);
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
  uint16_t unused[SPATIAL_BUCKETS]={};
  cell.reference=analyse(cell,true,unused);
  memset(cell.referenceBuckets,0,sizeof(cell.referenceBuckets));
  for(int peak=0;peak<cell.reference.peakCount;++peak) {
    const float radians=cell.reference.peakAngle[peak]*0.01745329252f;
    cell.directionX[peak]=(int16_t)lroundf(cosf(radians)*256);
    cell.directionY[peak]=(int16_t)lroundf(sinf(radians)*256);
  }
  // Calculate baseline buckets with the same projected assignment as live frames.
  if(cell.reference.peakCount) {
    int x0,x1,y0,y1;bounds(cell.config,x0,x1,y0,y1);
    const int minimum=max((int)cell.config.contrastFloor,(int)cell.reference.maxGradient/5);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      if(!insideCell(cell.config,x,y)) continue;
      int gx,gy;gradientAt(pixels_,y*width_+x,gx,gy);
      if(abs(gx)+abs(gy)>=minimum) {
        const uint8_t peak=directionBucket(cell,gx,gy);
        ++cell.referenceBuckets[spatialBucket(cell,peak,x,y)];
      }
    }
  }
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
  else if(cell.reference.peakCount) score=projectionDistance(cell,live,buckets);
  else score=histogramDistance(cell.reference,live);
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
    } else if(cell.scorePermille<(uint16_t)(cell.config.thresholdPermille*0.7f)) {
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
