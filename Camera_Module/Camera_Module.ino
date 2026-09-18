#define CAMERA_MODULE_VERSION "0.1.3"
#define CAMERA_DEBUG_SERIAL 1
#define CAMERA_BOARD_AI_THINKER 1
// #define CAMERA_BOARD_ESP32S3_EYE 1

// Railway camera firmware. ESP-NOW uses the Wi-Fi radio without an IP network.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <miniz.h>
#include <esp_camera.h>
#include <LittleFS.h>
#include <esp_partition.h>
#include <math.h>
#include <stddef.h>

#if defined(CAMERA_BOARD_AI_THINKER) && defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select one camera board
#endif
#if !defined(CAMERA_BOARD_AI_THINKER) && !defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select a camera board and add its pin map
#endif

#if defined(CAMERA_BOARD_AI_THINKER)
constexpr int PWDN=32, RESET=-1, XCLK=0, SIOD=26, SIOC=27;
constexpr int D0=5,D1=18,D2=19,D3=21,D4=36,D5=39,D6=34,D7=35;
constexpr int VSYNC=25,HREF=23,PCLK=22, STATUS_LED=4; // AI Thinker flash LED
#else
constexpr int PWDN=-1, RESET=-1, XCLK=15, SIOD=4, SIOC=5;
constexpr int D0=11,D1=9,D2=8,D3=10,D4=12,D5=18,D6=17,D7=16;
constexpr int VSYNC=6,HREF=7,PCLK=13, STATUS_LED=3; // S3-EYE LED: verify board revision
#endif

#if CAMERA_DEBUG_SERIAL
#define DEBUGF(...) Serial.printf(__VA_ARGS__)
#define DEBUGLN(s) Serial.println(s)
#else
#define DEBUGF(...) do {} while (0)
#define DEBUGLN(s) do {} while (0)
#endif

constexpr uint16_t MAGIC=0x5243;
constexpr uint8_t PROTOCOL_VERSION=1;
constexpr size_t RADIO_PAYLOAD=200; // Fits legacy ESP-NOW's 250-byte limit.
constexpr uint16_t MAX_CELLS=300;
constexpr uint16_t MAX_GROUPS=64;
constexpr uint8_t MAX_PEAKS=3, BINS=18, MAX_ANCHORS=2;
constexpr uint32_t HELLO_MS=2000, HEALTH_MS=5000, SNAP_TIMEOUT_MS=250;
constexpr uint8_t BROADCAST_MAC[6]={255,255,255,255,255,255};

enum MessageType : uint8_t {
  HELLO=1, CONFIG_BEGIN=2, CONFIG_CELL=3, CONFIG_COMMIT=4,
  CAPTURE_BASELINE=5, SNAPSHOT_REQUEST=6, ACK=7,
  STATE=8, HEALTH=9, SNAPSHOT_BEGIN=10, SNAPSHOT_CHUNK=11,
  SNAPSHOT_END=12, SNAPSHOT_ACK=13, DIAGNOSTIC=14, DIAGNOSTIC_ACK=15,
  HEALTH_ACK=16
};
enum AckStatus : uint8_t { ACK_OK=0, ACK_BAD_PAYLOAD=1, ACK_BAD_ORDER=2, ACK_NO_MEMORY=3, ACK_CAMERA_ERROR=4, ACK_BUSY=5 };
enum CellState : uint8_t { UNKNOWN=0, CLEAR=1, OCCUPIED=2 };
enum ResolutionId : uint8_t { QVGA=0, VGA=1, SVGA=2, XGA=3 };

struct __attribute__((packed)) Packet {
  uint16_t magic;
  uint8_t version, type;
  uint32_t seq;
  uint16_t length;
  uint8_t payload[RADIO_PAYLOAD];
};
static_assert(sizeof(Packet)==210, "Legacy ESP-NOW packet size");
struct __attribute__((packed)) CameraSettings {
  uint8_t resolution;
  int8_t brightness, contrast, saturation;
  uint8_t vflip, hmirror;
};
struct __attribute__((packed)) ConfigBeginPayload {
  uint32_t revision;
  uint16_t count;
  CameraSettings camera;
};
struct __attribute__((packed)) CellConfig {
  uint32_t id, groupId; // groupId=0 means an independent sensor.
  uint16_t x,y;
  uint8_t radius, shape; // shape: 0 circle, 1 square
  uint16_t contrastFloor, thresholdPermille;
  uint8_t angleTolerance, enterFrames, clearFrames;
};
struct __attribute__((packed)) ConfigCellPayload {
  uint16_t index;
  CellConfig cell;
};
struct __attribute__((packed)) AckPayload { uint8_t forType,status; uint16_t detail; };
struct __attribute__((packed)) StatePayload {
  uint32_t id, revision, frame;
  uint16_t scorePermille;
  uint8_t state, grouped;
};
struct __attribute__((packed)) HelloPayload {
  uint8_t mac[6];
  uint32_t revision;
  uint16_t width,height,count;
  uint8_t channel, baselineReady;
};
struct __attribute__((packed)) SnapshotBeginPayload {
  uint32_t frame, bytes, crc;
  uint16_t width,height;
};
struct __attribute__((packed)) CompressedSnapshotBeginPayload {
  SnapshotBeginPayload raw;
  uint32_t wireBytes,wireCrc;
  uint8_t codec;
};
static_assert(sizeof(SnapshotBeginPayload)==16 && sizeof(CompressedSnapshotBeginPayload)==25,"Snapshot header layout");
struct __attribute__((packed)) SnapshotChunkPrefix { uint32_t offset; };
struct __attribute__((packed)) SnapshotEndPayload { uint32_t frame, crc; };
enum DiagnosticEvent : uint8_t {
  DIAG_BOOT=1, DIAG_BRIDGE_JOIN=2, DIAG_BRIDGE_REJOIN=3,
  DIAG_SNAPSHOT_START=4, DIAG_SNAPSHOT_DONE=5,
  DIAG_SNAPSHOT_SEND_FAILED=6, DIAG_SNAPSHOT_ABORTED=7,
  DIAG_BASELINE_START=8, DIAG_BASELINE_DONE=9, DIAG_BASELINE_FAILED=10,
  DIAG_READY=11, DIAG_INIT_FAILED=12,
  DIAG_CONFIG_BEGIN=13, DIAG_CONFIG_ACK_FAILED=14
};
struct __attribute__((packed)) DiagnosticPayload {
  uint32_t bootId,eventSeq,uptimeMs,frame,detail,offset;
  uint16_t captureFailures,dropped;
  uint8_t event,resetReason,channel,operation,outcome;
};
static_assert(sizeof(ConfigCellPayload)<=RADIO_PAYLOAD, "Cell message too large");

struct Resolution { framesize_t frameSize; uint16_t width,height; };
const Resolution RESOLUTIONS[]={
  {FRAMESIZE_QVGA,320,240},{FRAMESIZE_VGA,640,480},
  {FRAMESIZE_SVGA,800,600},{FRAMESIZE_XGA,1024,768}
};
CameraSettings cameraSettings={QVGA,0,0,0,0,0};
uint16_t frameWidth=320,frameHeight=240;
uint8_t *framePixels=nullptr;
bool cameraReady=false, baselineReady=false, configured=false;
bool storageReady=false;
uint8_t baselineStorageError=0;
uint32_t frameNumber=0, configRevision=0, captureFailures=0;
uint32_t lastHello=0,lastHealth=0,lastCapture=0,nextSeq=1;
uint8_t localMac[6]={},bridgeMac[6]={};
bool bridgeKnown=false;
uint8_t radioChannel=1;
uint32_t lastBridgeBeacon=0,lastChannelScan=0;

struct Anchor { uint16_t x,y; };
struct Feature {
  uint16_t samples=0,edges=0,maxGradient=0,medianGradient=0,upperGradient=0;
  uint16_t whitePixels=0;
  uint32_t graySum=0;
  uint16_t hist[BINS]={};
  float angleSum[BINS]={};
  float peakAngle[MAX_PEAKS]={}, peakShare[MAX_PEAKS]={};
  uint8_t peakCount=0;
  bool textured=false;
};
struct CellRuntime {
  CellConfig config={};
  Feature reference={};
  Anchor anchors[MAX_ANCHORS]={};
  uint8_t anchorCount=0;
  uint8_t state=UNKNOWN,enterCount=0,clearCount=0;
  uint16_t scorePermille=0;
  int16_t directionX[MAX_PEAKS]={},directionY[MAX_PEAKS]={};
  uint16_t referenceBuckets[MAX_PEAKS+1]={};
};
struct GroupRuntime {
  uint32_t id=0;
  uint8_t state=UNKNOWN;
};
CellRuntime *cells=nullptr,*staging=nullptr;
GroupRuntime groups[MAX_GROUPS]={};
uint16_t cellCount=0,stagingCount=0,expectedCount=0,groupCount=0;
uint32_t stagingRevision=0;
CameraSettings stagingSettings={QVGA,0,0,0,0,0};
bool stagingOpen=false;

struct Received { uint8_t mac[6]; Packet packet; };
QueueHandle_t inbox=nullptr;
struct SnapshotTransfer {
  bool active=false,waiting=false;
  uint8_t type=0,retries=0;
  uint16_t chunkLength=0;
  uint32_t offset=0,seq=0,requestSeq=0,sentAt=0,crc=0,wireCrc=0,bytes=0;
  uint8_t codec=0;
  uint8_t *encoded=nullptr;
} snapshot;
struct RtcDiagnostic { uint32_t magic;uint8_t operation,outcome;uint32_t offset; };
RTC_DATA_ATTR RtcDiagnostic rtcDiagnostic;
constexpr uint32_t RTC_DIAG_MAGIC=0x52444941;
DiagnosticPayload diagnosticQueue[16]={};
uint8_t diagnosticHead=0,diagnosticCount=0;
uint16_t diagnosticDropped=0;
uint32_t diagnosticBootId=0,diagnosticSequence=1,lastDiagnosticSend=0;
uint8_t diagnosticResetReason=0;
struct __attribute__((packed)) BaselineHeader {
  uint32_t magic,revision,bytes,cellsCrc,imageCrc;
  uint16_t count;
  CameraSettings settings;
};
bool saveBaseline();
void loadBaseline();

void macText(const uint8_t *mac,char *out) {
  sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
bool sameMac(const uint8_t *a,const uint8_t *b) { return memcmp(a,b,6)==0; }
bool addPeer(const uint8_t *mac) {
  if(esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer={};
  memcpy(peer.peer_addr,mac,6);
  peer.channel=radioChannel; peer.ifidx=WIFI_IF_STA; peer.encrypt=false;
  return esp_now_add_peer(&peer)==ESP_OK;
}
bool transmit(const uint8_t *dest,uint8_t type,uint32_t seq,const void *data,size_t length) {
  if(length>RADIO_PAYLOAD || !addPeer(dest)) return false;
  Packet packet={}; packet.magic=MAGIC;packet.version=PROTOCOL_VERSION;
  packet.type=type;packet.seq=seq;packet.length=(uint16_t)length;
  if(length) memcpy(packet.payload,data,length);
  return esp_now_send(dest,(uint8_t *)&packet,offsetof(Packet,payload)+length)==ESP_OK;
}
bool sendAck(const uint8_t *dest,uint32_t seq,uint8_t type,uint8_t status,uint16_t detail=0) {
  AckPayload payload={type,status,detail};
  return transmit(dest,ACK,seq,&payload,sizeof(payload));
}
void sendHello() {
  HelloPayload payload={}; memcpy(payload.mac,localMac,6);
  payload.revision=configRevision;payload.width=frameWidth;payload.height=frameHeight;
  payload.count=cellCount;payload.channel=radioChannel;payload.baselineReady=baselineReady;
  transmit(BROADCAST_MAC,HELLO,nextSeq++,&payload,sizeof(payload));
}
void sendHealth() {
  if(!bridgeKnown) return;
  struct __attribute__((packed)) HealthPayload {
    uint32_t revision,frame,failures,freeHeap;
    uint16_t cells,width,height,captureAgeMs;
    uint8_t baselineReady,snapshotActive;
  } payload={configRevision,frameNumber,captureFailures,ESP.getFreeHeap(),cellCount,
             frameWidth,frameHeight,(uint16_t)min((uint32_t)65535,millis()-lastCapture),
             (uint8_t)baselineReady,(uint8_t)snapshot.active};
  transmit(bridgeMac,HEALTH,nextSeq++,&payload,sizeof(payload));
}
void queueDiagnostic(uint8_t event,uint32_t detail=0,uint32_t offset=0) {
  if(diagnosticCount==16) { diagnosticHead=(diagnosticHead+1)%16;--diagnosticCount;++diagnosticDropped; }
  DiagnosticPayload &d=diagnosticQueue[(diagnosticHead+diagnosticCount)%16];
  d={diagnosticBootId,diagnosticSequence++,millis(),frameNumber,detail,offset,
     (uint16_t)min(captureFailures,(uint32_t)65535),diagnosticDropped,
     event,diagnosticResetReason,radioChannel,rtcDiagnostic.operation,rtcDiagnostic.outcome};
  ++diagnosticCount;
}
void serviceDiagnostics() {
  if(!bridgeKnown || snapshot.active || !diagnosticCount || millis()-lastDiagnosticSend<500) return;
  const DiagnosticPayload &d=diagnosticQueue[diagnosticHead];
  transmit(bridgeMac,DIAGNOSTIC,d.eventSeq,&d,sizeof(d));
  lastDiagnosticSend=millis();
}

void onReceive(const esp_now_recv_info_t *info,const uint8_t *data,int length) {
  if(!info || length<(int)offsetof(Packet,payload) || !inbox) return;
  Received item={}; memcpy(item.mac,info->src_addr,6);
  const size_t bytes=min((size_t)length,sizeof(Packet));
  memcpy(&item.packet,data,bytes);
  if(item.packet.magic!=MAGIC || item.packet.version!=PROTOCOL_VERSION ||
     item.packet.length>RADIO_PAYLOAD ||
     length!=(int)(offsetof(Packet,payload)+item.packet.length)) return;
  xQueueSend(inbox,&item,0); // Sender retries if the queue is full.
}

void flashReady() {
  for(int i=0;i<2;++i) { digitalWrite(STATUS_LED,HIGH);delay(120);digitalWrite(STATUS_LED,LOW);delay(120); }
}
bool initCamera(const CameraSettings &settings) {
  if(settings.resolution>XGA) return false;
  if(cameraReady) { esp_camera_deinit();cameraReady=false; }
  free(framePixels);framePixels=nullptr;
  const Resolution &r=RESOLUTIONS[settings.resolution];
  frameWidth=r.width;frameHeight=r.height;
  framePixels=(uint8_t *)ps_malloc((size_t)r.width*r.height);
  if(!framePixels) return false;
  camera_config_t c={};
  c.pin_pwdn=PWDN;c.pin_reset=RESET;c.pin_xclk=XCLK;c.pin_sccb_sda=SIOD;c.pin_sccb_scl=SIOC;
  c.pin_d0=D0;c.pin_d1=D1;c.pin_d2=D2;c.pin_d3=D3;c.pin_d4=D4;c.pin_d5=D5;c.pin_d6=D6;c.pin_d7=D7;
  c.pin_vsync=VSYNC;c.pin_href=HREF;c.pin_pclk=PCLK;c.xclk_freq_hz=20000000;
  c.ledc_timer=LEDC_TIMER_0;c.ledc_channel=LEDC_CHANNEL_0;
  c.pixel_format=PIXFORMAT_GRAYSCALE;c.frame_size=r.frameSize;
  c.fb_location=CAMERA_FB_IN_PSRAM;c.fb_count=1;c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
  if(esp_camera_init(&c)!=ESP_OK) { free(framePixels);framePixels=nullptr;return false; }
  sensor_t *sensor=esp_camera_sensor_get();
  if(sensor) {
    sensor->set_brightness(sensor,settings.brightness);
    sensor->set_contrast(sensor,settings.contrast);
    sensor->set_saturation(sensor,settings.saturation);
    sensor->set_vflip(sensor,settings.vflip);
    sensor->set_hmirror(sensor,settings.hmirror);
  }
  cameraSettings=settings;cameraReady=true;
  DEBUGF("camera %ux%u grayscale ready\n",frameWidth,frameHeight);
  return true;
}
bool captureFrame() {
  if(!cameraReady || !framePixels) return false;
  lastCapture=millis();
  camera_fb_t *fb=esp_camera_fb_get();
  const size_t bytes=(size_t)frameWidth*frameHeight;
  if(!fb) { ++captureFailures;return false; }
  const bool valid=fb->format==PIXFORMAT_GRAYSCALE && fb->width==frameWidth &&
    fb->height==frameHeight && fb->len>=bytes;
  if(valid) memcpy(framePixels,fb->buf,bytes);
  esp_camera_fb_return(fb);
  if(!valid) { ++captureFailures;return false; }
  ++frameNumber;return true;
}

constexpr uint16_t STATE_QUEUE_CAPACITY=400;
StatePayload *stateQueue=nullptr;
uint16_t stateHead=0,stateCount=0;
uint32_t lastStateSend=0;
void queueState(uint32_t id,bool grouped,uint8_t state,uint16_t score) {
  if(!stateQueue) return;
  StatePayload value={id,configRevision,frameNumber,score,state,(uint8_t)grouped};
  for(uint16_t i=0;i<stateCount;++i) {
    StatePayload &slot=stateQueue[(stateHead+i)%STATE_QUEUE_CAPACITY];
    if(slot.id==id && slot.grouped==grouped) { slot=value;return; }
  }
  if(stateCount==STATE_QUEUE_CAPACITY) {
    DEBUGF("state queue full; state %lu will be recovered by refresh\n",(unsigned long)id);
    return;
  }
  stateQueue[(stateHead+stateCount)%STATE_QUEUE_CAPACITY]=value;
  ++stateCount;
}
void flushState() {
  if(!bridgeKnown || !stateCount || millis()-lastStateSend<20) return;
  lastStateSend=millis();
  StatePayload &value=stateQueue[stateHead];
  if(transmit(bridgeMac,STATE,nextSeq++,&value,sizeof(value))) {
    DEBUGF("state %s %lu=%s score=%u frame=%lu\n",value.grouped?"block":"sensor",
      (unsigned long)value.id,value.state==OCCUPIED?"occupied":value.state==CLEAR?"clear":"unknown",
      value.scorePermille,(unsigned long)value.frame);
    stateHead=(stateHead+1)%STATE_QUEUE_CAPACITY;--stateCount;
  }
}
void queueAllStates() {
  for(uint16_t i=0;i<cellCount;++i)
    queueState(cells[i].config.id,false,cells[i].state,cells[i].scorePermille);
  for(uint16_t g=0;g<groupCount;++g) {
    uint16_t score=0;
    for(uint16_t i=0;i<cellCount;++i)
      if(cells[i].config.groupId==groups[g].id) score=max(score,cells[i].scorePermille);
    queueState(groups[g].id,true,groups[g].state,score);
  }
}

void gradientAt(const uint8_t *pixels,int p,int &gx,int &gy) {
  gx=-pixels[p-frameWidth-1]+pixels[p-frameWidth+1]
     -2*pixels[p-1]+2*pixels[p+1]
     -pixels[p+frameWidth-1]+pixels[p+frameWidth+1];
  gy=-pixels[p-frameWidth-1]-2*pixels[p-frameWidth]-pixels[p-frameWidth+1]
     +pixels[p+frameWidth-1]+2*pixels[p+frameWidth]+pixels[p+frameWidth+1];
}
bool insideCell(const CellConfig &c,int x,int y) {
  if(x<=0 || x>=frameWidth-1 || y<=0 || y>=frameHeight-1) return false;
  const int dx=x-(int)c.x,dy=y-(int)c.y;
  return c.shape==0 ? dx*dx+dy*dy<=c.radius*c.radius
                    : abs(dx)<c.radius && abs(dy)<c.radius;
}
void bounds(const CellConfig &c,int &x0,int &x1,int &y0,int &y1) {
  x0=max(1,(int)c.x-c.radius);x1=min((int)frameWidth-2,(int)c.x+c.radius);
  y0=max(1,(int)c.y-c.radius);y1=min((int)frameHeight-2,(int)c.y+c.radius);
}
uint16_t tanQ8[21]={};
bool nearDirection(const CellRuntime &cell,int peak,int gx,int gy,int tolerance) {
  const int32_t dot=abs(gx*cell.directionX[peak]+gy*cell.directionY[peak]);
  const int32_t cross=abs(gy*cell.directionX[peak]-gx*cell.directionY[peak]);
  return cross*256<=dot*tanQ8[constrain(tolerance,0,20)];
}
uint8_t directionBucket(const CellRuntime &cell,int gx,int gy) {
  int32_t bestDot=-1;uint8_t best=MAX_PEAKS;
  for(uint8_t peak=0;peak<cell.reference.peakCount;++peak) {
    if(!nearDirection(cell,peak,gx,gy,cell.config.angleTolerance)) continue;
    const int32_t dot=abs(gx*cell.directionX[peak]+gy*cell.directionY[peak]);
    if(dot>bestDot) { bestDot=dot;best=peak; }
  }
  return best;
}
void selectPeaks(Feature &f) {
  uint32_t support[BINS]={};
  for(int i=0;i<BINS;++i) support[i]=f.hist[(i+BINS-1)%BINS]+f.hist[i]+f.hist[(i+1)%BINS];
  uint32_t selected=0,primary=0;
  for(int peak=0;peak<MAX_PEAKS;++peak) {
    int best=-1;uint32_t count=0;
    for(int bin=0;bin<BINS;++bin) {
      bool close=false;
      for(int delta=-2;delta<=2;++delta)
        if(selected&(1UL<<((bin+delta+BINS)%BINS))) close=true;
      if(!close && support[bin]>count) { best=bin;count=support[bin]; }
    }
    if(best<0 || count<6) break;
    if(peak==0) { if(count*4<f.edges) break;primary=count; }
    else if(count*5<f.edges || count*2<primary) break;
    selected|=1UL<<best;
    const float centre=best*10.0f+5.0f;
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
    f.peakAngle[f.peakCount]=refined;
    f.peakShare[f.peakCount]=(float)count/f.edges;
    ++f.peakCount;
  }
}
Feature analyse(CellRuntime &cell,bool referenceMode,uint16_t buckets[MAX_PEAKS+1]) {
  Feature f={};
  uint16_t magnitudes[64]={};
  memset(buckets,0,(MAX_PEAKS+1)*sizeof(uint16_t));
  int x0,x1,y0,y1;bounds(cell.config,x0,x1,y0,y1);
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
    if(!insideCell(cell.config,x,y)) continue;
    const int p=y*frameWidth+x;
    int gx,gy;gradientAt(framePixels,p,gx,gy);
    const int magnitude=abs(gx)+abs(gy);
    ++f.samples;f.graySum+=framePixels[p];
    if(framePixels[p]>=250) ++f.whitePixels;
    f.maxGradient=max(f.maxGradient,(uint16_t)magnitude);
    ++magnitudes[min(magnitude>>4,63)];
  }
  uint32_t total=0;
  for(int i=0;i<64;++i) {
    total+=magnitudes[i];
    if(!f.medianGradient && total>=(f.samples+1)/2) f.medianGradient=i*16+8;
    if(total>=(f.samples*95+99)/100) { f.upperGradient=i*16+8;break; }
  }
  if(f.maxGradient<cell.config.contrastFloor) return f;
  // Use the saved reference cutoff for live frames. A changing bright pixel
  // must not move the cutoff for every edge in an otherwise unchanged cell.
  const int minimum=max((int)cell.config.contrastFloor,
    (int)(referenceMode?f.maxGradient:cell.reference.maxGradient)/4);
  for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
    if(!insideCell(cell.config,x,y)) continue;
    int gx,gy;gradientAt(framePixels,y*frameWidth+x,gx,gy);
    if(abs(gx)+abs(gy)<minimum) continue;
    ++f.edges;
    if(!referenceMode && cell.reference.peakCount) {
      ++buckets[directionBucket(cell,gx,gy)];
    } else {
      float angle=atan2f((float)gy,(float)gx)*57.2957795f;
      if(angle<0) angle+=180;
      if(angle>=180) angle-=180;
      const int bin=min((int)(angle/10),BINS-1);
      ++f.hist[bin];f.angleSum[bin]+=angle;
    }
  }
  f.textured=f.edges>=8;
  if(referenceMode && f.textured) selectPeaks(f);
  return f;
}
float histogramDistance(const Feature &a,const Feature &b) {
  if(!a.edges || !b.edges) return 1;
  float sum=0;
  for(int i=0;i<BINS;++i) {
    const int am=a.hist[(i+BINS-1)%BINS]+2*a.hist[i]+a.hist[(i+1)%BINS];
    const int bm=b.hist[(i+BINS-1)%BINS]+2*b.hist[i]+b.hist[(i+1)%BINS];
    sum+=fabsf((float)am/(4*a.edges)-(float)bm/(4*b.edges));
  }
  return min(1.0f,0.5f*sum);
}
float projectionDistance(const CellRuntime &cell,const Feature &live,const uint16_t *buckets) {
  if(!cell.reference.edges || !live.edges) return 1;
  float sum=0;
  for(int i=0;i<=MAX_PEAKS;++i)
    sum+=fabsf((float)cell.referenceBuckets[i]/cell.reference.edges
             -(float)buckets[i]/live.edges);
  return min(1.0f,0.5f*sum);
}
void captureAnchors(CellRuntime &cell) {
  cell.anchorCount=0;
  if(!cell.reference.peakCount) return;
  const int minMagnitude=max((int)cell.config.contrastFloor,(int)cell.reference.maxGradient/4);
  const int separation=max(5,(int)cell.config.radius/2);
  int x0,x1,y0,y1;bounds(cell.config,x0,x1,y0,y1);
  for(int n=0;n<MAX_ANCHORS;++n) {
    int best=-1;Anchor point={};
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      if(!insideCell(cell.config,x,y)) continue;
      if(n && (x-cell.anchors[0].x)*(x-cell.anchors[0].x)
             +(y-cell.anchors[0].y)*(y-cell.anchors[0].y)<separation*separation) continue;
      int gx,gy;gradientAt(framePixels,y*frameWidth+x,gx,gy);
      const int magnitude=abs(gx)+abs(gy);
      if(magnitude>=minMagnitude && magnitude>best &&
         nearDirection(cell,0,gx,gy,max(3,(int)cell.config.angleTolerance))) {
        best=magnitude;point={(uint16_t)x,(uint16_t)y};
      }
    }
    if(best<0) break;
    cell.anchors[cell.anchorCount++]=point;
  }
}
float anchorDistance(const CellRuntime &cell,const Feature &live) {
  if(!cell.anchorCount || !live.textured) return 0;
  int found=0;
  const int minimum=max((int)cell.config.contrastFloor,(int)cell.reference.maxGradient/4);
  for(int n=0;n<cell.anchorCount;++n) {
    bool matched=false;
    for(int dy=-2;dy<=2 && !matched;++dy) for(int dx=-2;dx<=2;++dx) {
      const int x=cell.anchors[n].x+dx,y=cell.anchors[n].y+dy;
      if(!insideCell(cell.config,x,y)) continue;
      int gx,gy;gradientAt(framePixels,y*frameWidth+x,gx,gy);
      if(abs(gx)+abs(gy)>=minimum &&
         nearDirection(cell,0,gx,gy,max(3,(int)cell.config.angleTolerance))) {
        matched=true;break;
      }
    }
    if(matched) ++found;
  }
  return (float)(cell.anchorCount-found)/cell.anchorCount;
}
void calibrateCell(CellRuntime &cell) {
  uint16_t unused[MAX_PEAKS+1]={};
  cell.reference=analyse(cell,true,unused);
  memset(cell.referenceBuckets,0,sizeof(cell.referenceBuckets));
  cell.anchorCount=0;
  for(int peak=0;peak<cell.reference.peakCount;++peak) {
    const float radians=cell.reference.peakAngle[peak]*0.01745329252f;
    cell.directionX[peak]=(int16_t)lroundf(cosf(radians)*256);
    cell.directionY[peak]=(int16_t)lroundf(sinf(radians)*256);
  }
  // Calculate baseline buckets with the same projected assignment as live frames.
  if(cell.reference.peakCount) {
    int x0,x1,y0,y1;bounds(cell.config,x0,x1,y0,y1);
    const int minimum=max((int)cell.config.contrastFloor,(int)cell.reference.maxGradient/4);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
      if(!insideCell(cell.config,x,y)) continue;
      int gx,gy;gradientAt(framePixels,y*frameWidth+x,gx,gy);
      if(abs(gx)+abs(gy)>=minimum) ++cell.referenceBuckets[directionBucket(cell,gx,gy)];
    }
    captureAnchors(cell);
  }
  cell.state=CLEAR;cell.enterCount=cell.clearCount=0;cell.scorePermille=0;
  DEBUGF("base id=%lu group=%lu centre=(%u,%u) r=%u edges=%u angles=%u",
    (unsigned long)cell.config.id,(unsigned long)cell.config.groupId,
    cell.config.x,cell.config.y,cell.config.radius,cell.reference.edges,cell.reference.peakCount);
#if CAMERA_DEBUG_SERIAL
  for(int i=0;i<cell.reference.peakCount;++i) DEBUGF(" %.1fdeg",cell.reference.peakAngle[i]);
  DEBUGF(" anchors=%u\n",cell.anchorCount);
#endif
}
void baselineHeartbeat() {
  if(millis()-lastHello>=1000) { sendHello();lastHello=millis();delay(2); }
  if(millis()-lastHealth>=5000) { sendHealth();lastHealth=millis();delay(2); }
}
bool captureBaseline() {
  const uint32_t started=millis();
  rtcDiagnostic={RTC_DIAG_MAGIC,2,0,0};
  queueDiagnostic(DIAG_BASELINE_START,configRevision);
  DEBUGF("baseline capture start revision=%lu frame=%lu\n",(unsigned long)configRevision,(unsigned long)frameNumber);
  sendHello();lastHello=millis();
  if(snapshot.active || !captureFrame()) { rtcDiagnostic.outcome=1;queueDiagnostic(DIAG_BASELINE_FAILED,1);rtcDiagnostic.operation=0;return false; }
  for(uint16_t i=0;i<cellCount;++i) { calibrateCell(cells[i]);baselineHeartbeat(); }
  if(!saveBaseline()) {
    const uint32_t freeBytes=storageReady?LittleFS.totalBytes()-LittleFS.usedBytes():0;
    DEBUGF("baseline flash write failed stage=%u free_bytes=%lu\n",baselineStorageError,(unsigned long)freeBytes);
    rtcDiagnostic.outcome=2;queueDiagnostic(DIAG_BASELINE_FAILED,200+baselineStorageError,freeBytes);rtcDiagnostic.operation=0;return false;
  }
  baselineReady=true;
  for(uint16_t i=0;i<groupCount;++i) groups[i].state=CLEAR;
  queueAllStates();
  flashReady();
  sendHello();lastHello=millis();
  DEBUGF("baseline ready: %u cells frame=%lu duration_ms=%lu\n",cellCount,(unsigned long)frameNumber,(unsigned long)(millis()-started));
  rtcDiagnostic.outcome=3;queueDiagnostic(DIAG_BASELINE_DONE,millis()-started);rtcDiagnostic.operation=0;
  return true;
}

bool exposureUnreliable(const CellRuntime &cell,const Feature &live) {
  if(!live.samples || !cell.reference.samples) return false;
  const float mean=(float)live.graySum/live.samples;
  const float referenceMean=(float)cell.reference.graySum/cell.reference.samples;
  const float white=(float)live.whitePixels/live.samples;
  const float referenceWhite=(float)cell.reference.whitePixels/cell.reference.samples;
  const bool clipped=white>=0.40f && white>=referenceWhite+0.25f;
  const bool lostDetail=cell.reference.textured && cell.reference.edges>=16 && !live.textured &&
    live.maxGradient*2<cell.reference.maxGradient;
  return mean>=referenceMean+25 && (clipped || lostDetail);
}
uint16_t compareCell(CellRuntime &cell,const Feature &live,const uint16_t *buckets) {
  // Angle histograms from fewer than 16 edges are too sparse to compare reliably.
  // Treat a weak, unoriented reference as clear until the live patch has real detail.
  if(!cell.reference.peakCount && cell.reference.edges<16 && live.edges<16) return 0;
  float score=0;
  if(!cell.reference.textured && !live.textured) score=0;
  else if(cell.reference.textured!=live.textured) score=1;
  else if(cell.reference.peakCount) score=projectionDistance(cell,live,buckets);
  else score=histogramDistance(cell.reference,live);
  if(cell.reference.textured && live.textured && cell.reference.samples>=64 &&
     live.samples>=64 && (!cell.reference.peakCount || cell.reference.peakShare[0]<0.40f) &&
     live.upperGradient>cell.reference.upperGradient+40) {
    const float refTail=(float)(cell.reference.upperGradient+8)/(cell.reference.medianGradient+8);
    const float liveTail=(float)(live.upperGradient+8)/(live.medianGradient+8);
    if(liveTail>refTail) score=max(score,1.0f-refTail/liveTail);
  }
  if(cell.anchorCount && live.textured) score=max(score,anchorDistance(cell,live));
  return (uint16_t)constrain((int)lroundf(score*1000),0,1000);
}
void updateGroups() {
  for(uint16_t g=0;g<groupCount;++g) {
    bool occupied=false,unknown=false;
    uint16_t score=0;
    for(uint16_t i=0;i<cellCount;++i) if(cells[i].config.groupId==groups[g].id) {
      if(cells[i].state==OCCUPIED) occupied=true;
      if(cells[i].state==UNKNOWN) unknown=true;
      score=max(score,cells[i].scorePermille);
    }
    const uint8_t next=occupied?OCCUPIED:unknown?UNKNOWN:CLEAR;
    if(next!=groups[g].state) {
      groups[g].state=next;
      queueState(groups[g].id,true,next,score);
    }
  }
}
void analyseAllCells() {
  if(!baselineReady) return;
  for(uint16_t i=0;i<cellCount;++i) {
    CellRuntime &cell=cells[i];
    uint16_t buckets[MAX_PEAKS+1]={};
    const Feature live=analyse(cell,false,buckets);
    if(exposureUnreliable(cell,live)) {
      // Preserve occupancy while image evidence is unavailable.
      cell.enterCount=cell.clearCount=0;
      DEBUGF("unreliable id=%lu overexposure\n",(unsigned long)cell.config.id);
      continue;
    }
    cell.scorePermille=compareCell(cell,live,buckets);
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
      const uint16_t directionScore=cell.reference.peakCount && live.textured?
        (uint16_t)lroundf(projectionDistance(cell,live,buckets)*1000):0;
      const uint16_t anchorScore=cell.anchorCount && live.textured?
        (uint16_t)lroundf(anchorDistance(cell,live)*1000):0;
      DEBUGF("trigger id=%lu group=%lu %s score=%u direction=%u anchor=%u angles=%u anchors=%u edges=%u/%u max=%u/%u\n",
        (unsigned long)cell.config.id,(unsigned long)cell.config.groupId,
        cell.state==OCCUPIED?"occupied":"clear",cell.scorePermille,directionScore,anchorScore,
        cell.reference.peakCount,cell.anchorCount,live.edges,cell.reference.edges,
        live.maxGradient,cell.reference.maxGradient);
      queueState(cell.config.id,false,cell.state,cell.scorePermille);
    }
  }
  updateGroups();
}

bool validSettings(const CameraSettings &s) {
  return s.resolution<=XGA && s.brightness>=-2 && s.brightness<=2 &&
    s.contrast>=-2 && s.contrast<=2 && s.saturation>=-2 && s.saturation<=2 &&
    s.vflip<=1 && s.hmirror<=1;
}
bool validCell(const CellConfig &c,const CameraSettings &settings) {
  const Resolution &r=RESOLUTIONS[settings.resolution];
  return c.id!=0 && c.x<r.width && c.y<r.height && c.radius>=3 && c.radius<=50 &&
    c.shape<=1 && c.contrastFloor>=10 && c.contrastFloor<=500 &&
    c.thresholdPermille>=50 && c.thresholdPermille<=1000 &&
    c.angleTolerance<=20 && c.enterFrames>=1 && c.clearFrames>=1;
}
uint8_t beginConfig(const ConfigBeginPayload &value) {
  if(snapshot.active) return ACK_BUSY;
  if(value.count>MAX_CELLS || !validSettings(value.camera)) return ACK_BAD_PAYLOAD;
  stagingOpen=true;stagingCount=0;expectedCount=value.count;
  stagingRevision=value.revision;stagingSettings=value.camera;
  DEBUGF("config begin revision=%lu cells=%u resolution=%u\n",
    (unsigned long)stagingRevision,expectedCount,stagingSettings.resolution);
  return ACK_OK;
}
uint8_t stageCell(const ConfigCellPayload &value) {
  if(!stagingOpen) return ACK_BAD_ORDER;
  if(value.index<stagingCount) {
    const CellConfig &existing=staging[value.index].config;
    return memcmp(&existing,&value.cell,sizeof(existing))==0?ACK_OK:ACK_BAD_ORDER;
  }
  if(value.index!=stagingCount || stagingCount>=expectedCount ||
     !validCell(value.cell,stagingSettings)) return ACK_BAD_PAYLOAD;
  for(uint16_t i=0;i<stagingCount;++i)
    if(staging[i].config.id==value.cell.id) return ACK_BAD_PAYLOAD;
  staging[stagingCount]=CellRuntime();
  staging[stagingCount].config=value.cell;
  ++stagingCount;
  DEBUGF("config cell %u id=%lu group=%lu (%u,%u) r=%u shape=%u threshold=%u\n",
    value.index,(unsigned long)value.cell.id,(unsigned long)value.cell.groupId,
    value.cell.x,value.cell.y,value.cell.radius,value.cell.shape,value.cell.thresholdPermille);
  return ACK_OK;
}
uint8_t commitConfig(uint32_t revision) {
  if(!stagingOpen && configured && revision==configRevision) return ACK_OK;
  if(!stagingOpen || revision!=stagingRevision || stagingCount!=expectedCount) return ACK_BAD_ORDER;
  uint32_t groupIds[MAX_GROUPS]={};uint16_t found=0;
  for(uint16_t i=0;i<stagingCount;++i) {
    const uint32_t id=staging[i].config.groupId;
    if(!id) continue;
    bool known=false;
    for(uint16_t g=0;g<found;++g) if(groupIds[g]==id) known=true;
    if(!known) {
      if(found==MAX_GROUPS) return ACK_BAD_PAYLOAD;
      groupIds[found++]=id;
    }
  }
  // A change in optics or geometry invalidates the compact baseline.
  if(memcmp(&stagingSettings,&cameraSettings,sizeof(CameraSettings))!=0 || !cameraReady) {
    const CameraSettings previous=cameraSettings;
    if(!initCamera(stagingSettings)) {
      initCamera(previous);
      return ACK_CAMERA_ERROR;
    }
  }
  CellRuntime *old=cells;cells=staging;staging=old;
  cellCount=stagingCount;configRevision=stagingRevision;
  groupCount=found;
  for(uint16_t g=0;g<groupCount;++g) { groups[g].id=groupIds[g];groups[g].state=UNKNOWN; }
  baselineReady=false;configured=true;stagingOpen=false;
  stateHead=stateCount=0;
  queueAllStates();
  DEBUGF("config committed revision=%lu cells=%u groups=%u; capture baseline next\n",
    (unsigned long)configRevision,cellCount,groupCount);
  return ACK_OK;
}

uint32_t crc32(const uint8_t *data,size_t length) {
  uint32_t crc=0xFFFFFFFFUL;
  for(size_t i=0;i<length;++i) {
    crc^=data[i];
    for(int bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0);
  }
  return ~crc;
}
bool saveBaseline() {
  baselineStorageError=0;
  if(!storageReady) { baselineStorageError=1;return false; }
  const size_t cellsBytes=(size_t)cellCount*sizeof(CellRuntime);
  // Runtime features contain the calibration. The full grayscale frame is not
  // needed after reboot and made SVGA baselines require two 480 KB flash files.
  BaselineHeader h={0x52424C31,configRevision,0,
    crc32((const uint8_t *)cells,cellsBytes),0,cellCount,cameraSettings};
  if(LittleFS.exists("/baseline.bin")) LittleFS.remove("/baseline.bak");
  File f=LittleFS.open("/baseline.tmp","w");if(!f) { baselineStorageError=2;return false; }
  bool ok=f.write((const uint8_t *)&h,sizeof(h))==sizeof(h) &&
    f.write((const uint8_t *)cells,cellsBytes)==cellsBytes;
  f.close();if(!ok) { baselineStorageError=3;LittleFS.remove("/baseline.tmp");return false; }
  if(LittleFS.exists("/baseline.bin") && !LittleFS.rename("/baseline.bin","/baseline.bak")) { baselineStorageError=4;LittleFS.remove("/baseline.tmp");return false; }
  if(!LittleFS.rename("/baseline.tmp","/baseline.bin")) {
    baselineStorageError=5;
    if(LittleFS.exists("/baseline.bak")) LittleFS.rename("/baseline.bak","/baseline.bin");
    return false;
  }
  LittleFS.remove("/baseline.bak");return true;
}
void loadBaseline() {
  if(!storageReady) return;
  if(!LittleFS.exists("/baseline.bin") && LittleFS.exists("/baseline.bak"))
    LittleFS.rename("/baseline.bak","/baseline.bin");
  File f=LittleFS.open("/baseline.bin","r");if(!f) return;
  BaselineHeader h={};
  if(f.read((uint8_t *)&h,sizeof(h))!=sizeof(h) || h.magic!=0x52424C31 ||
     h.count>MAX_CELLS || h.settings.resolution>XGA) { f.close();return; }
  const size_t cellsBytes=(size_t)h.count*sizeof(CellRuntime);
  if(f.size()!=sizeof(h)+cellsBytes+h.bytes || !initCamera(h.settings) ||
     (h.bytes && h.bytes!=(uint32_t)frameWidth*frameHeight)) { f.close();return; }
  bool ok=f.read((uint8_t *)cells,cellsBytes)==cellsBytes &&
    crc32((const uint8_t *)cells,cellsBytes)==h.cellsCrc;
  if(ok && h.bytes) ok=f.read(framePixels,h.bytes)==h.bytes && crc32(framePixels,h.bytes)==h.imageCrc;
  f.close();if(!ok) return;
  uint16_t found=0;
  for(uint16_t i=0;i<h.count;++i) {
    cells[i].state=UNKNOWN;cells[i].enterCount=cells[i].clearCount=0;
    uint32_t id=cells[i].config.groupId;if(!id) continue;
    bool known=false;for(uint16_t j=0;j<found;++j) if(groups[j].id==id) known=true;
    if(!known) { if(found>=MAX_GROUPS) return;groups[found++].id=id; }
  }
  groupCount=found;for(uint16_t i=0;i<groupCount;++i) groups[i].state=UNKNOWN;
  cellCount=h.count;configRevision=h.revision;configured=baselineReady=true;
  queueAllStates();
  DEBUGF("restored baseline revision=%lu cells=%u\n",(unsigned long)configRevision,cellCount);
}
void sendSnapshotPart() {
  if(!snapshot.active || !bridgeKnown) return;
  bool ok=false;
  const size_t bytes=snapshot.bytes;
  if(snapshot.type==SNAPSHOT_BEGIN) {
    SnapshotBeginPayload raw={frameNumber,(uint32_t)frameWidth*frameHeight,snapshot.crc,frameWidth,frameHeight};
    if(snapshot.codec) {
      CompressedSnapshotBeginPayload payload={raw,snapshot.bytes,snapshot.wireCrc,snapshot.codec};
      ok=transmit(bridgeMac,SNAPSHOT_BEGIN,snapshot.seq,&payload,sizeof(payload));
    } else ok=transmit(bridgeMac,SNAPSHOT_BEGIN,snapshot.seq,&raw,sizeof(raw));
  } else if(snapshot.type==SNAPSHOT_CHUNK) {
    uint8_t payload[sizeof(SnapshotChunkPrefix)+180];
    memcpy(payload,&snapshot.offset,sizeof(snapshot.offset));
    snapshot.chunkLength=min((size_t)180,bytes-(size_t)snapshot.offset);
    memcpy(payload+sizeof(SnapshotChunkPrefix),(snapshot.encoded?snapshot.encoded:framePixels)+snapshot.offset,snapshot.chunkLength);
    ok=transmit(bridgeMac,SNAPSHOT_CHUNK,snapshot.seq,payload,
                sizeof(SnapshotChunkPrefix)+snapshot.chunkLength);
  } else if(snapshot.type==SNAPSHOT_END) {
    SnapshotEndPayload payload={frameNumber,snapshot.crc};
    ok=transmit(bridgeMac,SNAPSHOT_END,snapshot.seq,&payload,sizeof(payload));
  }
  snapshot.waiting=ok;
  snapshot.sentAt=millis();
  if(!ok) {
    DEBUGF("snapshot send failed type=%u offset=%lu retry=%u\n",
      snapshot.type,(unsigned long)snapshot.offset,snapshot.retries);
    if(snapshot.retries==0) queueDiagnostic(DIAG_SNAPSHOT_SEND_FAILED,snapshot.type,snapshot.offset);
  }
}
bool startSnapshot(uint32_t requestSeq) {
  if(snapshot.active || !framePixels || !frameNumber) return false;
  snapshot=SnapshotTransfer();
  snapshot.active=true;snapshot.type=SNAPSHOT_BEGIN;snapshot.seq=nextSeq++;snapshot.requestSeq=requestSeq;
  const size_t rawBytes=(size_t)frameWidth*frameHeight;
  snapshot.crc=crc32(framePixels,rawBytes);
  snapshot.bytes=rawBytes;snapshot.wireCrc=snapshot.crc;
  const uint32_t compressStarted=millis();
  uint8_t *compressed=(uint8_t *)ps_malloc(rawBytes);
  tdefl_compressor *compressor=(tdefl_compressor *)ps_malloc(sizeof(tdefl_compressor));
  if(compressed && compressor) {
    size_t inputOffset=0,outputOffset=0;
    const tdefl_status started=tdefl_init(compressor,nullptr,nullptr,1|TDEFL_GREEDY_PARSING_FLAG);
    tdefl_status result=started;
    while(result==TDEFL_STATUS_OKAY && outputOffset<rawBytes) {
      size_t inputBytes=min((size_t)16384,rawBytes-inputOffset);
      size_t outputBytes=rawBytes-outputOffset;
      const tdefl_flush flush=inputOffset+inputBytes==rawBytes?TDEFL_FINISH:TDEFL_NO_FLUSH;
      result=tdefl_compress(compressor,framePixels+inputOffset,&inputBytes,
        compressed+outputOffset,&outputBytes,flush);
      inputOffset+=inputBytes;outputOffset+=outputBytes;
      baselineHeartbeat();delay(1);
      if(millis()-compressStarted>6000) break;
      if(result==TDEFL_STATUS_OKAY && !inputBytes && !outputBytes) break;
    }
    const uint32_t compressMs=millis()-compressStarted;
    if(result==TDEFL_STATUS_DONE && inputOffset==rawBytes && outputOffset<rawBytes*9/10 &&
       (rawBytes-outputOffset)*1000UL>compressMs*20000UL) {
      snapshot.encoded=compressed;snapshot.bytes=outputOffset;
      snapshot.wireCrc=crc32(compressed,outputOffset);snapshot.codec=1;
    } else free(compressed);
  } else free(compressed);
  free(compressor);
  rtcDiagnostic={RTC_DIAG_MAGIC,1,0,0};
  queueDiagnostic(DIAG_SNAPSHOT_START,(uint32_t)frameWidth*frameHeight);
  DEBUGF("snapshot frame=%lu %ux%u raw=%lu wire=%lu codec=%u compress_ms=%lu; monitoring paused\n",
    (unsigned long)frameNumber,frameWidth,frameHeight,(unsigned long)rawBytes,
    (unsigned long)snapshot.bytes,snapshot.codec,(unsigned long)(millis()-compressStarted));
  sendSnapshotPart();
  return true;
}
void acceptSnapshotAck(uint32_t seq) {
  if(!snapshot.active || !snapshot.waiting || seq!=snapshot.seq) return;
  snapshot.waiting=false;snapshot.retries=0;
  if(snapshot.type==SNAPSHOT_BEGIN) {
    snapshot.type=SNAPSHOT_CHUNK;
  } else if(snapshot.type==SNAPSHOT_CHUNK) {
    snapshot.offset+=snapshot.chunkLength;
    rtcDiagnostic.offset=snapshot.offset;
    if(snapshot.offset>=snapshot.bytes) snapshot.type=SNAPSHOT_END;
  } else {
    DEBUGF("snapshot complete frame=%lu bytes=%lu\n",(unsigned long)frameNumber,
      (unsigned long)((size_t)frameWidth*frameHeight));
    rtcDiagnostic.outcome=3;queueDiagnostic(DIAG_SNAPSHOT_DONE,(uint32_t)frameWidth*frameHeight,snapshot.offset);rtcDiagnostic.operation=0;
    free(snapshot.encoded);snapshot.encoded=nullptr;snapshot.active=false;return;
  }
  snapshot.seq=nextSeq++;
  sendSnapshotPart();
}
void serviceSnapshot() {
  if(!snapshot.active) return;
  if(millis()-snapshot.sentAt<SNAP_TIMEOUT_MS) return;
  if(snapshot.retries++>=8) {
    DEBUGF("snapshot aborted at offset=%lu after %s retries\n",
      (unsigned long)snapshot.offset,snapshot.waiting?"ack":"send");
    free(snapshot.encoded);snapshot.encoded=nullptr;snapshot.active=false;
    rtcDiagnostic.outcome=snapshot.waiting?2:1;
    queueDiagnostic(DIAG_SNAPSHOT_ABORTED,snapshot.type,snapshot.offset);rtcDiagnostic.operation=0;
    return;
  }
  if(snapshot.type==SNAPSHOT_BEGIN && snapshot.codec && snapshot.retries>=3) {
    free(snapshot.encoded);snapshot.encoded=nullptr;
    snapshot.bytes=(uint32_t)frameWidth*frameHeight;snapshot.wireCrc=snapshot.crc;snapshot.codec=0;
    DEBUGLN("compressed snapshot not acknowledged; falling back to raw frame");
  }
  sendSnapshotPart();
}

bool decodeMac(const char *text,uint8_t *mac) {
  unsigned values[6];
  if(sscanf(text,"%2x:%2x:%2x:%2x:%2x:%2x",&values[0],&values[1],&values[2],
            &values[3],&values[4],&values[5])!=6) return false;
  for(int i=0;i<6;++i) mac[i]=(uint8_t)values[i];
  return true;
}
bool setBridge(const uint8_t *mac) {
  if(sameMac(mac,BROADCAST_MAC) || sameMac(mac,localMac) || !addPeer(mac)) return false;
  memcpy(bridgeMac,mac,6);bridgeKnown=true;
  lastBridgeBeacon=millis();
  char name[18];macText(mac,name);
  DEBUGF("bridge %s paired for this boot\n",name);
  queueDiagnostic(DIAG_BRIDGE_JOIN,configRevision);
  queueAllStates();
  return true;
}
uint32_t lastBaselineSeq=0;
uint32_t lastConfigBeginSeq=0;
uint8_t lastBaselineStatus=ACK_BAD_ORDER;
void setChannel(uint8_t channel);
void handleRadio(const Received &message) {
  const Packet &p=message.packet;
  if(p.type==HELLO && p.length==sizeof(HelloPayload)) {
    HelloPayload beacon;memcpy(&beacon,p.payload,sizeof(beacon));
    if(memcmp(beacon.mac,message.mac,6)!=0 || beacon.channel!=radioChannel ||
       beacon.width!=0 || beacon.height!=0 ||
       sameMac(message.mac,localMac)) return;
    if(!bridgeKnown) setBridge(message.mac);
    if(sameMac(message.mac,bridgeMac)) {
      if(lastBridgeBeacon && millis()-lastBridgeBeacon>12000) queueDiagnostic(DIAG_BRIDGE_REJOIN,millis()-lastBridgeBeacon);
      lastBridgeBeacon=millis();
    }
    return;
  }
  if(!bridgeKnown) {
    if(p.type!=CONFIG_BEGIN) return;
    if(!setBridge(message.mac)) return;
  }
  if(!sameMac(message.mac,bridgeMac)) return;
  // Snapshot/config acknowledgements also prove the bridge is still on this channel.
  lastBridgeBeacon=millis();
  if(p.type==HEALTH_ACK) return;
  if(p.type==DIAGNOSTIC_ACK) {
    uint32_t bootId=0;if(p.length==sizeof(bootId)) memcpy(&bootId,p.payload,sizeof(bootId));
    if(diagnosticCount && bootId==diagnosticBootId && p.seq==diagnosticQueue[diagnosticHead].eventSeq) {
      diagnosticHead=(diagnosticHead+1)%16;--diagnosticCount;
    }
    return;
  }
  if(p.type==SNAPSHOT_ACK) {
    if(p.length==0) acceptSnapshotAck(p.seq);
    return;
  }
  if(p.type==CONFIG_BEGIN) {
    if(p.length!=sizeof(ConfigBeginPayload)) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    ConfigBeginPayload value;memcpy(&value,p.payload,sizeof(value));
    if(p.seq!=lastConfigBeginSeq) {
      lastConfigBeginSeq=p.seq;
      queueDiagnostic(DIAG_CONFIG_BEGIN,value.revision,p.seq);
    }
    const uint8_t result=beginConfig(value);
    if(!sendAck(message.mac,p.seq,p.type,result)) queueDiagnostic(DIAG_CONFIG_ACK_FAILED,result,p.seq);
  } else if(p.type==CONFIG_CELL) {
    if(p.length!=sizeof(ConfigCellPayload)) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    ConfigCellPayload value;memcpy(&value,p.payload,sizeof(value));
    sendAck(message.mac,p.seq,p.type,stageCell(value),value.index);
  } else if(p.type==CONFIG_COMMIT) {
    if(p.length!=sizeof(uint32_t)) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    uint32_t revision;memcpy(&revision,p.payload,sizeof(revision));
    sendAck(message.mac,p.seq,p.type,commitConfig(revision));
  } else if(p.type==CAPTURE_BASELINE) {
    if(p.length!=0) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    if(p.seq!=lastBaselineSeq) {
      lastBaselineStatus=configured && !snapshot.active
        ? (captureBaseline()?ACK_OK:ACK_CAMERA_ERROR) : ACK_BAD_ORDER;
      lastBaselineSeq=p.seq;
    }
    sendAck(message.mac,p.seq,p.type,lastBaselineStatus);
  } else if(p.type==SNAPSHOT_REQUEST) {
    if(p.length!=0) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    if(snapshot.active) {
      sendAck(message.mac,p.seq,p.type,p.seq==snapshot.requestSeq?ACK_OK:ACK_BUSY);
    } else {
      const bool ready=framePixels && frameNumber;
      sendAck(message.mac,p.seq,p.type,ready?ACK_OK:ACK_BUSY);
      if(ready) startSnapshot(p.seq);
    }
  }
}

void setChannel(uint8_t channel) {
  if(channel<1 || channel>13) return;
  esp_now_del_peer(BROADCAST_MAC);
  if(bridgeKnown) esp_now_del_peer(bridgeMac);
  if(esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE)==ESP_OK) {
    radioChannel=channel;addPeer(BROADCAST_MAC);
    if(bridgeKnown) addPeer(bridgeMac);
    DEBUGF("ESP-NOW channel=%u\n",radioChannel);
  }
}

void printInfo() {
#if CAMERA_DEBUG_SERIAL
  char mac[18];macText(localMac,mac);
  DEBUGF("version %s mac=%s channel=%u bridge=%u config=%lu cells=%u groups=%u baseline=%u frame=%lu failures=%lu queue=%u\n",
    CAMERA_MODULE_VERSION,mac,radioChannel,bridgeKnown,(unsigned long)configRevision,
    cellCount,groupCount,baselineReady,(unsigned long)frameNumber,
    (unsigned long)captureFailures,stateCount);
  for(uint16_t i=0;i<cellCount;++i) {
    const CellRuntime &c=cells[i];
    DEBUGF("[%u] id=%lu group=%lu (%u,%u) r=%u %s angle0=%.1f angles=%u anchors=%u score=%u state=%u\n",
      i,(unsigned long)c.config.id,(unsigned long)c.config.groupId,c.config.x,c.config.y,
      c.config.radius,c.config.shape?"square":"circle",c.reference.peakAngle[0],
      c.reference.peakCount,c.anchorCount,c.scorePermille,c.state);
  }
#endif
}
void serialSnapshot() {
#if CAMERA_DEBUG_SERIAL
  if(snapshot.active || !frameNumber || !framePixels) { DEBUGLN("snapshot unavailable");return; }
  const uint32_t bytes=(uint32_t)frameWidth*frameHeight;
  const uint32_t crc=crc32(framePixels,bytes);
  struct __attribute__((packed)) SerialFrameHeader {
    char magic[8];uint32_t frame,bytes,crc;uint16_t width,height;
  } header={{'R','A','I','L','F','R','M','1'},frameNumber,bytes,crc,frameWidth,frameHeight};
  Serial.write((const uint8_t *)&header,sizeof(header));
  Serial.write(framePixels,bytes);Serial.flush();
  DEBUGF("\nsnapshot bytes=%lu crc=%08lx\n",(unsigned long)bytes,(unsigned long)crc);
#endif
}
char serialLine[180];size_t serialLength=0;
void handleSerialLine(char *line) {
  char *command=strtok(line," \t");if(!command) return;
  if(strcmp(command,"H")==0) {
    DEBUGLN("H help | I info | P MAC pair | C channel | B revision count resolution [brightness contrast saturation vflip hmirror] | S index id group x y radius C/S contrast tolerance threshold_permille enter clear | A commit | R baseline | F raw frame | X clear RAM config | Z FORMAT LittleFS");
  } else if(strcmp(command,"I")==0) printInfo();
  else if(strcmp(command,"P")==0) {
    char *value=strtok(nullptr," \t");uint8_t mac[6];
    DEBUGLN(value && decodeMac(value,mac) && setBridge(mac)?"pair OK":"pair failed");
  } else if(strcmp(command,"C")==0) {
    char *value=strtok(nullptr," \t");
    if(value) setChannel(atoi(value));
  } else if(strcmp(command,"B")==0) {
    char *args=strtok(nullptr,"");
    unsigned long revision;unsigned count,resolution;
    int brightness=0,contrast=0,saturation=0,vflip=0,hmirror=0;
    const int fields=args?sscanf(args,"%lu %u %u %d %d %d %d %d",&revision,&count,
      &resolution,&brightness,&contrast,&saturation,&vflip,&hmirror):0;
    if((fields==3 || fields==8) && count<=MAX_CELLS && resolution<=XGA &&
       brightness>=-2 && brightness<=2 && contrast>=-2 && contrast<=2 &&
       saturation>=-2 && saturation<=2 && vflip>=0 && vflip<=1 &&
       hmirror>=0 && hmirror<=1) {
      ConfigBeginPayload value={(uint32_t)revision,(uint16_t)count,
        {(uint8_t)resolution,(int8_t)brightness,(int8_t)contrast,(int8_t)saturation,
         (uint8_t)vflip,(uint8_t)hmirror}};
      DEBUGF("begin status=%u\n",beginConfig(value));
    } else DEBUGLN("B syntax error");
  } else if(strcmp(command,"S")==0) {
    char *args=strtok(nullptr,"");
    unsigned index,x,y,radius,contrast,tolerance,threshold,enter,clear;
    unsigned long id,group;char shape;
    if(args && sscanf(args,"%u %lu %lu %u %u %u %c %u %u %u %u %u",
       &index,&id,&group,&x,&y,&radius,&shape,&contrast,&tolerance,&threshold,&enter,&clear)==12 &&
       index<=MAX_CELLS && id<=UINT32_MAX && group<=UINT32_MAX && x<=UINT16_MAX &&
       y<=UINT16_MAX && radius<=UINT8_MAX && (shape=='C' || shape=='S') &&
       contrast<=UINT16_MAX && tolerance<=UINT8_MAX && threshold<=UINT16_MAX &&
       enter<=UINT8_MAX && clear<=UINT8_MAX) {
      ConfigCellPayload value={};value.index=index;
      value.cell={(uint32_t)id,(uint32_t)group,(uint16_t)x,(uint16_t)y,
        (uint8_t)radius,(uint8_t)(shape=='S'?1:0),(uint16_t)contrast,
        (uint16_t)threshold,(uint8_t)tolerance,(uint8_t)enter,(uint8_t)clear};
      DEBUGF("cell status=%u\n",stageCell(value));
    } else DEBUGLN("S syntax error");
  } else if(strcmp(command,"A")==0) DEBUGF("commit status=%u\n",commitConfig(stagingRevision));
  else if(strcmp(command,"R")==0) DEBUGLN(configured && captureBaseline()?"baseline OK":"baseline failed");
  else if(strcmp(command,"F")==0) serialSnapshot();
  else if(strcmp(command,"X")==0) {
    cellCount=groupCount=0;baselineReady=configured=stagingOpen=false;
    stateHead=stateCount=0;DEBUGLN("configuration cleared from RAM");
  } else if(strcmp(command,"Z")==0) {
    char *confirmation=strtok(nullptr," \t");
    if(!confirmation || strcmp(confirmation,"FORMAT")!=0 || snapshot.active || stagingOpen) {
      DEBUGLN("Z FORMAT required; camera must be idle");
    } else {
      LittleFS.end();storageReady=false;
      if(LittleFS.format()) storageReady=LittleFS.begin(false);
      baselineReady=false;queueAllStates();
      DEBUGLN(storageReady?"LittleFS formatted; capture a new baseline":"LittleFS format failed");
    }
  } else DEBUGLN("unknown command; H for help");
}
void pollSerial() {
  while(Serial.available()) {
    const char ch=Serial.read();
    if(ch=='\r') continue;
    if(ch=='\n') {
      serialLine[serialLength]=0;
      if(serialLength) handleSerialLine(serialLine);
      serialLength=0;
    } else if(serialLength<sizeof(serialLine)-1) serialLine[serialLength++]=ch;
    else serialLength=0;
  }
}

bool blankStorage(const esp_partition_t *partition) {
  uint8_t bytes[256];
  for(size_t offset=0;offset<partition->size;offset+=sizeof(bytes)) {
    size_t count=min(sizeof(bytes),(size_t)partition->size-offset);
    if(esp_partition_read(partition,offset,bytes,count)!=ESP_OK) return false;
    for(size_t i=0;i<count;++i) if(bytes[i]!=0xFF) return false;
  }
  return true;
}
void setup() {
  pinMode(STATUS_LED,OUTPUT);digitalWrite(STATUS_LED,LOW);
  Serial.begin(115200);delay(200);
  DEBUGF("Railway camera %s boot\n",CAMERA_MODULE_VERSION);
  if(!psramFound()) { DEBUGLN("PSRAM required");return; }
  cells=(CellRuntime *)ps_malloc(sizeof(CellRuntime)*MAX_CELLS);
  staging=(CellRuntime *)ps_malloc(sizeof(CellRuntime)*MAX_CELLS);
  stateQueue=(StatePayload *)ps_malloc(sizeof(StatePayload)*STATE_QUEUE_CAPACITY);
  inbox=xQueueCreate(16,sizeof(Received));
  if(!cells || !staging || !stateQueue || !inbox) { DEBUGLN("allocation failed");return; }
  memset(cells,0,sizeof(CellRuntime)*MAX_CELLS);
  memset(staging,0,sizeof(CellRuntime)*MAX_CELLS);
  for(int degree=0;degree<=20;++degree)
    tanQ8[degree]=(uint16_t)lroundf(tanf(degree*0.01745329252f)*256);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  diagnosticBootId=esp_random();diagnosticResetReason=(uint8_t)esp_reset_reason();
  const uint32_t previousOperation=rtcDiagnostic.magic==RTC_DIAG_MAGIC
    ? ((uint32_t)rtcDiagnostic.operation<<8)|rtcDiagnostic.outcome : 0;
  const uint32_t previousOffset=rtcDiagnostic.magic==RTC_DIAG_MAGIC?rtcDiagnostic.offset:0;
  rtcDiagnostic={RTC_DIAG_MAGIC,0,0,0};
  queueDiagnostic(DIAG_BOOT,previousOperation,previousOffset);
  esp_wifi_get_mac(WIFI_IF_STA,localMac);
  esp_wifi_set_channel(radioChannel,WIFI_SECOND_CHAN_NONE);
  if(esp_now_init()!=ESP_OK) { DEBUGLN("ESP-NOW init failed");return; }
  esp_now_register_recv_cb(onReceive);
  addPeer(BROADCAST_MAC);
  const esp_partition_t *storage=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_DATA_SPIFFS,"spiffs");
  bool freshStorage=storage && blankStorage(storage);
  storageReady=storage && LittleFS.begin(freshStorage);
  if(!storage) DEBUGLN("LittleFS partition missing");
  if(!storageReady) DEBUGLN("LittleFS unavailable; persistent baseline disabled");
  else if(freshStorage) DEBUGLN("LittleFS initialized");
  initCamera(cameraSettings);
  loadBaseline();
  queueDiagnostic(cameraReady?DIAG_READY:DIAG_INIT_FAILED,configRevision);
  printInfo();
  DEBUGLN("Send H for USB commands. Configuration and baseline are volatile.");
}

void loop() {
  pollSerial();
  if(!inbox) { delay(100);return; }
  Received message;
  for(int n=0;n<16 && xQueueReceive(inbox,&message,0)==pdTRUE;++n) handleRadio(message);
  serviceSnapshot();
  serviceDiagnostics();
  if(cameraReady && !snapshot.active && millis()-lastCapture>=100) {
    if(captureFrame()) analyseAllCells();
  }
  flushState();
  if(millis()-lastHello>=HELLO_MS) { lastHello=millis();sendHello(); }
  if(millis()-lastHealth>=HEALTH_MS) { lastHealth=millis();sendHealth(); }
  // A router channel change can move the bridge. Search until its beacon reappears.
  if(!snapshot.active && millis()-lastBridgeBeacon>18000 && millis()-lastChannelScan>=1400) {
    lastChannelScan=millis();
    setChannel(radioChannel==13?1:radioChannel+1);
  }
  static uint32_t lastRefresh=0;
  if(millis()-lastRefresh>=30000) { lastRefresh=millis();queueAllStates(); }
  delay(1);
}
