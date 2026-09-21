#define CAMERA_MODULE_VERSION "0.2.0-fixed-histogram"
#define CAMERA_DEBUG_SERIAL 1
// Override these in the build flags for another supported camera board.
#if !defined(CAMERA_BOARD_AI_THINKER) && !defined(CAMERA_BOARD_ESP32S3_EYE)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define CAMERA_BOARD_ESP32S3_EYE 1
#else
#define CAMERA_BOARD_AI_THINKER 1
#endif
#endif

// Railway camera firmware. ESP-NOW uses the Wi-Fi radio without an IP network.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <miniz.h>
#include <LittleFS.h>
#include <esp_partition.h>
#include <math.h>
#include <stddef.h>

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#error ESP32-P4 needs a board-specific image source and ESP-NOW transport via a companion radio
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
constexpr uint8_t MAX_PEAKS=10, FIXED_DIRECTIONS=9, BINS=36, MAX_ANCHORS=2;
constexpr uint8_t POSITION_BANDS=3, SPATIAL_BUCKETS=MAX_PEAKS*POSITION_BANDS+1;
constexpr uint32_t HELLO_MS=2000, HEALTH_MS=5000, SNAP_TIMEOUT_MS=250;
constexpr uint8_t BROADCAST_MAC[6]={255,255,255,255,255,255};

enum MessageType : uint8_t {
  HELLO=1, CONFIG_BEGIN=2, CONFIG_CELL=3, CONFIG_COMMIT=4,
  CAPTURE_BASELINE=5, SNAPSHOT_REQUEST=6, ACK=7,
  STATE=8, HEALTH=9, SNAPSHOT_BEGIN=10, SNAPSHOT_CHUNK=11,
  SNAPSHOT_END=12, SNAPSHOT_ACK=13, DIAGNOSTIC=14, DIAGNOSTIC_ACK=15,
  HEALTH_ACK=16, ANALYSIS_REQUEST=17, CELL_ANALYSIS=18,
  CALIBRATE_REQUEST=19, CALIBRATION_RESULT=20, CALIBRATION_ACK=21
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
  uint32_t createdRevision;
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
struct __attribute__((packed)) AnalysisPayload {
  uint32_t id,revision,frame;
  uint16_t scorePermille,thresholdPermille,referenceEdges,liveEdges;
  uint8_t peakCount;
  uint16_t lineAngleTenths[MAX_PEAKS];
  uint16_t referenceBuckets[SPATIAL_BUCKETS],liveBuckets[SPATIAL_BUCKETS];
};
struct __attribute__((packed)) CalibrationRequestPayload { uint32_t revision,durationMs,sinceRevision;uint8_t maxSamples; };
struct __attribute__((packed)) CalibrationResultPayload {
  uint32_t revision,id;uint16_t index,count,thresholdPermille,clearMaximum,referenceEdges,samples;
  uint8_t radius,confidence;
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
static_assert(sizeof(AnalysisPayload)<=RADIO_PAYLOAD, "Analysis message too large");
static_assert(sizeof(CalibrationResultPayload)<=RADIO_PAYLOAD, "Calibration result too large");

#include "ImageSource.h"
ImageSource imageSource;
CameraSettings cameraSettings={QVGA,0,0,0,0,0};
uint16_t frameWidth=320,frameHeight=240;
uint8_t *framePixels=nullptr;
bool cameraReady=false, baselineReady=false, configured=false;
bool storageReady=false;
uint8_t baselineStorageError=0;
uint32_t frameNumber=0, configRevision=0, captureFailures=0;
uint32_t lastHello=0,lastHealth=0,lastCapture=0,nextSeq=1;
uint32_t lastFullStateRefresh=0;
uint8_t localMac[6]={},bridgeMac[6]={};
bool bridgeKnown=false;
uint8_t radioChannel=1;
uint32_t lastBridgeBeacon=0,lastChannelScan=0,lastBridgeBeaconSeq=0;
constexpr uint32_t BRIDGE_SEARCH_AFTER_MS=5000,CHANNEL_DWELL_MS=600;

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
  // Retained only to read baselines written by earlier firmware revisions.
  // OccupancyDetector never uses these saved edge locations.
  Anchor anchors[MAX_ANCHORS]={};
  uint8_t anchorCount=0;
  uint8_t state=UNKNOWN,enterCount=0,clearCount=0;
  uint16_t scorePermille=0;
  int16_t directionX[MAX_PEAKS]={},directionY[MAX_PEAKS]={};
  uint16_t referenceBuckets[SPATIAL_BUCKETS]={};
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
// The board adapter owns the camera driver; the rest of the sketch sees grayscale frames.
bool initCamera(const CameraSettings &settings) {
  const bool ok=imageSource.begin(settings,framePixels,frameWidth,frameHeight);
  cameraReady=ok;
  if(ok) cameraSettings=settings;
  return ok;
}
bool captureFrame() {
  if(!cameraReady || !framePixels) return false;
  lastCapture=millis();
  if(!imageSource.capture(framePixels,frameWidth,frameHeight)) { ++captureFailures;return false; }
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

#include "OccupancyDetector.h"
OccupancyDetector detector;

void baselineHeartbeat() {
  if(millis()-lastHello>=1000) { sendHello();lastHello=millis();delay(2); }
  if(millis()-lastHealth>=5000) { sendHealth();lastHealth=millis();delay(2); }
}
bool captureBaseline() {
  constexpr uint8_t BASELINE_FRAMES=3;
  const uint32_t started=millis();
  rtcDiagnostic={RTC_DIAG_MAGIC,2,0,0};
  queueDiagnostic(DIAG_BASELINE_START,configRevision);
  DEBUGF("baseline capture start revision=%lu frames=%u from_frame=%lu\n",(unsigned long)configRevision,BASELINE_FRAMES,(unsigned long)frameNumber);
  sendHello();lastHello=millis();
  const size_t frameBytes=(size_t)frameWidth*frameHeight;
  uint8_t *meanPixels=snapshot.active?nullptr:(uint8_t *)ps_malloc(frameBytes);
  if(!meanPixels) { rtcDiagnostic.outcome=1;queueDiagnostic(DIAG_BASELINE_FAILED,1);rtcDiagnostic.operation=0;return false; }
  bool captured=true;
  for(uint8_t frame=0;frame<BASELINE_FRAMES;++frame) {
    if(!captureFrame()) { captured=false;break; }
    if(frame==0) memcpy(meanPixels,framePixels,frameBytes);
    else for(size_t p=0;p<frameBytes;++p)
      meanPixels[p]=(uint8_t)(((uint16_t)meanPixels[p]*frame+framePixels[p]+frame/2)/(frame+1));
    DEBUGF("baseline frame %u/%u captured frame=%lu\n",frame+1,BASELINE_FRAMES,(unsigned long)frameNumber);
    baselineHeartbeat();
  }
  if(!captured) { free(meanPixels);rtcDiagnostic.outcome=1;queueDiagnostic(DIAG_BASELINE_FAILED,1);rtcDiagnostic.operation=0;return false; }
  memcpy(framePixels,meanPixels,frameBytes);free(meanPixels);
  detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
  for(uint16_t i=0;i<cellCount;++i) {
    // Retain the flash record layout, but do not save old edge locations.
    memset(cells[i].anchors,0,sizeof(cells[i].anchors));cells[i].anchorCount=0;
    detector.calibrateCell(cells[i]);baselineHeartbeat();
  }
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
  DEBUGF("baseline ready: %u cells frames=%u last_frame=%lu duration_ms=%lu\n",cellCount,BASELINE_FRAMES,(unsigned long)frameNumber,(unsigned long)(millis()-started));
  rtcDiagnostic.outcome=3;queueDiagnostic(DIAG_BASELINE_DONE,millis()-started);rtcDiagnostic.operation=0;
  return true;
}

// Test five nested radii at every configured centre during one shared,
// wall-clock-limited empty-scene run. The configured radius is the user's
// maximum search radius; the smallest stable candidate with enough texture wins.
bool autoCalibrate(const CalibrationRequestPayload &request) {
  constexpr uint8_t CANDIDATES=5;
  if(!configured || snapshot.active || !cellCount || request.revision!=configRevision+1 ||
     request.durationMs<2000 || request.durationMs>30000 || !request.maxSamples) return false;
  const uint32_t previousRevision=configRevision;const bool previousBaselineReady=baselineReady;
  memcpy(staging,cells,(size_t)cellCount*sizeof(CellRuntime));
  const size_t total=(size_t)cellCount*CANDIDATES;
  CellRuntime *trials=(CellRuntime *)ps_malloc(total*sizeof(CellRuntime));
  struct Stats { uint32_t sum=0;uint16_t maximum=0,samples=0; };
  Stats *stats=(Stats *)ps_malloc(total*sizeof(Stats));
  const size_t frameBytes=(size_t)frameWidth*frameHeight;
  uint8_t *meanPixels=(uint8_t *)ps_malloc(frameBytes);
  if(!trials || !stats || !meanPixels) { free(trials);free(stats);free(meanPixels);return false; }
  memset(stats,0,total*sizeof(Stats));
  for(uint16_t i=0;i<cellCount;++i) for(uint8_t candidate=0;candidate<CANDIDATES;++candidate) {
    CellRuntime &trial=trials[(size_t)i*CANDIDATES+candidate];trial=CellRuntime();trial.config=cells[i].config;
    const uint8_t maximum=cells[i].config.radius;
    trial.config.radius=(uint8_t)(3+((uint16_t)(maximum-3)*candidate+(CANDIDATES-1)/2)/(CANDIDATES-1));
  }
  const uint32_t started=millis();uint8_t baselineFrames=0;
  while(baselineFrames<3 && millis()-started<request.durationMs) {
    if(!captureFrame()) continue;
    if(!baselineFrames) memcpy(meanPixels,framePixels,frameBytes);
    else for(size_t p=0;p<frameBytes;++p)
      meanPixels[p]=(uint8_t)(((uint16_t)meanPixels[p]*baselineFrames+framePixels[p]+baselineFrames/2)/(baselineFrames+1));
    ++baselineFrames;baselineHeartbeat();
  }
  if(baselineFrames<3) { free(trials);free(stats);free(meanPixels);return false; }
  uint8_t *livePixels=framePixels;framePixels=meanPixels;
  detector.bind(framePixels,frameWidth,frameHeight,trials,(uint16_t)min(total,(size_t)65535),groups,0,nullptr);
  for(size_t n=0;n<total;++n) { detector.calibrateCell(trials[n]);if((n&15)==15) baselineHeartbeat(); }
  framePixels=livePixels;detector.bind(framePixels,frameWidth,frameHeight,trials,(uint16_t)min(total,(size_t)65535),groups,0,nullptr);
  uint32_t nextSample=millis();uint8_t sampleCount=0;
  const uint32_t interval=max((uint32_t)1,request.durationMs/request.maxSamples);
  while(millis()-started<request.durationMs) {
    baselineHeartbeat();
    if((int32_t)(millis()-nextSample)<0) { delay(1);continue; }
    nextSample+=interval;
    if(!captureFrame()) continue;
    for(size_t n=0;n<total;++n) {
      Feature live={};uint16_t buckets[SPATIAL_BUCKETS]={};
      const uint16_t score=detector.inspectCell(trials[n],live,buckets);
      stats[n].sum+=score;stats[n].maximum=max(stats[n].maximum,score);++stats[n].samples;
      if((n&15)==15) baselineHeartbeat();
    }
    ++sampleCount;
  }
  for(uint16_t i=0;i<cellCount;++i) {
    // Prefer the smallest candidate only when the short calibration window
    // shows a genuinely quiet empty scene. If none qualifies, retain the
    // user's maximum radius rather than selecting a noisy smaller crop.
    const bool tune=request.sinceRevision==0 || staging[i].config.createdRevision>request.sinceRevision;
    uint8_t chosen=CANDIDATES-1;
    if(tune) for(uint8_t candidate=0;candidate<CANDIDATES;++candidate) {
      const size_t n=(size_t)i*CANDIDATES+candidate;
      if(trials[n].reference.edges>=100 && stats[n].samples>=3 && stats[n].maximum<=150) { chosen=candidate;break; }
    }
    const size_t n=(size_t)i*CANDIDATES+chosen;
    cells[i]=trials[n];
    // Ten seconds cannot represent every later daylight condition. Preserve
    // the user's existing threshold and add a deliberately conservative
    // margin above the worst empty score observed during calibration.
    uint16_t threshold=staging[i].config.thresholdPermille;
    if(tune) {
      threshold=(uint16_t)constrain((int)stats[n].maximum*2+100,400,800);
      threshold=max(threshold,staging[i].config.thresholdPermille);
      threshold=(uint16_t)(((threshold+9)/10)*10);
    }
    cells[i].config.thresholdPermille=threshold;
    cells[i].state=CLEAR;cells[i].scorePermille=0;
  }
  configRevision=request.revision;baselineReady=saveBaseline();
  if(baselineReady) {
    for(uint16_t g=0;g<groupCount;++g) groups[g].state=CLEAR;
    for(uint16_t i=0;i<cellCount;++i) {
      const size_t base=(size_t)i*CANDIDATES;uint8_t chosen=0;
      for(;chosen<CANDIDATES;++chosen) if(trials[base+chosen].config.radius==cells[i].config.radius) break;
      const Stats &s=stats[base+min(chosen,(uint8_t)(CANDIDATES-1))];
      CalibrationResultPayload result={configRevision,cells[i].config.id,i,cellCount,
        cells[i].config.thresholdPermille,s.maximum,cells[i].reference.edges,s.samples,
        cells[i].config.radius,(uint8_t)(s.samples>=40?2:s.samples>=15?1:0)};
      for(uint8_t attempt=0;attempt<3;++attempt) { transmit(bridgeMac,CALIBRATION_RESULT,nextSeq++,&result,sizeof(result));delay(5); }
    }
    queueAllStates();sendHello();lastHello=millis();
  } else {
    memcpy(cells,staging,(size_t)cellCount*sizeof(CellRuntime));
    configRevision=previousRevision;baselineReady=previousBaselineReady;
  }
  free(trials);free(stats);free(meanPixels);
  detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
  DEBUGF("auto calibration revision=%lu cells=%u samples=%u duration_ms=%lu result=%u\n",
    (unsigned long)configRevision,cellCount,sampleCount,(unsigned long)(millis()-started),baselineReady);
  return baselineReady;
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
    const bool wasReady=cameraReady;
    if(!initCamera(stagingSettings)) {
      if(wasReady) initCamera(previous);
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
  BaselineHeader h={0x52424C39,configRevision,0,
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
  if(f.read((uint8_t *)&h,sizeof(h))!=sizeof(h) || h.magic!=0x52424C39 ||
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
  queueAllStates();lastFullStateRefresh=millis();
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
    const uint32_t now=millis();
    const bool newBridge=!bridgeKnown;
    const bool missedBridge=lastBridgeBeacon && now-lastBridgeBeacon>1500;
    const bool bridgeRestarted=lastBridgeBeaconSeq && (int32_t)(p.seq-lastBridgeBeaconSeq)<0;
    if(!bridgeKnown) setBridge(message.mac);
    if(sameMac(message.mac,bridgeMac)) {
      if(lastBridgeBeacon && now-lastBridgeBeacon>12000) queueDiagnostic(DIAG_BRIDGE_REJOIN,now-lastBridgeBeacon);
      lastBridgeBeacon=now;lastBridgeBeaconSeq=p.seq;
      if(newBridge || missedBridge || bridgeRestarted) {
        if(!newBridge) { queueAllStates();lastFullStateRefresh=now; }
        // Reply promptly, with a short random offset so several cameras do not answer together.
        lastHello=now-HELLO_MS+(esp_random()%200);
        lastHealth=now-HEALTH_MS+250+(esp_random()%200);
      }
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
  } else if(p.type==ANALYSIS_REQUEST) {
    uint32_t id=0;if(p.length==sizeof(id)) memcpy(&id,p.payload,sizeof(id));
    int index=-1;for(uint16_t i=0;i<cellCount;++i) if(cells[i].config.id==id) { index=i;break; }
    if(p.length!=sizeof(id) || index<0 || !baselineReady || !framePixels) return;
    Feature live={};uint16_t liveBuckets[SPATIAL_BUCKETS]={};
    detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
    const uint16_t score=detector.inspectCell(cells[index],live,liveBuckets);
    AnalysisPayload value={};
    value.id=id;value.revision=configRevision;value.frame=frameNumber;
    value.scorePermille=score;value.thresholdPermille=cells[index].config.thresholdPermille;
    value.referenceEdges=cells[index].reference.edges;value.liveEdges=live.edges;
    value.peakCount=cells[index].reference.peakCount;
    for(uint8_t peak=0;peak<value.peakCount;++peak)
      value.lineAngleTenths[peak]=(uint16_t)lroundf(fmodf(cells[index].reference.peakAngle[peak]+90.0f,180.0f)*10.0f);
    memcpy(value.referenceBuckets,cells[index].referenceBuckets,sizeof(value.referenceBuckets));
    memcpy(value.liveBuckets,liveBuckets,sizeof(value.liveBuckets));
    transmit(bridgeMac,CELL_ANALYSIS,p.seq,&value,sizeof(value));
  } else if(p.type==CALIBRATE_REQUEST) {
    CalibrationRequestPayload value={};
    if(p.length!=sizeof(value)) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    memcpy(&value,p.payload,sizeof(value));
    if(!configured || snapshot.active || value.revision!=configRevision+1) { sendAck(message.mac,p.seq,p.type,ACK_BAD_ORDER);return; }
    sendAck(message.mac,p.seq,p.type,ACK_OK);
    autoCalibrate(value);
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
    // Probe each scanned channel instead of relying on one periodic beacon.
    sendHello();lastHello=millis();
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
    DEBUGF("[%u] id=%lu group=%lu (%u,%u) r=%u %s angle0=%.1f angles=%u score=%u state=%u\n",
      i,(unsigned long)c.config.id,(unsigned long)c.config.groupId,c.config.x,c.config.y,
      c.config.radius,c.config.shape?"square":"circle",c.reference.peakAngle[0],
      c.reference.peakCount,c.scorePermille,c.state);
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
  detector.begin();
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
  if(cameraReady && !snapshot.active) {
    if(captureFrame()) {
      detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
      if(baselineReady) detector.analyseAllCells();
    }
  }
  flushState();
  if(millis()-lastHello>=HELLO_MS) { lastHello=millis();sendHello(); }
  if(millis()-lastHealth>=HEALTH_MS) { lastHealth=millis();sendHealth(); }
  // A router channel change can move the bridge. Search until its beacon reappears.
  if(!snapshot.active && millis()-lastBridgeBeacon>BRIDGE_SEARCH_AFTER_MS && millis()-lastChannelScan>=CHANNEL_DWELL_MS) {
    lastChannelScan=millis();
    setChannel(radioChannel==13?1:radioChannel+1);
  }
  if(millis()-lastFullStateRefresh>=30000) { lastFullStateRefresh=millis();queueAllStates(); }
  yield();
}
