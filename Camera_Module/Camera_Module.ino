#define CAMERA_MODULE_VERSION "0.2.22"
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
#include <esp_heap_caps.h>
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
constexpr uint8_t PROTOCOL_VERSION=3;
constexpr size_t RADIO_PAYLOAD=200; // Fits legacy ESP-NOW's 250-byte limit.
constexpr uint16_t MAX_CELLS=300;
constexpr uint16_t MAX_GROUPS=64;
constexpr uint8_t MAX_PEAKS=12, FIXED_DIRECTIONS=12;
constexpr uint8_t POSITION_BANDS=3, SPATIAL_BUCKETS=MAX_PEAKS*POSITION_BANDS+1;
constexpr uint32_t HELLO_MS=2000, HEALTH_MS=5000, SNAP_TIMEOUT_MS=250;
constexpr uint8_t BROADCAST_MAC[6]={255,255,255,255,255,255};

enum MessageType : uint8_t {
  HELLO=1, CONFIG_BEGIN=2, CONFIG_CELL=3, CONFIG_COMMIT=4,
  CAPTURE_BASELINE=5, SNAPSHOT_REQUEST=6, ACK=7,
  HEALTH=9, SNAPSHOT_BEGIN=10, SNAPSHOT_CHUNK=11,
  SNAPSHOT_END=12, SNAPSHOT_ACK=13, DIAGNOSTIC=14, DIAGNOSTIC_ACK=15,
  HEALTH_ACK=16, ANALYSIS_REQUEST=17, CELL_ANALYSIS=18,
  CALIBRATE_REQUEST=19, CALIBRATION_RESULT=20, CALIBRATION_ACK=21,
  STATE_BITMAP=22, SCORE_BATCH=23
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
  uint8_t enterFrames, clearFrames;
  uint32_t createdRevision;
};
struct __attribute__((packed)) ConfigCellPayload {
  uint16_t index;
  CellConfig cell;
};
struct __attribute__((packed)) AckPayload { uint8_t forType,status; uint16_t detail; };
struct __attribute__((packed)) ScorePending { uint32_t id; uint16_t scorePermille; };
constexpr uint16_t STATE_BITMAP_BYTES=(MAX_CELLS*2+7)/8;
struct __attribute__((packed)) StateBitmapPayload {
  uint32_t revision,frame;uint16_t count;uint8_t encoding;
  uint8_t states[STATE_BITMAP_BYTES];
};
struct __attribute__((packed)) ScoreRecord { uint16_t index,scorePermille; };
constexpr uint8_t MAX_BATCHED_SCORES=(RADIO_PAYLOAD-9)/sizeof(ScoreRecord);
struct __attribute__((packed)) ScoreBatchPayload {
  uint32_t revision,frame;uint8_t count;ScoreRecord records[MAX_BATCHED_SCORES];
};
static_assert(sizeof(StateBitmapPayload)<=RADIO_PAYLOAD && sizeof(ScoreBatchPayload)<=RADIO_PAYLOAD,"State payload exceeds ESP-NOW payload");
struct __attribute__((packed)) AnalysisPayload {
  uint32_t id,revision,frame;
  uint16_t scorePermille,thresholdPermille,referenceEdges,liveEdges;
  uint8_t peakCount;
  uint16_t lineAngleTenths[MAX_PEAKS];
  uint16_t referenceBuckets[SPATIAL_BUCKETS],liveBuckets[SPATIAL_BUCKETS];
};
static_assert(sizeof(AnalysisPayload)<=RADIO_PAYLOAD,"Analysis payload exceeds ESP-NOW payload");
enum CalibrationMode:uint8_t { AUTO_SIZE=0, RETUNE_LIGHTING=1 };
struct __attribute__((packed)) CalibrationRequestPayload {
  uint32_t revision,durationMs,sinceRevision;uint8_t maxSamples,mode;uint32_t targetId;
};
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
uint32_t frameCapturedAt=0,analysisCount=0,analysisWindowStarted=0;
uint32_t lastPerformanceCaptureCount=0,lastPerformanceAnalysisCount=0;
uint32_t lastFullStateRefresh=0;
uint8_t localMac[6]={},bridgeMac[6]={};
bool bridgeKnown=false;
uint8_t radioChannel=1;
uint32_t lastBridgeBeacon=0,lastChannelScan=0,lastBridgeBeaconSeq=0;
// Stay long enough for several normal bridge beacons. A 600 ms dwell gave a
// scanning camera only one marginal receive opportunity on each channel and
// produced repeated one-way discoveries at noisy installations.
// Normal beacons arrive every 500 ms and health acknowledgements also prove
// contact. Require roughly thirty missed beacon periods before abandoning the
// known channel; leaving after five seconds made brief coexistence delays turn
// into a disruptive 13-channel search.
constexpr uint32_t BRIDGE_SEARCH_AFTER_MS=15000,CHANNEL_DWELL_MS=2000;

struct Feature {
  uint16_t edges=0,maxGradient=0;
  bool textured=false;
};
struct CellRuntime {
  CellConfig config={};
  Feature reference={};
  uint8_t state=UNKNOWN,enterCount=0,clearCount=0;
  uint16_t scorePermille=0;
  uint16_t referenceBuckets[SPATIAL_BUCKETS]={};
  // A second, illumination-normalised view of the same gradients.  The
  // projected bands retain which side of a line changed; these equal-area
  // rings retain whether its extent moved towards or away from the centre.
  uint16_t referenceRadialBuckets[SPATIAL_BUCKETS]={};
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

struct Received { uint8_t mac[6],channel; Packet packet; };
QueueHandle_t inbox=nullptr;
struct SnapshotTransfer {
  bool active=false,waiting=false;
  uint8_t type=0,retries=0;
  uint16_t chunkLength=0;
  uint32_t offset=0,seq=0,requestSeq=0,sentAt=0,crc=0,wireCrc=0,bytes=0;
  uint8_t codec=0;
  uint8_t *encoded=nullptr;
} snapshot;
struct CalibrationTransfer {
  bool active=false;
  CalibrationResultPayload *results=nullptr;
  uint16_t count=0,index=0;
  uint8_t retries=0;
  uint32_t seq=0,sentAt=0,revision=0;
} calibration;
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
  const uint32_t now=millis();
  if(!analysisWindowStarted) analysisWindowStarted=now;
  const uint32_t elapsed=now-analysisWindowStarted;
  if(elapsed) {
    const uint32_t captures=imageSource.acquisitionCount();
    DEBUGF("pipeline capture_fps=%lu.%lu analysis_fps=%lu.%lu frame_age_ms=%lu replaced=%lu acquisition_failures=%lu free_psram=%lu largest_psram=%lu\n",
      (unsigned long)(((captures-lastPerformanceCaptureCount)*1000)/elapsed),
      (unsigned long)((((captures-lastPerformanceCaptureCount)*10000)/elapsed)%10),
      (unsigned long)(((analysisCount-lastPerformanceAnalysisCount)*1000)/elapsed),
      (unsigned long)((((analysisCount-lastPerformanceAnalysisCount)*10000)/elapsed)%10),
      (unsigned long)(frameCapturedAt?now-frameCapturedAt:0),
      (unsigned long)imageSource.replacedFrames(),
      (unsigned long)imageSource.acquisitionFailures(),
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
      (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    lastPerformanceCaptureCount=captures;lastPerformanceAnalysisCount=analysisCount;
    analysisWindowStarted=now;
  }
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
  if(info->rx_ctrl) item.channel=info->rx_ctrl->channel;
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
  for(uint8_t attempt=0;attempt<3;++attempt) {
    if(imageSource.begin(settings,framePixels,frameWidth,frameHeight)) {
      cameraReady=true;cameraSettings=settings;return true;
    }
    cameraReady=false;
    if(attempt<2) delay(150);
  }
  return false;
}
bool applyCameraSettings(const CameraSettings &settings) {
  if(cameraReady) {
    if(!imageSource.reconfigure(settings,framePixels,frameWidth,frameHeight)) return false;
    cameraSettings=settings;return true;
  }
  return initCamera(settings);
}
bool captureFrame() {
  if(!cameraReady) return false;
  ImageSource::Frame frame;
  if(!imageSource.capture(framePixels,frameWidth,frameHeight,frame)) { ++captureFailures;return false; }
  frameNumber=frame.sequence;frameCapturedAt=frame.capturedAtMs;lastCapture=frameCapturedAt;return true;
}

constexpr uint16_t STATE_QUEUE_CAPACITY=400;
ScorePending *stateQueue=nullptr;
uint16_t stateHead=0,stateCount=0;
bool stateBitmapDirty=false,scoreReportingEnabled=false;
uint32_t lastStateBitmap=0;
constexpr uint32_t STATE_BITMAP_REFRESH_MS=1000;
void queueState(uint32_t id,bool grouped,uint16_t score) {
  stateBitmapDirty=true;
  if(!stateQueue || !scoreReportingEnabled || grouped) return;
  ScorePending value={id,score};
  for(uint16_t i=0;i<stateCount;++i) {
    ScorePending &slot=stateQueue[(stateHead+i)%STATE_QUEUE_CAPACITY];
    if(slot.id==id) { slot=value;return; }
  }
  if(stateCount==STATE_QUEUE_CAPACITY) {
    DEBUGF("state queue full; state %lu will be recovered by refresh\n",(unsigned long)id);
    return;
  }
  stateQueue[(stateHead+stateCount)%STATE_QUEUE_CAPACITY]=value;
  ++stateCount;
}
void sendStateBitmap() {
  if(!bridgeKnown || (!stateBitmapDirty && millis()-lastStateBitmap<STATE_BITMAP_REFRESH_MS)) return;
  StateBitmapPayload bitmap={};bitmap.revision=configRevision;bitmap.frame=frameNumber;
  bitmap.count=cellCount;bitmap.encoding=1;
  for(uint16_t i=0;i<cellCount;++i)
    bitmap.states[i/4]|=(cells[i].state&3)<<((i%4)*2);
  const uint32_t seq=nextSeq++;
  const size_t bytes=offsetof(StateBitmapPayload,states)+(cellCount*2+7)/8;
  const bool accepted=transmit(bridgeMac,STATE_BITMAP,seq,&bitmap,bytes);
  if(accepted) { stateBitmapDirty=false;lastStateBitmap=millis(); }
}
void sendScores() {
  if(!bridgeKnown || !scoreReportingEnabled || !stateCount) return;
  ScoreBatchPayload batch={};batch.revision=configRevision;batch.frame=frameNumber;
  const uint16_t take=min(stateCount,(uint16_t)MAX_BATCHED_SCORES);
  for(uint16_t offset=0;offset<take;++offset) {
    const ScorePending &value=stateQueue[(stateHead+offset)%STATE_QUEUE_CAPACITY];
    int index=-1;for(uint16_t i=0;i<cellCount;++i) if(cells[i].config.id==value.id) { index=i;break; }
    if(index>=0) batch.records[batch.count++]={(uint16_t)index,value.scorePermille};
  }
  if(!batch.count) { stateHead=(stateHead+take)%STATE_QUEUE_CAPACITY;stateCount-=take;return; }
  const size_t bytes=offsetof(ScoreBatchPayload,records)+(size_t)batch.count*sizeof(ScoreRecord);
  if(transmit(bridgeMac,SCORE_BATCH,nextSeq++,&batch,bytes)) {
    stateHead=(stateHead+take)%STATE_QUEUE_CAPACITY;stateCount-=take;
  }
}
void queueAllStates() {
  stateBitmapDirty=true;
  for(uint16_t i=0;i<cellCount;++i)
    queueState(cells[i].config.id,false,cells[i].scorePermille);
}

#include "OccupancyDetector.h"
OccupancyDetector detector;

void baselineHeartbeat() {
  if(millis()-lastHello>=1000) { sendHello();lastHello=millis();delay(2); }
  if(millis()-lastHealth>=5000) { sendHealth();lastHealth=millis();delay(2); }
}
bool captureBaseline() {
  if(calibration.active) return false;
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

// Preserve the saved visual baseline and move selected empty sensors above the
// worst mismatch observed under the current lighting. A zero target selects
// cells that are occupied when the operation starts.
bool retuneLighting(const CalibrationRequestPayload &request) {
  if(!configured || !baselineReady || snapshot.active || !cellCount ||
     request.revision!=configRevision+1 || request.durationMs<2000 ||
     request.durationMs>30000 || !request.maxSamples) return false;
  struct Stats { uint16_t maximum=0,samples=0;bool target=false; };
  Stats *stats=(Stats *)ps_malloc((size_t)cellCount*sizeof(Stats));
  CellRuntime *previous=(CellRuntime *)ps_malloc((size_t)cellCount*sizeof(CellRuntime));
  if(!stats || !previous) { free(stats);free(previous);return false; }
  memset(stats,0,(size_t)cellCount*sizeof(Stats));
  memcpy(previous,cells,(size_t)cellCount*sizeof(CellRuntime));
  uint16_t targets=0;
  for(uint16_t i=0;i<cellCount;++i) {
    stats[i].target=request.targetId ? cells[i].config.id==request.targetId : cells[i].state==OCCUPIED;
    if(stats[i].target) ++targets;
  }
  if(!targets) { free(stats);free(previous);return false; }
  const uint32_t started=millis(),interval=max((uint32_t)1,request.durationMs/request.maxSamples);
  uint32_t nextSample=started;
  detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,nullptr);
  while(millis()-started<request.durationMs) {
    baselineHeartbeat();
    if((int32_t)(millis()-nextSample)<0) { delay(1);continue; }
    nextSample+=interval;
    if(!captureFrame()) continue;
    for(uint16_t i=0;i<cellCount;++i) if(stats[i].target) {
      Feature live={};uint16_t buckets[SPATIAL_BUCKETS]={};
      stats[i].maximum=max(stats[i].maximum,detector.inspectCell(cells[i],live,buckets));
      ++stats[i].samples;
      if((i&15)==15) baselineHeartbeat();
    }
  }
  CalibrationResultPayload *results=(CalibrationResultPayload *)ps_malloc(
    (size_t)cellCount*sizeof(CalibrationResultPayload));
  if(!results) { free(stats);free(previous);return false; }
  for(uint16_t i=0;i<cellCount;++i) {
    uint8_t flags=0x80;
    if(stats[i].target) {
      flags|=0x20;
      // The sampled scene must also sit below the fixed 70% clearing boundary,
      // otherwise the sensor clears now but sticks after its next real trigger.
      const uint32_t occupySafe=(uint32_t)stats[i].maximum+100;
      const uint32_t clearSafe=((uint32_t)stats[i].maximum+20)*10/7+1;
      const uint32_t required=max(occupySafe,clearSafe);
      const uint16_t proposed=(uint16_t)(((required+9)/10)*10);
      if(stats[i].samples<3 || required>1000) flags|=0x40;
      else {
        cells[i].config.thresholdPermille=max(cells[i].config.thresholdPermille,proposed);
        cells[i].state=CLEAR;cells[i].enterCount=cells[i].clearCount=0;cells[i].scorePermille=0;
      }
    }
    results[i]={request.revision,cells[i].config.id,i,cellCount,
      cells[i].config.thresholdPermille,stats[i].maximum,cells[i].reference.edges,
      stats[i].samples,cells[i].config.radius,(uint8_t)(flags|(stats[i].samples>=40?2:stats[i].samples>=15?1:0))};
  }
  const uint32_t previousRevision=configRevision;configRevision=request.revision;
  if(!saveBaseline()) {
    configRevision=previousRevision;memcpy(cells,previous,(size_t)cellCount*sizeof(CellRuntime));
    free(results);free(stats);free(previous);return false;
  }
  for(uint16_t g=0;g<groupCount;++g) {
    groups[g].state=CLEAR;
    for(uint16_t i=0;i<cellCount;++i) if(cells[i].config.groupId==groups[g].id && cells[i].state==OCCUPIED) groups[g].state=OCCUPIED;
  }
  free(calibration.results);calibration=CalibrationTransfer();
  calibration.active=true;calibration.results=results;calibration.count=cellCount;
  calibration.revision=configRevision;calibration.seq=nextSeq++;
  detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
  queueAllStates();sendHello();lastHello=millis();
  free(stats);free(previous);
  DEBUGF("lighting retune revision=%lu targets=%u duration_ms=%lu\n",
    (unsigned long)configRevision,targets,(unsigned long)(millis()-started));
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
  CalibrationResultPayload *results=(CalibrationResultPayload *)ps_malloc(
    (size_t)cellCount*sizeof(CalibrationResultPayload));
  if(!results) { free(trials);free(stats);free(meanPixels);return false; }
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
      results[i]={configRevision,cells[i].config.id,i,cellCount,
        cells[i].config.thresholdPermille,s.maximum,cells[i].reference.edges,s.samples,
        cells[i].config.radius,(uint8_t)(s.samples>=40?2:s.samples>=15?1:0)};
    }
    free(calibration.results);calibration=CalibrationTransfer();
    calibration.active=true;calibration.results=results;calibration.count=cellCount;
    calibration.revision=configRevision;calibration.seq=nextSeq++;
    queueAllStates();sendHello();lastHello=millis();
  } else {
    free(results);
    memcpy(cells,staging,(size_t)cellCount*sizeof(CellRuntime));
    configRevision=previousRevision;baselineReady=previousBaselineReady;
  }
  free(trials);free(stats);free(meanPixels);
  detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
  DEBUGF("auto calibration revision=%lu cells=%u samples=%u duration_ms=%lu result=%u\n",
    (unsigned long)configRevision,cellCount,sampleCount,(unsigned long)(millis()-started),baselineReady);
  return baselineReady;
}

void sendCalibrationResult() {
  if(!calibration.active || !bridgeKnown || calibration.index>=calibration.count) return;
  transmit(bridgeMac,CALIBRATION_RESULT,calibration.seq,
    &calibration.results[calibration.index],sizeof(CalibrationResultPayload));
  calibration.sentAt=millis();
}
void serviceCalibration() {
  if(!calibration.active) return;
  if(!calibration.sentAt) { sendCalibrationResult();return; }
  if(millis()-calibration.sentAt<500) return;
  if(calibration.retries<255) ++calibration.retries;
  sendCalibrationResult();
}
void acceptCalibrationAck(const Packet &p) {
  uint32_t revision=0;
  if(!calibration.active || p.length!=sizeof(revision) || p.seq!=calibration.seq) return;
  memcpy(&revision,p.payload,sizeof(revision));
  if(revision!=calibration.revision) return;
  ++calibration.index;calibration.retries=0;calibration.sentAt=0;
  if(calibration.index<calibration.count) calibration.seq=nextSeq++;
  else {
    free(calibration.results);calibration=CalibrationTransfer();
    DEBUGLN("calibration results acknowledged");
  }
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
    c.enterFrames>=1 && c.clearFrames>=1;
}
uint8_t beginConfig(const ConfigBeginPayload &value) {
  if(snapshot.active || calibration.active) return ACK_BUSY;
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
    if(!applyCameraSettings(stagingSettings)) {
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
  BaselineHeader h={0x52424C3C,configRevision,0,
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
  if(f.read((uint8_t *)&h,sizeof(h))!=sizeof(h) || h.magic!=0x52424C3C ||
     h.count>MAX_CELLS || h.settings.resolution>XGA) { f.close();return; }
  const size_t cellsBytes=(size_t)h.count*sizeof(CellRuntime);
  if(f.size()!=sizeof(h)+cellsBytes+h.bytes || !initCamera(h.settings) ||
     (h.bytes && h.bytes!=(uint32_t)frameWidth*frameHeight)) { f.close();return; }
  bool ok=f.read((uint8_t *)cells,cellsBytes)==cellsBytes &&
    crc32((const uint8_t *)cells,cellsBytes)==h.cellsCrc;
  if(ok && h.bytes) {
    uint8_t *legacyPixels=(uint8_t *)ps_malloc(h.bytes);
    ok=legacyPixels && f.read(legacyPixels,h.bytes)==h.bytes &&
      crc32(legacyPixels,h.bytes)==h.imageCrc;
    free(legacyPixels);
  }
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
  if(!imageSource.pause()) return false;
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
    free(snapshot.encoded);snapshot.encoded=nullptr;snapshot.active=false;imageSource.resume();return;
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
    free(snapshot.encoded);snapshot.encoded=nullptr;snapshot.active=false;imageSource.resume();
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
  // A packet can arrive in the Wi-Fi callback just after this loop drained the
  // inbox, followed by the channel scanner advancing before the packet is
  // handled on the next iteration. Return to the channel on which a known
  // bridge packet was actually received before replying or starting a transfer.
  if(bridgeKnown && sameMac(message.mac,bridgeMac) && message.channel &&
     message.channel!=radioChannel) setChannel(message.channel);
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
      const bool scoresRequested=beacon.baselineReady!=0;
      scoreReportingEnabled=scoresRequested;
      if(!scoreReportingEnabled) stateHead=stateCount=0;
      if(lastBridgeBeacon && now-lastBridgeBeacon>=BRIDGE_SEARCH_AFTER_MS) queueDiagnostic(DIAG_BRIDGE_REJOIN,now-lastBridgeBeacon);
      lastBridgeBeacon=now;lastBridgeBeaconSeq=p.seq;
      if(newBridge || missedBridge || bridgeRestarted) {
        if((bridgeRestarted || missedBridge) && calibration.active) {
          calibration.index=0;calibration.retries=0;calibration.sentAt=0;
          calibration.seq=nextSeq++;
        }
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
  if(p.type==CALIBRATION_ACK) { acceptCalibrationAck(p);return; }
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
      lastBaselineStatus=configured && !snapshot.active && !calibration.active
        ? (captureBaseline()?ACK_OK:ACK_CAMERA_ERROR) : ACK_BAD_ORDER;
      lastBaselineSeq=p.seq;
    }
    sendAck(message.mac,p.seq,p.type,lastBaselineStatus);
  } else if(p.type==SNAPSHOT_REQUEST) {
    if(p.length!=0) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    if(snapshot.active) {
      sendAck(message.mac,p.seq,p.type,p.seq==snapshot.requestSeq?ACK_OK:ACK_BUSY);
    } else if(calibration.active) {
      sendAck(message.mac,p.seq,p.type,ACK_BUSY);
    } else {
      const bool ready=framePixels && frameNumber && startSnapshot(p.seq);
      sendAck(message.mac,p.seq,p.type,ready?ACK_OK:ACK_BUSY);
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
    value.peakCount=cells[index].reference.textured?FIXED_DIRECTIONS:0;
    for(uint8_t peak=0;peak<value.peakCount;++peak)
      value.lineAngleTenths[peak]=peak*150;
    memcpy(value.referenceBuckets,cells[index].referenceBuckets,sizeof(value.referenceBuckets));
    memcpy(value.liveBuckets,liveBuckets,sizeof(value.liveBuckets));
    transmit(bridgeMac,CELL_ANALYSIS,p.seq,&value,sizeof(value));
  } else if(p.type==CALIBRATE_REQUEST) {
    CalibrationRequestPayload value={};
    if(p.length!=sizeof(value)) { sendAck(message.mac,p.seq,p.type,ACK_BAD_PAYLOAD);return; }
    memcpy(&value,p.payload,sizeof(value));
    if(!configured || snapshot.active || calibration.active || value.revision!=configRevision+1 || value.mode>RETUNE_LIGHTING) { sendAck(message.mac,p.seq,p.type,ACK_BAD_ORDER);return; }
    sendAck(message.mac,p.seq,p.type,ACK_OK);
    if(value.mode==RETUNE_LIGHTING) retuneLighting(value); else autoCalibrate(value);
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
    DEBUGF("[%u] id=%lu group=%lu (%u,%u) r=%u %s directions=%u score=%u state=%u\n",
      i,(unsigned long)c.config.id,(unsigned long)c.config.groupId,c.config.x,c.config.y,
      c.config.radius,c.config.shape?"square":"circle",
      c.reference.textured?FIXED_DIRECTIONS:0,c.scorePermille,c.state);
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
    DEBUGLN("H help | I info | P MAC pair | C channel | B revision count resolution [brightness contrast saturation vflip hmirror] | S index id group x y radius C/S contrast threshold_permille enter clear | A commit | R baseline | F raw frame | X clear RAM config | Z FORMAT LittleFS");
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
    unsigned index,x,y,radius,contrast,threshold,enter,clear;
    unsigned long id,group;char shape;
    if(args && sscanf(args,"%u %lu %lu %u %u %u %c %u %u %u %u",
       &index,&id,&group,&x,&y,&radius,&shape,&contrast,&threshold,&enter,&clear)==11 &&
       index<=MAX_CELLS && id<=UINT32_MAX && group<=UINT32_MAX && x<=UINT16_MAX &&
       y<=UINT16_MAX && radius<=UINT8_MAX && (shape=='C' || shape=='S') &&
       contrast<=UINT16_MAX && threshold<=UINT16_MAX &&
       enter<=UINT8_MAX && clear<=UINT8_MAX) {
      ConfigCellPayload value={};value.index=index;
      value.cell={(uint32_t)id,(uint32_t)group,(uint16_t)x,(uint16_t)y,
        (uint8_t)radius,(uint8_t)(shape=='S'?1:0),(uint16_t)contrast,
        (uint16_t)threshold,(uint8_t)enter,(uint8_t)clear};
      DEBUGF("cell status=%u\n",stageCell(value));
    } else DEBUGLN("S syntax error");
  } else if(strcmp(command,"A")==0) DEBUGF("commit status=%u\n",commitConfig(stagingRevision));
  else if(strcmp(command,"R")==0) DEBUGLN(configured && captureBaseline()?"baseline OK":"baseline failed");
  else if(strcmp(command,"F")==0) serialSnapshot();
  else if(strcmp(command,"X")==0 && !calibration.active) {
    cellCount=groupCount=0;baselineReady=configured=stagingOpen=false;
    stateHead=stateCount=0;DEBUGLN("configuration cleared from RAM");
  } else if(strcmp(command,"X")==0) DEBUGLN("configuration busy sending calibration results");
  else if(strcmp(command,"Z")==0) {
    char *confirmation=strtok(nullptr," \t");
    if(!confirmation || strcmp(confirmation,"FORMAT")!=0 || snapshot.active || calibration.active || stagingOpen) {
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
  stateQueue=(ScorePending *)ps_malloc(sizeof(ScorePending)*STATE_QUEUE_CAPACITY);
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
  // A saved baseline knows the required resolution and initializes the camera
  // directly at that size. Initializing QVGA first and immediately tearing the
  // driver down for the saved size made cold boot timing-dependent.
  loadBaseline();
  if(!cameraReady) initCamera(cameraSettings);
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
  serviceCalibration();
  serviceDiagnostics();
  // Submit state first. ESP-NOW continues radio work while the next frame is
  // captured; optional score diagnostics never delay capture.
  sendStateBitmap();sendScores();
  if(cameraReady && !snapshot.active) {
    if(captureFrame()) {
      detector.bind(framePixels,frameWidth,frameHeight,cells,cellCount,groups,groupCount,queueState);
      if(baselineReady) { detector.analyseAllCells();++analysisCount; }
    }
  }
  sendStateBitmap();sendScores();
  if(millis()-lastHello>=HELLO_MS) { lastHello=millis();sendHello(); }
  if(millis()-lastHealth>=HEALTH_MS) { lastHealth=millis();sendHealth(); }
  // A router channel change can move the bridge. Search until its beacon reappears.
  if(!snapshot.active && millis()-lastBridgeBeacon>BRIDGE_SEARCH_AFTER_MS && millis()-lastChannelScan>=CHANNEL_DWELL_MS) {
    lastChannelScan=millis();
    setChannel(radioChannel==13?1:radioChannel+1);
  }
  // Keep live tuning useful even when a sensor's state does not change. State
  // transitions are immediate; this refresh supplies a recent supporting score.
  if(millis()-lastFullStateRefresh>=5000) { lastFullStateRefresh=millis();queueAllStates(); }
  yield();
}
