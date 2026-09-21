#define BRIDGE_VERSION "0.1.9-created-revisions"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <esp_partition.h>
#include <stddef.h>

// Keep these packed declarations in step with Camera_Module.ino protocol v1.
constexpr uint16_t MAGIC=0x5243;
constexpr uint8_t VERSION=1, START_CHANNEL=1, MAX_CAMERAS=8; // Conservative ESP32-C3 RAM budget.
constexpr uint16_t MAX_CELLS=300;
constexpr size_t PAYLOAD=200;
constexpr uint32_t ACK_TIMEOUT=1500, CAMERA_TIMEOUT=15000;
const uint8_t BROADCAST[6]={255,255,255,255,255,255};
enum Type:uint8_t { HELLO=1,CONFIG_BEGIN=2,CONFIG_CELL=3,CONFIG_COMMIT=4,
  CAPTURE_BASELINE=5,SNAPSHOT_REQUEST=6,ACK=7,STATE=8,HEALTH=9,
  SNAPSHOT_BEGIN=10,SNAPSHOT_CHUNK=11,SNAPSHOT_END=12,SNAPSHOT_ACK=13,DIAGNOSTIC=14,DIAGNOSTIC_ACK=15,HEALTH_ACK=16,
  ANALYSIS_REQUEST=17,CELL_ANALYSIS=18,CALIBRATE_REQUEST=19,CALIBRATION_RESULT=20,CALIBRATION_ACK=21 };
enum CellState:uint8_t { UNKNOWN=0,CLEAR=1,OCCUPIED=2 };
struct __attribute__((packed)) Packet { uint16_t magic; uint8_t version,type; uint32_t seq; uint16_t length; uint8_t payload[PAYLOAD]; };
struct __attribute__((packed)) CameraSettings { uint8_t resolution; int8_t brightness,contrast,saturation; uint8_t vflip,hmirror; };
struct __attribute__((packed)) Begin { uint32_t revision; uint16_t count; CameraSettings settings; };
struct __attribute__((packed)) Cell { uint32_t id,group; uint16_t x,y; uint8_t radius,shape; uint16_t floor,threshold; uint8_t tolerance,enter,clear; uint32_t createdRevision; };
struct __attribute__((packed)) CellMessage { uint16_t index; Cell cell; };
struct __attribute__((packed)) HelloMessage { uint8_t mac[6]; uint32_t revision; uint16_t width,height,count; uint8_t channel,baseline; };
struct __attribute__((packed)) AckMessage { uint8_t type,status; uint16_t detail; };
struct __attribute__((packed)) StateMessage { uint32_t id,revision,frame; uint16_t score; uint8_t state,grouped; };
struct __attribute__((packed)) AnalysisMessage {
  uint32_t id,revision,frame;uint16_t score,threshold,referenceEdges,liveEdges;uint8_t peakCount;
  uint16_t angles[10],referenceBuckets[31],liveBuckets[31];
};
struct __attribute__((packed)) CalibrationRequest { uint32_t revision,durationMs,sinceRevision;uint8_t maxSamples; };
struct __attribute__((packed)) CalibrationResult {
  uint32_t revision,id;uint16_t index,count,threshold,clearMaximum,referenceEdges,samples;
  uint8_t radius,confidence;
};
struct __attribute__((packed)) HealthMessage { uint32_t revision,frame,failures,heap; uint16_t cells,width,height,age; uint8_t baseline,snapshot; };
struct __attribute__((packed)) SnapshotBegin { uint32_t frame,bytes,crc; uint16_t width,height; };
struct __attribute__((packed)) CompressedSnapshotBegin { SnapshotBegin raw; uint32_t wireBytes,wireCrc; uint8_t codec; };
static_assert(sizeof(SnapshotBegin)==16 && sizeof(CompressedSnapshotBegin)==25,"Snapshot header layout");
struct __attribute__((packed)) SnapshotEnd { uint32_t frame,crc; };
struct __attribute__((packed)) DiagnosticPayload {
  uint32_t bootId,eventSeq,uptimeMs,frame,detail,offset;
  uint16_t captureFailures,dropped;
  uint8_t event,resetReason,channel,operation,outcome;
};
struct __attribute__((packed)) FileHeader { uint32_t magic,revision,lastAutoSizeRevision; uint16_t count; CameraSettings settings; uint32_t crc; };
struct __attribute__((packed)) LegacyFileHeader { uint32_t magic,revision; uint16_t count; CameraSettings settings; uint32_t crc; };
struct __attribute__((packed)) LegacyCell { uint32_t id,group; uint16_t x,y; uint8_t radius,shape; uint16_t floor,threshold; uint8_t tolerance,enter,clear; };
static_assert(sizeof(Packet)==210 && sizeof(Cell)==25 && sizeof(HealthMessage)==26,"wire layout mismatch");
static_assert(sizeof(AnalysisMessage)<=PAYLOAD,"analysis wire layout");

struct Camera {
  bool used=false,seen=false,upload=false;
  uint8_t mac[6]={};
  uint32_t revision=0,remoteRevision=0,lastSeen=0,lastHelloSeq=0,lastStateSeq=0;
  uint16_t count=0,received=0;
  CameraSettings settings={};
  Cell *cells=nullptr;
  uint8_t *states=nullptr;
  uint8_t *cellStates=nullptr;
  Cell *staged=nullptr;
  uint32_t stagedRevision=0;
  uint16_t stagedCount=0;
  CameraSettings stagedSettings={};
  uint8_t phase=0,retries=0;
  uint16_t uploadIndex=0;
  uint32_t pendingSeq=0,pendingAt=0;
  uint8_t pendingType=0;
  Packet pending={};
  bool baseline=false,snapshot=false;
  uint32_t snapFrame=0,snapBytes=0,snapCrc=0,snapRawBytes=0,snapRawCrc=0,snapOffset=0,snapRunning=0xFFFFFFFF,snapLastAt=0;
  uint8_t snapCodec=0;
  uint32_t lastSnapEndSeq=0;
  uint32_t diagBootId=0,diagEventSeq=0;
  uint32_t calibrationRevision=0;
  uint32_t lastAutoSizeRevision=0;
  uint16_t calibrationReceived=0;
  uint8_t calibrationSeen[(MAX_CELLS+7)/8]={};
  uint16_t snapWidth=0,snapHeight=0;
};
Camera cameras[MAX_CAMERAS];
Preferences prefs;
WiFiClient net;
PubSubClient mqtt(net);
String ssid,password,broker,topic="railway/home",mqttUser,mqttPass;
bool wifiPending=false,mqttPending=false,lastWifiConnected=false,lastMqttConnected=false;
uint32_t wifiPendingAt=0,mqttPendingAt=0;
uint16_t brokerPort=1883;
uint32_t sequence=1,lastMqttAttempt=0,lastWifiAttempt=0;
constexpr uint32_t MQTT_RETRY_MS=30000;
constexpr uint32_t MQTT_EARLY_RETRY_MS=10000;
uint8_t mqttStartupFailures=0;
uint8_t radioChannel=START_CHANNEL;
uint32_t lastBeacon=0,fastBeaconUntil=0,lastBeaconFailureLog=0;
constexpr uint32_t FAST_BEACON_MS=250,NORMAL_BEACON_MS=500,FAST_BEACON_WINDOW_MS=20000;
QueueHandle_t inbox=nullptr;
struct Received { uint8_t mac[6]; Packet packet; };
char line[1100]; size_t lineLength=0;
char diagnosticLog[64][192]={};
uint8_t diagnosticHead=0,diagnosticCount=0;
uint32_t bridgeBootId=0,diagnosticSequence=1;
void recordDiagnostic(const char *code,const char *mac,const char *details="-") {
  char row[192];
  snprintf(row,sizeof(row),"DIAG %08lX %lu %lu %s %s %s",(unsigned long)bridgeBootId,
    (unsigned long)diagnosticSequence++,(unsigned long)millis(),code,mac,details);
  uint8_t index=(diagnosticHead+diagnosticCount)%64;
  if(diagnosticCount==64) { index=diagnosticHead;diagnosticHead=(diagnosticHead+1)%64; }
  else ++diagnosticCount;
  strncpy(diagnosticLog[index],row,sizeof(diagnosticLog[index])-1);
  diagnosticLog[index][sizeof(diagnosticLog[index])-1]=0;
  Serial.print("EVENT ");Serial.println(row);
}
const char *cameraDiagnosticName(uint8_t event) {
  switch(event) {
    case 1:return "CAMERA_BOOT";case 2:return "CAMERA_JOIN";case 3:return "CAMERA_REJOIN";
    case 4:return "CAMERA_FRAME_START";case 5:return "CAMERA_FRAME_DONE";
    case 6:return "CAMERA_SEND_FAILED";case 7:return "CAMERA_FRAME_ABORTED";
    case 8:return "CAMERA_BASELINE_START";case 9:return "CAMERA_BASELINE_DONE";
    case 10:return "CAMERA_BASELINE_FAILED";case 11:return "CAMERA_READY";
    case 12:return "CAMERA_INIT_FAILED";case 13:return "CAMERA_CONFIG_BEGIN";
    case 14:return "CAMERA_CONFIG_ACK_FAILED";default:return "CAMERA_EVENT";
  }
}

uint32_t crcStep(uint32_t crc,const uint8_t *data,size_t length) {
  for(size_t i=0;i<length;++i) { crc^=data[i];for(int b=0;b<8;++b) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0); }
  return crc;
}
void macText(const uint8_t *mac,char *out) { sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]); }
bool parseMac(const char *s,uint8_t *mac) {
  if(strlen(s)!=17 || s[2]!=':' || s[5]!=':' || s[8]!=':' || s[11]!=':' || s[14]!=':') return false;
  unsigned v[6];int used=0;
  if(sscanf(s,"%2x:%2x:%2x:%2x:%2x:%2x%n",v,v+1,v+2,v+3,v+4,v+5,&used)!=6 || s[used]) return false;
  bool any=false;for(int i=0;i<6;++i) { mac[i]=v[i];if(mac[i]) any=true; }
  return any;
}
Camera *findCamera(const uint8_t *mac) { for(auto &c:cameras) if(c.used && !memcmp(c.mac,mac,6)) return &c;return nullptr; }
Camera *cameraFor(const char *text) { uint8_t mac[6];return parseMac(text,mac)?findCamera(mac):nullptr; }
String pathFor(const uint8_t *mac) { char s[18];macText(mac,s);String p="/c";for(int i=0;i<17;++i) if(s[i]!=':') p+=(char)tolower(s[i]);return p; }
String topicPath(uint32_t id) { return "/t"+String(id); }
String areaName(uint32_t id) {
  File f=LittleFS.open(topicPath(id),"r");
  if(!f) return String(id);
  String name=f.readString();f.close();return name.length()?name:String(id);
}
String areaSuffix(uint32_t id) { return "/areas/"+areaName(id)+"/state"; }
bool addPeer(const uint8_t *mac) {
  if(esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,mac,6);peer.channel=0;peer.ifidx=WIFI_IF_STA;
  return esp_now_add_peer(&peer)==ESP_OK;
}
bool sendPacket(const uint8_t *mac,const Packet &p) {
  return addPeer(mac) && esp_now_send(mac,(const uint8_t *)&p,offsetof(Packet,payload)+p.length)==ESP_OK;
}
Packet makePacket(uint8_t type,uint32_t seq,const void *data,size_t length) {
  Packet p={};p.magic=MAGIC;p.version=VERSION;p.type=type;p.seq=seq;p.length=length;
  if(length) memcpy(p.payload,data,length);return p;
}
void broadcastBeacon() {
  uint8_t mac[6]={};esp_wifi_get_mac(WIFI_IF_STA,mac);
  HelloMessage beacon={};memcpy(beacon.mac,mac,6);beacon.channel=radioChannel;
  Packet p=makePacket(HELLO,sequence++,&beacon,sizeof(beacon));
  if(!sendPacket(BROADCAST,p) && millis()-lastBeaconFailureLog>=5000) {
    lastBeaconFailureLog=millis();
    char detail[48];snprintf(detail,sizeof(detail),"channel=%u",radioChannel);
    recordDiagnostic("BEACON_SEND_FAILED","-",detail);
  }
  lastBeacon=millis();
}
void sendSnapshotAck(Camera &c,uint32_t seq) { Packet p=makePacket(SNAPSHOT_ACK,seq,nullptr,0);sendPacket(c.mac,p); }
void publish(const String &suffix,const String &value,bool retained=true) {
  if(mqtt.connected()) mqtt.publish((topic+suffix).c_str(),value.c_str(),retained);
  Serial.printf("EVENT MQTT %s %s\n",(topic+suffix).c_str(),value.c_str());
}
void clearArea(const String &name) {
  String full=topic+"/areas/"+name+"/state";
  if(mqtt.connected()) mqtt.publish(full.c_str(),"",true); // Empty retained payload deletes the broker's saved topic.
  Serial.printf("EVENT MQTT_REMOVED %s\n",full.c_str());
}
bool activeAreaName(const String &name) {
  for(const auto &c:cameras) if(c.used) for(uint16_t i=0;i<c.count;++i) {
    uint32_t id=c.cells[i].group?c.cells[i].group:c.cells[i].id;
    if(areaName(id)==name) return true;
  }
  return false;
}
void onMqttMessage(char *receivedTopic,uint8_t *payload,unsigned int length) {
  String prefix=topic+"/areas/",full(receivedTopic);
  if(!full.startsWith(prefix) || !full.endsWith("/state") || length==0) return;
  String name=full.substring(prefix.length(),full.length()-6);
  if(!name.length() || name.length()>32 || name.indexOf('/')>=0 || activeAreaName(name)) return;
  clearArea(name);
}
const char *stateName(uint8_t state) { return state==CLEAR?"clear":state==OCCUPIED?"occupied":"unknown"; }
bool isReported(const Camera &c,uint32_t id) {
  for(uint16_t i=0;i<c.count;++i) if(c.cells[i].group?c.cells[i].group==id:c.cells[i].id==id) return true;
  return false;
}
int cellIndex(const Camera &c,uint32_t id) {
  for(uint16_t i=0;i<c.count;++i) if(c.cells[i].id==id) return i;
  return -1;
}
void allUnknown(Camera &c) {
  char mac[18];macText(c.mac,mac);
  for(uint16_t i=0;i<c.count;++i) {
    uint32_t id=c.cells[i].group?c.cells[i].group:c.cells[i].id;
    bool first=true;for(uint16_t j=0;j<i;++j) if((c.cells[j].group?c.cells[j].group:c.cells[j].id)==id) first=false;
    if(c.states) c.states[i]=UNKNOWN;
    if(c.cellStates) c.cellStates[i]=UNKNOWN;
    if(first) publish(areaSuffix(id),"unknown");
  }
  Serial.printf("EVENT CAMERA %s unknown\n",mac);
}
void publishSnapshot(Camera &c) {
  for(uint16_t i=0;i<c.count;++i) {
    uint32_t id=c.cells[i].group?c.cells[i].group:c.cells[i].id;
    bool first=true;for(uint16_t j=0;j<i;++j) if((c.cells[j].group?c.cells[j].group:c.cells[j].id)==id) first=false;
    if(first) publish(areaSuffix(id),
      c.seen && c.baseline && c.states?stateName(c.states[i]):"unknown");
  }
}
bool validCell(const Cell &x,const CameraSettings &s) {
  const uint16_t w[]={320,640,800,1024},h[]={240,480,600,768};
  return s.resolution<4 && x.id && x.x<w[s.resolution] && x.y<h[s.resolution] &&
    x.radius>=3 && x.radius<=50 && x.shape<=1 && x.floor>=10 && x.floor<=500 &&
    x.threshold>=50 && x.threshold<=1000 && x.tolerance<=20 && x.enter && x.clear;
}
bool validSettings(const CameraSettings &s) {
  return s.resolution<4 && s.brightness>=-2 && s.brightness<=2 && s.contrast>=-2 && s.contrast<=2 &&
    s.saturation>=-2 && s.saturation<=2 && s.vflip<=1 && s.hmirror<=1;
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
bool save(Camera &c) {
  String tmp=pathFor(c.mac)+".tmp",path=pathFor(c.mac);
  File f=LittleFS.open(tmp,"w");if(!f) return false;
  FileHeader h={0x52434632,c.revision,c.lastAutoSizeRevision,c.count,c.settings,0};
  h.crc=~crcStep(0xFFFFFFFF,(const uint8_t *)c.cells,c.count*sizeof(Cell));
  bool ok=f.write((const uint8_t *)&h,sizeof(h))==sizeof(h) &&
    f.write((const uint8_t *)c.cells,c.count*sizeof(Cell))==c.count*sizeof(Cell);
  f.close();if(!ok) { LittleFS.remove(tmp);return false; }
  String backup=path+".bak";LittleFS.remove(backup);
  if(LittleFS.exists(path) && !LittleFS.rename(path,backup)) { LittleFS.remove(tmp);return false; }
  if(!LittleFS.rename(tmp,path)) { if(LittleFS.exists(backup)) LittleFS.rename(backup,path);return false; }
  LittleFS.remove(backup);return true;
}
bool saveStaged(Camera &c) {
  Cell *oldCells=c.cells;uint16_t oldCount=c.count;uint32_t oldRevision=c.revision;CameraSettings oldSettings=c.settings;
  c.cells=c.staged;c.count=c.stagedCount;c.revision=c.stagedRevision;c.settings=c.stagedSettings;
  if(save(c)) {
    free(c.states);c.states=(uint8_t *)malloc(c.count?c.count:1);
    if(c.states) memset(c.states,UNKNOWN,c.count);
    free(c.cellStates);c.cellStates=(uint8_t *)malloc(c.count?c.count:1);
    if(c.cellStates) memset(c.cellStates,UNKNOWN,c.count);
    for(uint16_t i=0;i<oldCount;++i) {
      uint32_t id=oldCells[i].group?oldCells[i].group:oldCells[i].id;
      bool earlier=false,still=false;
      for(uint16_t j=0;j<i;++j) if((oldCells[j].group?oldCells[j].group:oldCells[j].id)==id) earlier=true;
      for(uint16_t j=0;j<c.count;++j) if((c.cells[j].group?c.cells[j].group:c.cells[j].id)==id) still=true;
      if(!earlier && !still) { clearArea(areaName(id));LittleFS.remove(topicPath(id)); }
    }
    free(oldCells);c.staged=nullptr;return true;
  }
  c.cells=oldCells;c.count=oldCount;c.revision=oldRevision;c.settings=oldSettings;return false;
}
bool load(Camera &c,const String &path) {
  File f=LittleFS.open(path,"r");if(!f) return false;
  uint32_t magic=0;if(f.read((uint8_t *)&magic,sizeof(magic))!=sizeof(magic)) { f.close();return false; }f.seek(0);
  if(magic==0x52434631) {
    LegacyFileHeader h={};bool ok=f.read((uint8_t *)&h,sizeof(h))==sizeof(h) && h.count<=MAX_CELLS &&
      validSettings(h.settings) && f.size()==sizeof(h)+h.count*sizeof(LegacyCell);
    LegacyCell *old=ok?(LegacyCell *)malloc((h.count?h.count:1)*sizeof(LegacyCell)):nullptr;
    if(ok) ok=old && f.read((uint8_t *)old,h.count*sizeof(LegacyCell))==h.count*sizeof(LegacyCell) &&
      ~crcStep(0xFFFFFFFF,(uint8_t *)old,h.count*sizeof(LegacyCell))==h.crc;
    Cell *items=ok?(Cell *)malloc((h.count?h.count:1)*sizeof(Cell)):nullptr;
    if(ok) ok=items;
    if(ok) for(uint16_t i=0;i<h.count;++i) {
      const LegacyCell &x=old[i];items[i]={x.id,x.group,x.x,x.y,x.radius,x.shape,x.floor,x.threshold,x.tolerance,x.enter,x.clear,h.revision};
      if(!validCell(items[i],h.settings)) ok=false;
    }
    free(old);f.close();
    if(!ok) { free(items);return false; }
    c.cells=items;c.count=h.count;c.revision=h.revision;c.lastAutoSizeRevision=h.revision;c.settings=h.settings;
    c.states=(uint8_t *)malloc(h.count?h.count:1);if(c.states) memset(c.states,UNKNOWN,h.count);
    c.cellStates=(uint8_t *)malloc(h.count?h.count:1);if(c.cellStates) memset(c.cellStates,UNKNOWN,h.count);
    save(c);return true;
  }
  FileHeader h={};bool ok=f.read((uint8_t *)&h,sizeof(h))==sizeof(h) && h.magic==0x52434632 &&
    h.count<=MAX_CELLS && validSettings(h.settings) && f.size()==sizeof(h)+h.count*sizeof(Cell);
  if(ok) {
    Cell *items=(Cell *)malloc((h.count?h.count:1)*sizeof(Cell));
    ok=items && f.read((uint8_t *)items,h.count*sizeof(Cell))==h.count*sizeof(Cell) &&
      ~crcStep(0xFFFFFFFF,(uint8_t *)items,h.count*sizeof(Cell))==h.crc;
    if(ok) { for(uint16_t i=0;i<h.count;++i) if(!validCell(items[i],h.settings)) ok=false; }
    if(ok) { c.cells=items;c.count=h.count;c.revision=h.revision;c.lastAutoSizeRevision=h.lastAutoSizeRevision;c.settings=h.settings;
      c.states=(uint8_t *)malloc(h.count?h.count:1);if(c.states) memset(c.states,UNKNOWN,h.count);
      c.cellStates=(uint8_t *)malloc(h.count?h.count:1);if(c.cellStates) memset(c.cellStates,UNKNOWN,h.count); }
    else free(items);
  }
  f.close();return ok;
}
void nextUpload(Camera &c) {
  if(c.pendingType || !c.seen) return;
  Packet p={};
  if(c.phase==1) { Begin b={c.revision,c.count,c.settings};p=makePacket(CONFIG_BEGIN,sequence++,&b,sizeof(b)); }
  else if(c.phase==2 && c.uploadIndex<c.count) {
    CellMessage x={c.uploadIndex,c.cells[c.uploadIndex]};p=makePacket(CONFIG_CELL,sequence++,&x,sizeof(x));
  } else if(c.phase==2) { c.phase=3;nextUpload(c);return; }
  else if(c.phase==3) p=makePacket(CONFIG_COMMIT,sequence++,&c.revision,sizeof(c.revision));
  else return;
  c.pending=p;c.pendingType=p.type;c.pendingSeq=p.seq;c.pendingAt=millis();c.retries=0;
  bool sent=sendPacket(c.mac,p);
  char mac[18],detail[80];macText(c.mac,mac);
  snprintf(detail,sizeof(detail),"type=%u seq=%lu queued=%u revision=%lu",p.type,(unsigned long)p.seq,sent,(unsigned long)c.revision);
  recordDiagnostic("UPLOAD_SEND",mac,detail);
}
void startUpload(Camera &c) { c.phase=1;c.uploadIndex=0;c.pendingType=0;c.baseline=false;allUnknown(c);nextUpload(c); }
void onReceive(const esp_now_recv_info_t *info,const uint8_t *data,int length) {
  if(!info || !inbox || length<(int)offsetof(Packet,payload) || length>(int)sizeof(Packet)) return;
  Received r={};memcpy(r.mac,info->src_addr,6);memcpy(&r.packet,data,length);
  if(r.packet.magic!=MAGIC || r.packet.version!=VERSION ||
    r.packet.length>PAYLOAD || length!=(int)(offsetof(Packet,payload)+r.packet.length)) return;
  xQueueSend(inbox,&r,0);
}
void hexWrite(const uint8_t *data,size_t n) { static const char *hex="0123456789ABCDEF";for(size_t i=0;i<n;++i) { Serial.write(hex[data[i]>>4]);Serial.write(hex[data[i]&15]); } }
void handleSnapshot(Camera &c,const Packet &p) {
  char mac[18];macText(c.mac,mac);
  if(p.type==SNAPSHOT_END && p.seq==c.lastSnapEndSeq) { sendSnapshotAck(c,p.seq);return; }
  if(p.type==SNAPSHOT_BEGIN && (p.length==sizeof(SnapshotBegin) || p.length==sizeof(CompressedSnapshotBegin))) {
    SnapshotBegin b;memcpy(&b,p.payload,sizeof(b));
    if(b.bytes!=(uint32_t)b.width*b.height || b.bytes>1024UL*768 || !b.bytes) return;
    uint32_t wireBytes=b.bytes,wireCrc=b.crc;uint8_t codec=0;
    if(p.length==sizeof(CompressedSnapshotBegin)) {
      CompressedSnapshotBegin compressed;memcpy(&compressed,p.payload,sizeof(compressed));
      wireBytes=compressed.wireBytes;wireCrc=compressed.wireCrc;codec=compressed.codec;
      if(codec!=1 || !wireBytes || wireBytes>=b.bytes) return;
    }
    c.snapshot=true;c.snapFrame=b.frame;c.snapBytes=wireBytes;c.snapCrc=wireCrc;
    c.snapRawBytes=b.bytes;c.snapRawCrc=b.crc;c.snapCodec=codec;c.snapLastAt=millis();
    c.snapWidth=b.width;c.snapHeight=b.height;c.snapOffset=0;c.snapRunning=0xFFFFFFFF;
    sendSnapshotAck(c,p.seq);
    Serial.printf("EVENT SNAP_BEGIN %s %lu %u %u %lu %08lX %lu %08lX %u\n",mac,(unsigned long)b.frame,b.width,b.height,(unsigned long)b.bytes,(unsigned long)b.crc,(unsigned long)wireBytes,(unsigned long)wireCrc,codec);
    char detail[90];snprintf(detail,sizeof(detail),"frame=%lu raw=%lu wire=%lu codec=%u",(unsigned long)b.frame,(unsigned long)b.bytes,(unsigned long)wireBytes,codec);
    recordDiagnostic("FRAME_BEGIN",mac,detail);
  } else if(p.type==SNAPSHOT_CHUNK && c.snapshot && p.length>=5) {
    uint32_t offset;memcpy(&offset,p.payload,4);size_t n=p.length-4;
    if(offset==c.snapOffset && offset+n<=c.snapBytes) {
      c.snapRunning=crcStep(c.snapRunning,p.payload+4,n);c.snapOffset+=n;c.snapLastAt=millis();
      sendSnapshotAck(c,p.seq);
      Serial.printf("EVENT SNAP_DATA %s %lu ",mac,(unsigned long)offset);hexWrite(p.payload+4,n);Serial.println();
    }
    else if(offset+n==c.snapOffset) sendSnapshotAck(c,p.seq);
  } else if(p.type==SNAPSHOT_END && c.snapshot && p.length==sizeof(SnapshotEnd)) {
    SnapshotEnd e;memcpy(&e,p.payload,sizeof(e));
    bool ok=c.snapOffset==c.snapBytes && e.frame==c.snapFrame && e.crc==c.snapRawCrc && ~c.snapRunning==c.snapCrc;
    c.snapshot=false;c.lastSnapEndSeq=p.seq;sendSnapshotAck(c,p.seq);
    Serial.printf("EVENT SNAP_END %s %s\n",mac,ok?"ok":"crc_error");
    char detail[90];snprintf(detail,sizeof(detail),"result=%s bytes=%lu/%lu",ok?"ok":"crc_error",(unsigned long)c.snapOffset,(unsigned long)c.snapBytes);
    recordDiagnostic("FRAME_END",mac,detail);
  }
}
void handleRadio(const Received &r) {
  const Packet &p=r.packet;Camera *c=findCamera(r.mac);
  if(p.type==HELLO && p.length==sizeof(HelloMessage)) {
    HelloMessage h;memcpy(&h,p.payload,sizeof(h));
    if(memcmp(h.mac,r.mac,6) || h.channel!=radioChannel || !h.width || !h.height) return;
    if(!c) { for(auto &slot:cameras) if(!slot.used) { c=&slot;c->used=true;memcpy(c->mac,r.mac,6);break; } }
    if(!c) return;
    const bool wasSeen=c->seen;
    char mac[18];macText(c->mac,mac);
    if(!c->seen) {
      Serial.printf("EVENT DISCOVER %s %s\n",mac,c->cells?"configured":"new");if(c->cells) allUnknown(*c);
      char detail[110];snprintf(detail,sizeof(detail),"camera_rev=%lu bridge_rev=%lu cells=%u baseline=%u channel=%u",(unsigned long)h.revision,(unsigned long)c->revision,h.count,h.baseline,h.channel);
      recordDiagnostic("CAMERA_ONLINE",mac,detail);
    }
    if(c->lastHelloSeq && (int32_t)(p.seq-c->lastHelloSeq)<0) c->lastStateSeq=0;
    c->lastHelloSeq=p.seq;
    if(c->baseline && !h.baseline && c->cells) allUnknown(*c);
    c->seen=true;c->lastSeen=millis();c->remoteRevision=h.revision;c->baseline=h.baseline;
    addPeer(c->mac);
    // A scanning camera may only stay on this channel briefly. Answer its
    // first HELLO while it is still listening here.
    if(!wasSeen) broadcastBeacon();
    if(c->cells && !c->phase && h.revision!=c->revision) startUpload(*c);
    return;
  }
  if(!c || !c->seen) return;
  c->lastSeen=millis();
  if(p.type==DIAGNOSTIC && p.length==sizeof(DiagnosticPayload)) {
    DiagnosticPayload d;memcpy(&d,p.payload,sizeof(d));
    if(d.bootId!=c->diagBootId) { c->diagBootId=d.bootId;c->diagEventSeq=0; }
    if(d.eventSeq>c->diagEventSeq) {
      c->diagEventSeq=d.eventSeq;
      char mac[18],detail[128];macText(c->mac,mac);
      snprintf(detail,sizeof(detail),"boot=%08lX event=%lu ms=%lu reset=%u frame=%lu value=%lu offset=%lu fails=%u lost=%u ch=%u op=%u result=%u",
        (unsigned long)d.bootId,(unsigned long)d.eventSeq,(unsigned long)d.uptimeMs,d.resetReason,
        (unsigned long)d.frame,(unsigned long)d.detail,(unsigned long)d.offset,d.captureFailures,d.dropped,d.channel,d.operation,d.outcome);
      recordDiagnostic(cameraDiagnosticName(d.event),mac,detail);
    }
    Packet ack=makePacket(DIAGNOSTIC_ACK,p.seq,&d.bootId,sizeof(d.bootId));sendPacket(c->mac,ack);
    return;
  }
  if(p.type==SNAPSHOT_BEGIN && c->pendingType==SNAPSHOT_REQUEST) c->pendingType=0;
  if(p.type==ACK && p.length==sizeof(AckMessage)) {
    AckMessage a;memcpy(&a,p.payload,sizeof(a));
    if(p.seq!=c->pendingSeq || a.type!=c->pendingType) return;
    c->pendingType=0;
    if(a.type==CAPTURE_BASELINE || a.type==SNAPSHOT_REQUEST) {
      char mac[18],detail[64];macText(c->mac,mac);
      snprintf(detail,sizeof(detail),"type=%u status=%u retries=%u",a.type,a.status,c->retries);
      recordDiagnostic("REQUEST_ACK",mac,detail);
    }
    if(a.status) { char mac[18];macText(c->mac,mac);Serial.printf("EVENT CONFIG_ERROR %s %u %u\n",mac,a.type,a.status);c->phase=0;return; }
    if(a.type==CAPTURE_BASELINE || a.type==SNAPSHOT_REQUEST) {
      char mac[18];macText(c->mac,mac);Serial.printf("EVENT REQUEST_ACK %s %u %u\n",mac,a.type,a.status);
      if(a.type==CAPTURE_BASELINE) c->baseline=true;
      return;
    }
    if(c->phase==1) c->phase=2;
    else if(c->phase==2) ++c->uploadIndex;
    else if(c->phase==3) { c->phase=0;c->remoteRevision=c->revision;char mac[18];macText(c->mac,mac);Serial.printf("EVENT CONFIG_APPLIED %s %lu\n",mac,(unsigned long)c->revision); }
    nextUpload(*c);
  } else if(p.type==STATE && p.length==sizeof(StateMessage)) {
    StateMessage s;memcpy(&s,p.payload,sizeof(s));
    const int index=c->cells && !s.grouped?cellIndex(*c,s.id):-1;
    if(!c->cells || !c->baseline || s.revision!=c->revision || s.state>OCCUPIED ||
      (s.grouped?!isReported(*c,s.id):index<0) ||
      (c->lastStateSeq && (int32_t)(p.seq-c->lastStateSeq)<=0)) return;
    c->lastStateSeq=p.seq;
    char mac[18];macText(c->mac,mac);
    if(!s.grouped) {
      if(c->cellStates) c->cellStates[index]=s.state;
      Serial.printf("EVENT CELL_STATE %s %lu %s %u %lu\n",mac,(unsigned long)s.id,
        stateName(s.state),s.score,(unsigned long)s.frame);
      if(c->cells[index].group) return; // Grouped members are visual diagnostics, not MQTT outputs.
    }
    if(c->states) for(uint16_t i=0;i<c->count;++i)
      if((c->cells[i].group?c->cells[i].group:c->cells[i].id)==s.id) c->states[i]=s.state;
    Serial.printf("EVENT STATE %s %lu %s %u %lu\n",mac,(unsigned long)s.id,stateName(s.state),s.score,(unsigned long)s.frame);
    publish(areaSuffix(s.id),stateName(s.state));
  } else if(p.type==HEALTH && p.length==sizeof(HealthMessage)) {
    HealthMessage h;memcpy(&h,p.payload,sizeof(h));
    Packet reply=makePacket(HEALTH_ACK,p.seq,nullptr,0);sendPacket(c->mac,reply);
    char mac[18];macText(c->mac,mac);
    Serial.printf("EVENT HEALTH %s %lu %lu %u %u %u\n",mac,(unsigned long)h.revision,(unsigned long)h.frame,h.age,h.baseline,h.snapshot);
    publish("/cameras/"+String(mac)+"/health",String("{\"revision\":")+h.revision+",\"frame\":"+h.frame+",\"age_ms\":"+h.age+",\"baseline\":"+(h.baseline?"true":"false")+"}");
    if(c->baseline && !h.baseline && c->cells) allUnknown(*c);
    c->baseline=h.baseline;
    if(c->cells && h.revision!=c->revision && !c->phase) startUpload(*c);
  } else if(p.type==CELL_ANALYSIS && p.length==sizeof(AnalysisMessage)) {
    AnalysisMessage a;memcpy(&a,p.payload,sizeof(a));
    if(!c->baseline || a.revision!=c->revision || a.peakCount>10 || cellIndex(*c,a.id)<0) return;
    char mac[18];macText(c->mac,mac);
    Serial.printf("EVENT CELL_ANALYSIS %s %lu %lu %u %u %u %u %u ",mac,(unsigned long)a.id,
      (unsigned long)a.frame,a.score,a.threshold,a.referenceEdges,a.liveEdges,a.peakCount);
    for(int i=0;i<10;++i) Serial.printf("%s%u",i?",":"",a.angles[i]);
    Serial.print(' ');
    for(int i=0;i<31;++i) Serial.printf("%s%u",i?",":"",a.referenceBuckets[i]);
    Serial.print(' ');
    for(int i=0;i<31;++i) Serial.printf("%s%u",i?",":"",a.liveBuckets[i]);Serial.println();
  } else if(p.type==CALIBRATION_RESULT && p.length==sizeof(CalibrationResult)) {
    CalibrationResult value;memcpy(&value,p.payload,sizeof(value));
    if(value.count!=c->count || value.index>=c->count || value.revision!=c->revision+1 ||
       value.radius<3 || value.radius>50 || value.threshold<50 || value.threshold>1000 ||
       c->cells[value.index].id!=value.id) return;
    if(c->calibrationRevision!=value.revision) {
      Cell *next=(Cell *)realloc(c->staged,(c->count?c->count:1)*sizeof(Cell));
      if(!next) return;
      c->staged=next;memcpy(c->staged,c->cells,c->count*sizeof(Cell));
      c->stagedCount=c->count;c->stagedSettings=c->settings;c->stagedRevision=value.revision;
      c->calibrationRevision=value.revision;c->calibrationReceived=0;memset(c->calibrationSeen,0,sizeof(c->calibrationSeen));
    }
    const uint8_t mask=(uint8_t)(1u<<(value.index&7));
    if(c->calibrationSeen[value.index>>3]&mask) return;
    c->calibrationSeen[value.index>>3]|=mask;
    c->staged[value.index].radius=value.radius;c->staged[value.index].threshold=value.threshold;
    ++c->calibrationReceived;
    char mac[18];macText(c->mac,mac);
    Serial.printf("EVENT CALIBRATION_SENSOR %s %lu %u %u %u %u %u %u\n",mac,
      (unsigned long)value.id,value.radius,value.threshold,value.clearMaximum,
      value.referenceEdges,value.samples,value.confidence);
    if(c->calibrationReceived==c->count) {
      c->received=c->stagedCount;
      const uint32_t previousAutoSizeRevision=c->lastAutoSizeRevision;
      c->lastAutoSizeRevision=c->stagedRevision;
      if(saveStaged(*c)) {
        c->remoteRevision=c->revision;c->baseline=true;c->calibrationRevision=0;c->calibrationReceived=0;
        Serial.printf("EVENT CALIBRATION_APPLIED %s %lu %u\n",mac,(unsigned long)c->revision,c->count);
        Packet ack=makePacket(CALIBRATION_ACK,p.seq,&c->revision,sizeof(c->revision));sendPacket(c->mac,ack);
      } else { c->lastAutoSizeRevision=previousAutoSizeRevision;Serial.printf("EVENT CALIBRATION_ERROR %s storage\n",mac); }
    }
  } else if(p.type>=SNAPSHOT_BEGIN && p.type<=SNAPSHOT_END) handleSnapshot(*c,p);
}

bool hexDecode(const char *s,String &out) {
  if(!strcmp(s,"-")) { out="";return true; }
  size_t n=strlen(s);if(n%2) return false;out="";
  for(size_t i=0;i<n;i+=2) { char pair[3]={s[i],s[i+1],0};char *end;unsigned long v=strtoul(pair,&end,16);if(end!=pair+2 || v==0) return false;out+=(char)v; }
  return true;
}
bool number(const char *s,long &v,long low,long high) { if(!s || !*s) return false;char *end;v=strtol(s,&end,10);return !*end && v>=low && v<=high; }
const char *wifiError(int status) {
  if(status==WL_NO_SSID_AVAIL) return "network_not_found";
  if(status==WL_CONNECT_FAILED) return "authentication_failed";
  if(status==WL_CONNECTION_LOST) return "connection_lost";
  return "timeout_or_unreachable";
}
const char *mqttError(int status) {
  if(status==MQTT_CONNECTION_TIMEOUT) return "timeout";
  if(status==MQTT_CONNECT_FAILED) return "server_unreachable";
  if(status==MQTT_CONNECT_BAD_CREDENTIALS || status==MQTT_CONNECT_UNAUTHORIZED) return "credentials_rejected";
  if(status==MQTT_CONNECT_BAD_PROTOCOL || status==MQTT_CONNECT_BAD_CLIENT_ID) return "broker_rejected_connection";
  return "connection_failed";
}
void command(char *input) {
  char *saveptr=nullptr;char *cmd=strtok_r(input," \r\n",&saveptr);if(!cmd) return;
  if(!strcmp(cmd,"LOG")) {
    Serial.printf("LOG_NOW %08lX %lu\n",(unsigned long)bridgeBootId,(unsigned long)millis());
    for(uint8_t i=0;i<diagnosticCount;++i) Serial.println(diagnosticLog[(diagnosticHead+i)%64]);
    Serial.println("OK LOG");return;
  }
  if(!strcmp(cmd,"LIST")) {
    for(auto &c:cameras) if(c.used) { char mac[18];macText(c.mac,mac);Serial.printf("CAMERA %s %s %lu %lu %u %u\n",mac,c.seen?"online":"offline",(unsigned long)c.revision,(unsigned long)c.remoteRevision,c.count,c.baseline); }
    Serial.println("OK LIST");return;
  }
  if(!strcmp(cmd,"STATUS")) {
    bool wifiReady=ssid.length() && WiFi.status()==WL_CONNECTED;
    wifi_second_chan_t secondary=WIFI_SECOND_CHAN_NONE;
    uint8_t actualChannel=0;
    esp_wifi_get_channel(&actualChannel,&secondary);
    Serial.printf("RADIO_STATUS %u %u\n",radioChannel,actualChannel);
    Serial.printf("WIFI_STATUS %s %s\n",wifiReady?"connected":wifiPending?"connecting":"disconnected",
      wifiReady?WiFi.localIP().toString().c_str():"-");
    Serial.printf("MQTT_STATUS %s %d\n",mqtt.connected()?"connected":mqttPending?"connecting":"disconnected",mqtt.state());
    Serial.printf("SAVED_SETTINGS %u %u\n",prefs.getString("ssid","").length()?1u:0u,prefs.getString("broker","").length()?1u:0u);
    Serial.println("OK STATUS");return;
  }
  if(!strcmp(cmd,"FORGET_WIFI")) {
    if(mqtt.connected()) publish("/controller/health","offline");
    wifiPending=false;ssid="";password="";
    prefs.remove("ssid");prefs.remove("pass");
    mqttPending=false;
    broker=prefs.getString("broker","");brokerPort=prefs.getUShort("port",1883);
    topic=prefs.getString("topic","railway/home");mqttUser=prefs.getString("user","");mqttPass=prefs.getString("mqttPass","");
    mqtt.disconnect();WiFi.disconnect(false,true);
    mqtt.setServer(broker.c_str(),brokerPort);
    radioChannel=START_CHANNEL;
    esp_wifi_set_channel(START_CHANNEL,WIFI_SECOND_CHAN_NONE);
    fastBeaconUntil=millis()+FAST_BEACON_WINDOW_MS;broadcastBeacon();
    lastWifiConnected=false;lastMqttConnected=false;
    Serial.println("OK FORGET_WIFI");
    Serial.println("EVENT WIFI disconnected");
    Serial.println("EVENT MQTT_STATUS disconnected");return;
  }
  if(!strcmp(cmd,"FORGET_MQTT")) {
    if(mqtt.connected()) publish("/controller/health","offline");
    mqttPending=false;broker="";brokerPort=1883;topic="railway/home";mqttUser="";mqttPass="";
    prefs.remove("broker");prefs.remove("port");prefs.remove("topic");prefs.remove("user");prefs.remove("mqttPass");
    mqtt.disconnect();mqtt.setServer(broker.c_str(),brokerPort);
    lastMqttConnected=false;
    Serial.println("OK FORGET_MQTT");
    Serial.println("EVENT MQTT_STATUS disconnected");return;
  }
  if(!strcmp(cmd,"STATES")) {
    for(auto &c:cameras) if(c.used && c.cells) {
      char mac[18];macText(c.mac,mac);
      for(uint16_t i=0;i<c.count;++i) {
        Serial.printf("SENSOR %s %lu %s\n",mac,(unsigned long)c.cells[i].id,
          c.seen && c.baseline && c.cellStates?stateName(c.cellStates[i]):"unknown");
        uint32_t id=c.cells[i].group?c.cells[i].group:c.cells[i].id;
        bool first=true;for(uint16_t j=0;j<i;++j) if((c.cells[j].group?c.cells[j].group:c.cells[j].id)==id) first=false;
        if(first) Serial.printf("OUTPUT %s %lu %s %s\n",mac,(unsigned long)id,
          c.seen && c.baseline && c.states?stateName(c.states[i]):"unknown",areaName(id).c_str());
      }
    }
    Serial.println("OK STATES");return;
  }
  if(!strcmp(cmd,"WIFI")) {
    char *a=strtok_r(nullptr," \r\n",&saveptr),*b=strtok_r(nullptr," \r\n",&saveptr);
    String x,y;if(!a || !b || !hexDecode(a,x) || !hexDecode(b,y) || x.length()>32 || y.length()>64) { Serial.println("ERR WIFI args");return; }
    if(!x.length()) { Serial.println("ERR WIFI ssid");return; }
    ssid=x;password=y;wifiPending=true;wifiPendingAt=millis();
    WiFi.disconnect();WiFi.begin(ssid.c_str(),password.c_str());
    Serial.println("OK WIFI");Serial.println("EVENT WIFI connecting");return;
  }
  if(!strcmp(cmd,"MQTT")) {
    char *a=strtok_r(nullptr," \r\n",&saveptr),*b=strtok_r(nullptr," \r\n",&saveptr),*d=strtok_r(nullptr," \r\n",&saveptr);
    char *e=strtok_r(nullptr," \r\n",&saveptr),*f=strtok_r(nullptr," \r\n",&saveptr);
    String host,root,user,pass;long port;
    if(!a||!b||!d||!e||!f||!hexDecode(a,host)||!number(b,port,1,65535)||!hexDecode(d,root)||!hexDecode(e,user)||!hexDecode(f,pass)||
      host.length()>120||root.length()>100||user.length()>64||pass.length()>64||root.indexOf('#')>=0||root.indexOf('+')>=0) { Serial.println("ERR MQTT args");return; }
    if(!host.length() || !root.length()) { Serial.println("ERR MQTT host_or_root");return; }
    broker=host;brokerPort=port;topic=root;mqttUser=user;mqttPass=pass;
    mqttPending=true;mqttPendingAt=millis();lastMqttAttempt=0;
    mqtt.disconnect();mqtt.setServer(broker.c_str(),brokerPort);
    Serial.println("OK MQTT");Serial.println("EVENT MQTT_STATUS connecting");return;
  }
  char *macArg=strtok_r(nullptr," \r\n",&saveptr);Camera *c=macArg?cameraFor(macArg):nullptr;
  if(!c) { Serial.println("ERR CAMERA unknown");return; }
  if(!strcmp(cmd,"GET")) {
    char mac[18];macText(c->mac,mac);Serial.printf("CONFIG %s %lu %u %lu %u %d %d %d %u %u\n",mac,(unsigned long)c->revision,c->count,(unsigned long)c->lastAutoSizeRevision,c->settings.resolution,c->settings.brightness,c->settings.contrast,c->settings.saturation,c->settings.vflip,c->settings.hmirror);
    for(uint16_t i=0;i<c->count;++i) { Cell &x=c->cells[i];Serial.printf("CELL %u %lu %lu %u %u %u %u %u %u %u %u %u %lu\n",i,(unsigned long)x.id,(unsigned long)x.group,x.x,x.y,x.radius,x.shape,x.floor,x.tolerance,x.threshold,x.enter,x.clear,(unsigned long)x.createdRevision); }
    for(uint16_t i=0;i<c->count;++i) {
      uint32_t id=c->cells[i].group?c->cells[i].group:c->cells[i].id;
      bool first=true;for(uint16_t j=0;j<i;++j) if((c->cells[j].group?c->cells[j].group:c->cells[j].id)==id) first=false;
      if(first) { String name=areaName(id);Serial.printf("TOPIC %lu %s\n",(unsigned long)id,name.c_str()); }
    }
    Serial.println("OK GET");return;
  }
  if(!strcmp(cmd,"TOPIC")) {
    char *idArg=strtok_r(nullptr," \r\n",&saveptr),*nameArg=strtok_r(nullptr," \r\n",&saveptr);
    long parsed;String name;
    if(!number(idArg,parsed,1,2147483647) || !nameArg || !hexDecode(nameArg,name) ||
       !isReported(*c,(uint32_t)parsed) || name.length()>32) { Serial.println("ERR TOPIC args");return; }
    if(!name.length()) name=String(parsed);
    for(size_t i=0;i<name.length();++i) if(!((name[i]>='a'&&name[i]<='z') ||
      (name[i]>='0'&&name[i]<='9') || name[i]=='-' || name[i]=='_')) { Serial.println("ERR TOPIC name");return; }
    for(auto &other:cameras) if(other.used && other.cells) for(uint16_t i=0;i<other.count;++i) {
      uint32_t id=other.cells[i].group?other.cells[i].group:other.cells[i].id;
      if(id!=(uint32_t)parsed && areaName(id)==name) { Serial.println("ERR TOPIC duplicate");return; }
    }
    String old=areaName((uint32_t)parsed);
    if(name==String(parsed)) { LittleFS.remove(topicPath((uint32_t)parsed));if(old!=name) { clearArea(old);publishSnapshot(*c); }Serial.println("OK TOPIC");return; }
    File f=LittleFS.open(topicPath((uint32_t)parsed),"w");
    if(!f || f.print(name)!=name.length()) { if(f) f.close();Serial.println("ERR TOPIC storage");return; }
    f.close();if(old!=name) { clearArea(old);publishSnapshot(*c); }Serial.println("OK TOPIC");return;
  }
  if(!strcmp(cmd,"BEGIN")) {
    long v[8];for(int i=0;i<8;++i) { char *a=strtok_r(nullptr," \r\n",&saveptr);if(!number(a,v[i],i==0?1:i==1?0:i>=3&&i<=5?-2:0,i==0?2147483647:i==1?MAX_CELLS:i==2?3:i>=3&&i<=5?2:1)) { Serial.println("ERR BEGIN args");return; } }
    CameraSettings s={(uint8_t)v[2],(int8_t)v[3],(int8_t)v[4],(int8_t)v[5],(uint8_t)v[6],(uint8_t)v[7]};
    if((uint32_t)v[0]<=c->revision || !validSettings(s)) { Serial.println("ERR BEGIN revision");return; }
    Cell *next=(Cell *)realloc(c->staged,(v[1]?v[1]:1)*sizeof(Cell));
    if(!next || ESP.getFreeHeap()<24000) { if(next) c->staged=next;Serial.println("ERR BEGIN memory");return; }
    c->staged=next;
    c->stagedRevision=v[0];c->stagedCount=v[1];c->stagedSettings=s;c->received=0;
    Serial.println("OK BEGIN");return;
  }
  if(!strcmp(cmd,"CELL")) {
    long v[13];for(int i=0;i<13;++i) { char *a=strtok_r(nullptr," \r\n",&saveptr);if(!number(a,v[i],0,2147483647)) { Serial.println("ERR CELL args");return; } }
    Cell x={(uint32_t)v[1],(uint32_t)v[2],(uint16_t)v[3],(uint16_t)v[4],(uint8_t)v[5],(uint8_t)v[6],(uint16_t)v[7],(uint16_t)v[9],(uint8_t)v[8],(uint8_t)v[10],(uint8_t)v[11],c->stagedRevision};
    // Creation is bridge-owned metadata. Preserve it for an existing sensor
    // even when a setup app holds a stale draft after its first save.
    for(uint16_t i=0;i<c->count;++i) if(c->cells[i].id==x.id) { x.createdRevision=c->cells[i].createdRevision;break; }
    if(!c->staged || v[0]!=c->received || c->received>=c->stagedCount || !validCell(x,c->stagedSettings)) { Serial.println("ERR CELL invalid");return; }
    for(uint16_t i=0;i<c->received;++i) if(c->staged[i].id==x.id) { Serial.println("ERR CELL duplicate");return; }
    c->staged[c->received++]=x;Serial.println("OK CELL");return;
  }
  if(!strcmp(cmd,"COMMIT")) {
    if(c->staged && c->received==c->stagedCount) {
      for(uint16_t i=0;i<c->stagedCount;++i) {
        uint32_t id=c->staged[i].group?c->staged[i].group:c->staged[i].id;
        for(uint16_t j=0;j<i;++j) {
          uint32_t earlier=c->staged[j].group?c->staged[j].group:c->staged[j].id;
          if(earlier==id && (!c->staged[i].group || !c->staged[j].group)) {
            Serial.println("ERR COMMIT duplicate_output_id");return;
          }
        }
        for(auto &other:cameras) if(other.used && &other!=c && other.cells && isReported(other,id)) {
          Serial.println("ERR COMMIT duplicate_global_id");return;
        }
      }
    }
    if(!c->staged || c->received!=c->stagedCount || !saveStaged(*c)) { Serial.println("ERR COMMIT incomplete_or_storage");return; }
    Serial.println("OK COMMIT");if(c->seen) startUpload(*c);return;
  }
  if(!strcmp(cmd,"BASELINE") || !strcmp(cmd,"FRAME")) {
    if(!c->seen || c->phase || c->pendingType || c->snapshot) { Serial.println("ERR busy_or_offline");return; }
    uint8_t type=!strcmp(cmd,"BASELINE")?CAPTURE_BASELINE:SNAPSHOT_REQUEST;
    Packet p=makePacket(type,sequence++,nullptr,0);c->pending=p;c->pendingType=type;c->pendingSeq=p.seq;c->pendingAt=millis();c->retries=0;
    char mac[18],detail[75];macText(c->mac,mac);
    snprintf(detail,sizeof(detail),"seq=%lu camera_rev=%lu bridge_rev=%lu",(unsigned long)p.seq,(unsigned long)c->remoteRevision,(unsigned long)c->revision);
    recordDiagnostic(type==CAPTURE_BASELINE?"BASELINE_REQUEST":"FRAME_REQUEST",mac,detail);
    if(!sendPacket(c->mac,p)) { c->pendingType=0;recordDiagnostic("REQUEST_SEND_FAILED",mac,detail);Serial.println("ERR radio");return; }
    if(type==CAPTURE_BASELINE) { c->baseline=false;allUnknown(*c); }
    Serial.println("OK REQUEST");return;
  }
  if(!strcmp(cmd,"ANALYSIS")) {
    char *idArg=strtok_r(nullptr," \r\n",&saveptr);long id;
    if(!number(idArg,id,1,2147483647) || !c->seen || !c->baseline || cellIndex(*c,(uint32_t)id)<0) { Serial.println("ERR ANALYSIS unavailable");return; }
    const uint32_t sensorId=(uint32_t)id;
    Packet p=makePacket(ANALYSIS_REQUEST,sequence++,&sensorId,sizeof(sensorId));
    if(!sendPacket(c->mac,p)) { Serial.println("ERR ANALYSIS radio");return; }
    Serial.println("OK ANALYSIS");return;
  }
  if(!strcmp(cmd,"CALIBRATE")) {
    if(!c->seen || c->phase || c->pendingType || c->snapshot || !c->count) { Serial.println("ERR CALIBRATE unavailable");return; }
    char *mode=strtok_r(nullptr," \r\n",&saveptr);const bool all=mode && !strcmp(mode,"all");
    if(mode && !all && strcmp(mode,"new")) { Serial.println("ERR CALIBRATE mode");return; }
    const uint32_t since=all?0:c->lastAutoSizeRevision;
    CalibrationRequest value={c->revision+1,10000,since,50};
    Packet p=makePacket(CALIBRATE_REQUEST,sequence++,&value,sizeof(value));
    c->pending=p;c->pendingType=p.type;c->pendingSeq=p.seq;c->pendingAt=millis();c->retries=0;
    if(!sendPacket(c->mac,p)) { c->pendingType=0;Serial.println("ERR CALIBRATE radio");return; }
    Serial.println("OK CALIBRATE");return;
  }
  Serial.println("ERR command");
}
void serviceSerial() {
  while(Serial.available()) { char ch=Serial.read();if(ch=='\n') { line[lineLength]=0;command(line);lineLength=0; }
    else if(ch!='\r') { if(lineLength<sizeof(line)-1) line[lineLength++]=ch;else lineLength=0; } }
}
void serviceNetwork() {
  uint32_t now=millis();
  bool wifiConnected=ssid.length() && WiFi.status()==WL_CONNECTED;
  bool reportedWifi=false;
  if(wifiPending && wifiConnected) {
    prefs.putString("ssid",ssid);prefs.putString("pass",password);wifiPending=false;
    Serial.printf("EVENT WIFI connected %s\n",WiFi.localIP().toString().c_str());
    reportedWifi=true;
  } else if(wifiPending && now-wifiPendingAt>20000) {
    wifiPending=false;Serial.printf("EVENT WIFI failed %s\n",wifiError(WiFi.status()));
    ssid=prefs.getString("ssid","");password=prefs.getString("pass","");
    WiFi.disconnect();if(ssid.length()) WiFi.begin(ssid.c_str(),password.c_str());
    wifiConnected=false;
  }
  if(wifiConnected!=lastWifiConnected) {
    lastWifiConnected=wifiConnected;
    if(wifiConnected) { lastMqttAttempt=0;mqttStartupFailures=0; }
    char detail[75];snprintf(detail,sizeof(detail),"status=%s channel=%u",wifiConnected?"connected":"disconnected",wifiConnected?WiFi.channel():radioChannel);
    recordDiagnostic("WIFI_RADIO","-",detail);
    if(!wifiPending && !reportedWifi) Serial.printf("EVENT WIFI %s%s\n",wifiConnected?"connected":"disconnected",
      wifiConnected?(String(" ")+WiFi.localIP().toString()).c_str():"");
  }
  if(ssid.length() && !wifiConnected && !wifiPending && now-lastWifiAttempt>15000) { lastWifiAttempt=now;recordDiagnostic("WIFI_RETRY","-");WiFi.begin(ssid.c_str(),password.c_str()); }
  if(wifiConnected && radioChannel!=WiFi.channel()) {
    char detail[75];snprintf(detail,sizeof(detail),"from=%u to=%u",radioChannel,WiFi.channel());
    recordDiagnostic("CHANNEL_CHANGE","-",detail);
    radioChannel=WiFi.channel();
    fastBeaconUntil=now+FAST_BEACON_WINDOW_MS;
    broadcastBeacon();
  }
  if(mqttPending && now-mqttPendingAt>30000) {
    mqttPending=false;Serial.printf("EVENT MQTT_STATUS failed %s\n",mqttError(mqtt.state()));
    broker=prefs.getString("broker","");brokerPort=prefs.getUShort("port",1883);
    topic=prefs.getString("topic","railway/home");mqttUser=prefs.getString("user","");mqttPass=prefs.getString("mqttPass","");
    mqtt.disconnect();mqtt.setServer(broker.c_str(),brokerPort);
  }
  if(!wifiConnected || !broker.length()) {
    if(lastMqttConnected) { lastMqttConnected=false;Serial.println("EVENT MQTT_STATUS disconnected"); }
    return;
  }
  const uint32_t mqttRetryInterval=mqttPending?5000:mqttStartupFailures<2?MQTT_EARLY_RETRY_MS:MQTT_RETRY_MS;
  if(!mqtt.connected() && (lastMqttAttempt==0 || now-lastMqttAttempt>=mqttRetryInterval)) {
    lastMqttAttempt=now;String id="railway-"+String((uint32_t)ESP.getEfuseMac(),HEX);
    recordDiagnostic("MQTT_RETRY","-");
    if(mqtt.connect(id.c_str(),mqttUser.c_str(),mqttPass.c_str(),(topic+"/controller/health").c_str(),1,true,"offline")) {
      if(mqttPending) {
        prefs.putString("broker",broker);prefs.putUShort("port",brokerPort);prefs.putString("topic",topic);
        prefs.putString("user",mqttUser);prefs.putString("mqttPass",mqttPass);mqttPending=false;
      }
      Serial.println("EVENT MQTT_STATUS connected");lastMqttConnected=true;
      mqttStartupFailures=0;
      mqtt.subscribe((topic+"/areas/+/state").c_str());
      publish("/controller/health","online");for(auto &c:cameras) if(c.used) publishSnapshot(c);
    } else { if(mqttStartupFailures<2) ++mqttStartupFailures;char detail[32];snprintf(detail,sizeof(detail),"state=%d",mqtt.state());recordDiagnostic("MQTT_CONNECT_FAILED","-",detail); }
  }
  if(!mqtt.connected() && lastMqttConnected) { lastMqttConnected=false;Serial.printf("EVENT MQTT_STATUS disconnected %d\n",mqtt.state()); }
  mqtt.loop();
}
void setup() {
  Serial.begin(115200);delay(200);
  bridgeBootId=esp_random();recordDiagnostic("BRIDGE_BOOT","-");
  prefs.begin("railway",false);ssid=prefs.getString("ssid","");password=prefs.getString("pass","");
  broker=prefs.getString("broker","");brokerPort=prefs.getUShort("port",1883);topic=prefs.getString("topic","railway/home");
  mqttUser=prefs.getString("user","");mqttPass=prefs.getString("mqttPass","");
  { char detail[48];snprintf(detail,sizeof(detail),"wifi_saved=%u mqtt_saved=%u",ssid.length()?1u:0u,broker.length()?1u:0u);recordDiagnostic("SETTINGS_LOADED","-",detail); }
  net.setConnectionTimeout(2000);mqtt.setSocketTimeout(2);
  mqtt.setServer(broker.c_str(),brokerPort);mqtt.setBufferSize(512);
  mqtt.setCallback(onMqttMessage);
  const esp_partition_t *storage=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_DATA_SPIFFS,"spiffs");
  bool freshStorage=storage && blankStorage(storage);
  bool storageReady=storage && LittleFS.begin(freshStorage);
  if(!storage) Serial.println("ERR storage partition_missing");
  if(!storageReady) Serial.println("ERR storage mount");
  else { if(freshStorage) Serial.println("EVENT STORAGE initialized"); }
  if(storageReady) {
    // File.name() is only the basename on Arduino-ESP32; saved assignments use absolute paths.
    File root=LittleFS.open("/");File f=root.openNextFile();
    while(f) {
      String name=f.path();if(!name.startsWith("/")) name="/"+name;
      if(name.length()==18 && name.startsWith("/c") && name.endsWith(".bak")) {
        String live=name.substring(0,14);
        if(!LittleFS.exists(live) && !LittleFS.rename(name,live)) Serial.printf("ERR storage backup_restore %s\n",name.c_str());
      }
      f=root.openNextFile();
    }
    root.close();
    root=LittleFS.open("/");f=root.openNextFile();
    while(f) {
      String name=f.path();if(!name.startsWith("/")) name="/"+name;
      if(name.length()==14 && name.startsWith("/c")) {
        uint8_t mac[6];String hex=name.substring(2);char printable[18];
        snprintf(printable,sizeof(printable),"%c%c:%c%c:%c%c:%c%c:%c%c:%c%c",hex[0],hex[1],hex[2],hex[3],hex[4],hex[5],hex[6],hex[7],hex[8],hex[9],hex[10],hex[11]);
        if(parseMac(printable,mac)) for(auto &c:cameras) if(!c.used) {
          // load() may migrate and rewrite an old file, so its destination
          // path must be based on the real MAC rather than the zeroed slot.
          memcpy(c.mac,mac,6);
          if(load(c,name)) { c.used=true;Serial.printf("EVENT STORAGE camera_loaded %s revision=%lu cells=%u\n",printable,(unsigned long)c.revision,c.count); }
          else { memset(c.mac,0,sizeof(c.mac));Serial.printf("ERR storage camera_load %s\n",name.c_str()); }
          break;
        }
      }
      f=root.openNextFile();
    }
    root.close();
  }
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);
  if(ssid.length()) WiFi.begin(ssid.c_str(),password.c_str());
  else esp_wifi_set_channel(START_CHANNEL,WIFI_SECOND_CHAN_NONE);
  inbox=xQueueCreate(32,sizeof(Received));
  if(esp_now_init()!=ESP_OK) Serial.println("ERR esp_now init");
  else esp_now_register_recv_cb(onReceive);
  if(WiFi.status()==WL_CONNECTED) radioChannel=WiFi.channel();
  addPeer(BROADCAST);
  fastBeaconUntil=millis()+FAST_BEACON_WINDOW_MS;
  broadcastBeacon();
  Serial.printf("READY bridge %s channel=%u\n",BRIDGE_VERSION,radioChannel);
}
void loop() {
  serviceSerial();Received r;
  for(int i=0;i<32 && xQueueReceive(inbox,&r,0)==pdTRUE;++i) handleRadio(r);
  uint32_t now=millis();
  for(auto &c:cameras) if(c.used) {
    if(c.seen && now-c.lastSeen>CAMERA_TIMEOUT) {
      char mac[18];macText(c.mac,mac);
      char detail[90];snprintf(detail,sizeof(detail),"age_ms=%lu pending=%u frame_bytes=%lu/%lu",(unsigned long)(now-c.lastSeen),c.pendingType,(unsigned long)c.snapOffset,(unsigned long)c.snapBytes);
      recordDiagnostic("CAMERA_OFFLINE",mac,detail);
      if(c.pendingType) Serial.printf("EVENT TIMEOUT %s %u\n",mac,c.pendingType);
      if(c.snapshot) { Serial.printf("EVENT SNAP_END %s timeout\n",mac);c.snapshot=false; }
      c.seen=false;c.phase=0;c.pendingType=0;allUnknown(c);
      Serial.printf("EVENT CAMERA %s offline\n",mac);
    }
    if(c.snapshot && now-c.snapLastAt>10000) {
      char mac[18];macText(c.mac,mac);
      char detail[75];snprintf(detail,sizeof(detail),"bytes=%lu/%lu stalled_ms=%lu",(unsigned long)c.snapOffset,(unsigned long)c.snapBytes,(unsigned long)(now-c.snapLastAt));
      recordDiagnostic("FRAME_STALLED",mac,detail);
      c.snapshot=false;Serial.printf("EVENT SNAP_END %s timeout\n",mac);
    }
    uint32_t requestTimeout=c.pendingType==CAPTURE_BASELINE?10000:ACK_TIMEOUT;
    uint8_t maxRetries=8;
    if(c.pendingType && now-c.pendingAt>requestTimeout) {
      if(c.retries++>=maxRetries) { char mac[18],detail[75];macText(c.mac,mac);snprintf(detail,sizeof(detail),"type=%u retries=%u",c.pendingType,c.retries);recordDiagnostic("REQUEST_TIMEOUT",mac,detail);Serial.printf("EVENT TIMEOUT %s %u\n",mac,c.pendingType);c.pendingType=0;c.phase=0; }
      else { char mac[18],detail[75];macText(c.mac,mac);snprintf(detail,sizeof(detail),"type=%u retry=%u queued=%u",c.pendingType,c.retries,sendPacket(c.mac,c.pending));recordDiagnostic("REQUEST_RETRY",mac,detail);c.pendingAt=now; }
    }
    if(c.phase && !c.pendingType) nextUpload(c);
  }
  serviceNetwork();
  const uint32_t beaconNow=millis();
  const uint32_t beaconInterval=(int32_t)(fastBeaconUntil-beaconNow)>0?FAST_BEACON_MS:NORMAL_BEACON_MS;
  if(beaconNow-lastBeacon>=beaconInterval) broadcastBeacon();
  delay(1);
}
