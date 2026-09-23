#pragma once
#include <esp_camera.h>

// Select the board pin map here. Additional DVP boards need only a new map.
#if defined(CAMERA_BOARD_AI_THINKER) && defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select one camera board
#endif
#if !defined(CAMERA_BOARD_AI_THINKER) && !defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select a camera board and add its pin map
#endif

#if defined(CAMERA_BOARD_AI_THINKER)
constexpr const char *CAMERA_BOARD_NAME="AI Thinker ESP32-CAM";
constexpr int PWDN=32, RESET=-1, XCLK=0, SIOD=26, SIOC=27;
constexpr int D0=5,D1=18,D2=19,D3=21,D4=36,D5=39,D6=34,D7=35;
constexpr int VSYNC=25,HREF=23,PCLK=22, STATUS_LED=4; // AI Thinker flash LED
#else
constexpr const char *CAMERA_BOARD_NAME="ESP32-S3-EYE";
constexpr int PWDN=-1, RESET=-1, XCLK=15, SIOD=4, SIOC=5;
constexpr int D0=11,D1=9,D2=8,D3=10,D4=12,D5=18,D6=17,D7=16;
constexpr int VSYNC=6,HREF=7,PCLK=13, STATUS_LED=3; // S3-EYE LED: verify board revision
#endif

struct Resolution { framesize_t frameSize; uint16_t width,height; };
const Resolution RESOLUTIONS[]={
  {FRAMESIZE_QVGA,320,240},{FRAMESIZE_VGA,640,480},
  {FRAMESIZE_SVGA,800,600},{FRAMESIZE_XGA,1024,768}
};

class ImageSource {
 public:
  struct Frame {
    uint8_t *pixels=nullptr;
    uint32_t sequence=0,capturedAtMs=0;
  };
  bool applyControls(const CameraSettings &settings) {
    if(!ready_) return false;
    sensor_t *sensor=esp_camera_sensor_get();
    if(!sensor) return false;
    // Sensor drivers do not consistently report success for every cosmetic
    // control in grayscale mode (notably saturation). The camera can still
    // capture valid frames, so only absence of the sensor is fatal here.
    sensor->set_brightness(sensor,settings.brightness);
    sensor->set_contrast(sensor,settings.contrast);
    sensor->set_saturation(sensor,settings.saturation);
    sensor->set_vflip(sensor,settings.vflip);
    sensor->set_hmirror(sensor,settings.hmirror);
    return true;
  }
  bool reconfigure(const CameraSettings &settings,uint8_t *&framePixels,
                   uint16_t &frameWidth,uint16_t &frameHeight) {
    if(!ready_ || settings.resolution>XGA) return false;
    const Resolution &next=RESOLUTIONS[settings.resolution];
    // A dimension change is a cold rebuild. This avoids needing both the old
    // and new double buffers in PSRAM at the same time; the caller already
    // restores the previous settings if this rebuild fails.
    if(frameWidth!=next.width || frameHeight!=next.height)
      return begin(settings,framePixels,frameWidth,frameHeight);
    if(!pause()) return false;
    const bool applied=applyControls(settings);
    resume();
    return applied;
  }
  bool begin(const CameraSettings &settings,uint8_t *&framePixels,uint16_t &frameWidth,uint16_t &frameHeight) {
  if(settings.resolution>XGA) return false;
  if(ready_) { stopCaptureTask();esp_camera_deinit();ready_=false; }
  free(buffers_[0]);free(buffers_[1]);buffers_[0]=buffers_[1]=nullptr;framePixels=nullptr;
  const Resolution &r=RESOLUTIONS[settings.resolution];
  frameWidth=r.width;frameHeight=r.height;
  captureWidth_=r.width;captureHeight_=r.height;
  buffers_[0]=(uint8_t *)ps_malloc((size_t)r.width*r.height);
  buffers_[1]=(uint8_t *)ps_malloc((size_t)r.width*r.height);
  if(!buffers_[0] || !buffers_[1]) {
    free(buffers_[0]);free(buffers_[1]);buffers_[0]=buffers_[1]=nullptr;return false;
  }
  camera_config_t c={};
  c.pin_pwdn=PWDN;c.pin_reset=RESET;c.pin_xclk=XCLK;c.pin_sccb_sda=SIOD;c.pin_sccb_scl=SIOC;
  c.pin_d0=D0;c.pin_d1=D1;c.pin_d2=D2;c.pin_d3=D3;c.pin_d4=D4;c.pin_d5=D5;c.pin_d6=D6;c.pin_d7=D7;
  c.pin_vsync=VSYNC;c.pin_href=HREF;c.pin_pclk=PCLK;c.xclk_freq_hz=20000000;
  c.ledc_timer=LEDC_TIMER_0;c.ledc_channel=LEDC_CHANNEL_0;
  c.pixel_format=PIXFORMAT_GRAYSCALE;c.frame_size=r.frameSize;
  c.fb_location=CAMERA_FB_IN_PSRAM;c.fb_count=1;c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
  const esp_err_t error=esp_camera_init(&c);
  if(error!=ESP_OK) {
    DEBUGF("camera init failed board=%s sensor_sda=%d sensor_scl=%d error=0x%X\n",
      CAMERA_BOARD_NAME,SIOD,SIOC,(unsigned)error);
    // esp_camera_init can leave partially installed GPIO/LEDC state behind.
    // Tear it down so a later cold-start recovery attempt can succeed.
    esp_camera_deinit();
    free(buffers_[0]);free(buffers_[1]);buffers_[0]=buffers_[1]=nullptr;return false;
  }
  ready_=true;
  if(!applyControls(settings)) {
    esp_camera_deinit();ready_=false;free(buffers_[0]);free(buffers_[1]);
    buffers_[0]=buffers_[1]=nullptr;return false;
  }
  resetMailbox();
  if(!startCaptureTask()) {
    esp_camera_deinit();ready_=false;free(buffers_[0]);free(buffers_[1]);
    buffers_[0]=buffers_[1]=nullptr;return false;
  }
  DEBUGF("camera %ux%u grayscale ready\n",frameWidth,frameHeight);
  return true;
}
  // Lease the newest completed frame. The producer will not overwrite it
  // until a later frame is leased, so analysis and snapshot reads are safe.
  bool capture(uint8_t *&framePixels,uint16_t,uint16_t,Frame &frame) {
    if(!ready_) return false;
    const uint32_t started=millis();
    do {
      portENTER_CRITICAL(&mailboxMux_);
      if(published_>=0 && published_!=writing_ && publishedSequence_!=leasedSequence_) {
        const uint32_t advanced=publishedSequence_-leasedSequence_;
        if(leasedSequence_ && advanced>1) replacedFrames_+=advanced-1;
        leased_=published_;leasedSequence_=publishedSequence_;
        framePixels=buffers_[leased_];frame={framePixels,leasedSequence_,publishedAtMs_};
        portEXIT_CRITICAL(&mailboxMux_);
        return true;
      }
      portEXIT_CRITICAL(&mailboxMux_);
      const uint32_t elapsed=millis()-started;
      if(elapsed>=1000) break;
      xSemaphoreTake(frameReady_,pdMS_TO_TICKS(1000-elapsed));
    } while(millis()-started<1000);
    return false;
  }

  bool pause() {
    if(!taskRunning_ || pauseRequested_) return taskRunning_;
    pauseRequested_=true;
    xSemaphoreTake(paused_,portMAX_DELAY);
    return true;
  }
  void resume() { pauseRequested_=false; }
  uint32_t acquisitionCount() const { return acquisitionCount_; }
  uint32_t acquisitionFailures() const { return acquisitionFailures_; }
  uint32_t replacedFrames() const { return replacedFrames_; }

 private:
  bool ready_=false;
  uint8_t *buffers_[2]={nullptr,nullptr};
  TaskHandle_t captureTask_=nullptr;
  SemaphoreHandle_t frameReady_=nullptr,stopped_=nullptr,paused_=nullptr;
  portMUX_TYPE mailboxMux_=portMUX_INITIALIZER_UNLOCKED;
  volatile bool stopRequested_=false,pauseRequested_=false,taskRunning_=false;
  volatile int8_t published_=-1,leased_=-1,writing_=-1;
  volatile uint32_t publishedSequence_=0,leasedSequence_=0,publishedAtMs_=0;
  volatile uint32_t acquisitionCount_=0,acquisitionFailures_=0,replacedFrames_=0;
  uint16_t captureWidth_=0,captureHeight_=0;

  void resetMailbox() {
    portENTER_CRITICAL(&mailboxMux_);
    published_=leased_=writing_=-1;publishedSequence_=leasedSequence_=publishedAtMs_=0;
    portEXIT_CRITICAL(&mailboxMux_);
  }
  static void captureTaskEntry(void *context) {
    static_cast<ImageSource *>(context)->captureLoop();
  }
  void captureLoop() {
    while(!stopRequested_) {
      if(pauseRequested_) {
        xSemaphoreGive(paused_);
        while(pauseRequested_ && !stopRequested_) delay(1);
        continue;
      }
      camera_fb_t *fb=esp_camera_fb_get();
      if(!fb) { ++acquisitionFailures_;delay(1);continue; }
      int8_t target;
      portENTER_CRITICAL(&mailboxMux_);
      target=leased_==0?1:0;
      writing_=target;
      portEXIT_CRITICAL(&mailboxMux_);
      const size_t bytes=(size_t)captureWidth_*captureHeight_;
      const bool valid=fb->format==PIXFORMAT_GRAYSCALE && fb->width==captureWidth_ &&
        fb->height==captureHeight_ && buffers_[target] && fb->len>=bytes;
      if(valid) memcpy(buffers_[target],fb->buf,bytes);
      esp_camera_fb_return(fb);
      if(valid) {
        portENTER_CRITICAL(&mailboxMux_);
        published_=target;publishedAtMs_=millis();++publishedSequence_;++acquisitionCount_;
        writing_=-1;
        portEXIT_CRITICAL(&mailboxMux_);
        xSemaphoreGive(frameReady_);
      } else {
        ++acquisitionFailures_;
        portENTER_CRITICAL(&mailboxMux_);writing_=-1;portEXIT_CRITICAL(&mailboxMux_);
      }
    }
    xSemaphoreGive(stopped_);
    vTaskDelete(nullptr);
  }
  bool startCaptureTask() {
    if(!frameReady_) frameReady_=xSemaphoreCreateBinary();
    if(!stopped_) stopped_=xSemaphoreCreateBinary();
    if(!paused_) paused_=xSemaphoreCreateBinary();
    if(!frameReady_ || !stopped_ || !paused_) return false;
    xSemaphoreTake(frameReady_,0);xSemaphoreTake(stopped_,0);xSemaphoreTake(paused_,0);
    stopRequested_=pauseRequested_=false;taskRunning_=true;
    if(xTaskCreate(captureTaskEntry,"camera-capture",4096,this,2,&captureTask_)==pdPASS) return true;
    taskRunning_=false;captureTask_=nullptr;return false;
  }
  void stopCaptureTask() {
    if(!taskRunning_) return;
    stopRequested_=true;pauseRequested_=false;
    xSemaphoreTake(stopped_,portMAX_DELAY);
    taskRunning_=false;captureTask_=nullptr;
  }
};
