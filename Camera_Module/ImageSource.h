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
    if(frameWidth!=next.width || frameHeight!=next.height) {
      uint8_t *nextPixels=(uint8_t *)ps_malloc((size_t)next.width*next.height);
      sensor_t *sensor=esp_camera_sensor_get();
      if(!nextPixels || !sensor) { free(nextPixels);return false; }
      if(sensor->set_framesize(sensor,next.frameSize)!=0) { free(nextPixels);return false; }
      free(framePixels);framePixels=nextPixels;frameWidth=next.width;frameHeight=next.height;
      DEBUGF("camera changed to %ux%u grayscale\n",frameWidth,frameHeight);
    }
    return applyControls(settings);
  }
  bool begin(const CameraSettings &settings,uint8_t *&framePixels,uint16_t &frameWidth,uint16_t &frameHeight) {
  if(settings.resolution>XGA) return false;
  if(ready_) { esp_camera_deinit();ready_=false; }
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
  const esp_err_t error=esp_camera_init(&c);
  if(error!=ESP_OK) {
    DEBUGF("camera init failed board=%s sensor_sda=%d sensor_scl=%d error=0x%X\n",
      CAMERA_BOARD_NAME,SIOD,SIOC,(unsigned)error);
    // esp_camera_init can leave partially installed GPIO/LEDC state behind.
    // Tear it down so a later cold-start recovery attempt can succeed.
    esp_camera_deinit();
    free(framePixels);framePixels=nullptr;return false;
  }
  ready_=true;
  if(!applyControls(settings)) { esp_camera_deinit();ready_=false;free(framePixels);framePixels=nullptr;return false; }
  DEBUGF("camera %ux%u grayscale ready\n",frameWidth,frameHeight);
  return true;
}
  bool capture(uint8_t *framePixels,uint16_t frameWidth,uint16_t frameHeight) {
  if(!ready_ || !framePixels) return false;
  camera_fb_t *fb=esp_camera_fb_get();
  const size_t bytes=(size_t)frameWidth*frameHeight;
  if(!fb) return false;
  const bool valid=fb->format==PIXFORMAT_GRAYSCALE && fb->width==frameWidth &&
    fb->height==frameHeight && fb->len>=bytes;
  if(valid) memcpy(framePixels,fb->buf,bytes);
  esp_camera_fb_return(fb);
  return valid;
}

 private:
  bool ready_=false;
};
