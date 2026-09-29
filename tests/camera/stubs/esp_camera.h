#pragma once
// Host-only driver/RTOS model for ImageSource ownership tests.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
inline uint32_t millis() {
  static const auto start=std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
}
inline void delay(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
struct Semaphore { std::mutex mutex;std::condition_variable changed;bool available=false; };
using SemaphoreHandle_t=Semaphore *;
constexpr int pdTRUE=1,pdPASS=1;
constexpr uint32_t portMAX_DELAY=UINT32_MAX;
inline uint32_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
inline SemaphoreHandle_t xSemaphoreCreateBinary() { return new Semaphore; }
inline int xSemaphoreGive(SemaphoreHandle_t s) {
  std::lock_guard<std::mutex> lock(s->mutex);s->available=true;s->changed.notify_one();return pdTRUE;
}
inline int xSemaphoreTake(SemaphoreHandle_t s,uint32_t ms) {
  std::unique_lock<std::mutex> lock(s->mutex);
  if(ms==portMAX_DELAY) s->changed.wait(lock,[&]{return s->available;});
  else if(!s->changed.wait_for(lock,std::chrono::milliseconds(ms),[&]{return s->available;})) return 0;
  s->available=false;return pdTRUE;
}
using portMUX_TYPE=std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
using TaskHandle_t=void *;
inline std::thread producer;
inline bool failTask=false;
inline int xTaskCreate(void (*entry)(void *),const char *,int,void *context,int,TaskHandle_t *handle) {
  if(failTask) return 0;
  if(producer.joinable()) producer.join();
  producer=std::thread(entry,context);*handle=reinterpret_cast<void *>(1);return pdPASS;
}
inline void vTaskDelete(void *) {}
using esp_err_t=int;
constexpr int ESP_OK=0,LEDC_TIMER_0=0,LEDC_CHANNEL_0=0;
constexpr int PIXFORMAT_GRAYSCALE=0,CAMERA_FB_IN_PSRAM=1,CAMERA_GRAB_WHEN_EMPTY=0;
enum framesize_t { FRAMESIZE_QVGA,FRAMESIZE_VGA,FRAMESIZE_SVGA,FRAMESIZE_XGA };
struct camera_config_t {
  int pin_pwdn,pin_reset,pin_xclk,pin_sccb_sda,pin_sccb_scl;
  int pin_d0,pin_d1,pin_d2,pin_d3,pin_d4,pin_d5,pin_d6,pin_d7;
  int pin_vsync,pin_href,pin_pclk,xclk_freq_hz,ledc_timer,ledc_channel;
  int pixel_format,frame_size,fb_location,fb_count,grab_mode;
};
struct camera_fb_t { uint8_t *buf;size_t len;uint16_t width,height;int format; };
constexpr int OV2640_PID=0x26;
inline int clockRegister=7,clockWrites=0;
inline bool failClockWrite=false;
struct sensor_t {
  struct { int PID=OV2640_PID; } id;
  static int readRegister(sensor_t *,int reg,int mask) {
    assert(reg==0x111);return clockRegister & mask;
  }
  static int writeRegister(sensor_t *,int reg,int mask,int value) {
    assert(reg==0x111);++clockWrites;
    if(failClockWrite) return -1;
    clockRegister=(clockRegister & ~mask)|(value & mask);return 0;
  }
  int (*get_reg)(sensor_t *,int,int)=readRegister;
  int (*set_reg)(sensor_t *,int,int,int)=writeRegister;
  static int control(sensor_t *,int) { return 0; }
  int (*set_brightness)(sensor_t *,int)=control;
  int (*set_contrast)(sensor_t *,int)=control;
  int (*set_saturation)(sensor_t *,int)=control;
  int (*set_vflip)(sensor_t *,int)=control;
  int (*set_hmirror)(sensor_t *,int)=control;
};
namespace fake {
inline std::mutex mutex;
inline camera_config_t config;
inline camera_fb_t frames[2];
inline std::vector<uint8_t> pixels[2];
inline bool held[2]={},initialised=false,failInit=false,missingSensor=false;
inline std::atomic<bool> failGet{false};
// Malformed frame cases: short, wrong format, wrong width, wrong height, null pixels.
inline std::atomic<int> invalidFrames{0};
inline unsigned gets=0,returns=0;
inline sensor_t sensor;
}
inline esp_err_t esp_camera_init(camera_config_t *config) {
  std::lock_guard<std::mutex> lock(fake::mutex);
  if(fake::failInit) return -1;
  assert(config->fb_count==2);
  assert(config->grab_mode==CAMERA_GRAB_WHEN_EMPTY);
  fake::config=*config;fake::initialised=true;
  clockRegister=config->frame_size==FRAMESIZE_QVGA?3:7;
  const uint16_t widths[]={320,640,800,1024},heights[]={240,480,600,768};
  for(int i=0;i<2;++i) {
    fake::pixels[i].resize(size_t(widths[config->frame_size])*heights[config->frame_size]);
    fake::frames[i]={fake::pixels[i].data(),fake::pixels[i].size(),widths[config->frame_size],heights[config->frame_size],PIXFORMAT_GRAYSCALE};
    fake::held[i]=false;
  }
  return ESP_OK;
}
inline esp_err_t esp_camera_deinit() {
  if(producer.joinable()) producer.join();
  std::lock_guard<std::mutex> lock(fake::mutex);
  assert(!fake::held[0] && !fake::held[1]); // Every lease must return BEFORE deinit.
  fake::initialised=false;return ESP_OK;
}
inline sensor_t *esp_camera_sensor_get() { return fake::missingSensor?nullptr:&fake::sensor; }
inline camera_fb_t *esp_camera_fb_get() {
  if(fake::failGet) return nullptr;
  std::lock_guard<std::mutex> lock(fake::mutex);
  assert(fake::initialised);
  for(int i=0;i<2;++i) if(!fake::held[i]) {
    fake::held[i]=true;++fake::gets;
    std::fill(fake::pixels[i].begin(),fake::pixels[i].end(),uint8_t(fake::gets));
    auto &f=fake::frames[i];
    const int invalid=fake::invalidFrames.load();
    if(invalid) {
      --fake::invalidFrames;
      if(invalid==5) --f.len;
      if(invalid==4) f.format=99;
      if(invalid==3) --f.width;
      if(invalid==2) --f.height;
      if(invalid==1) f.buf=nullptr;
    }
    return &f;
  }
  assert(false && "Producer requested more than two outstanding buffers");return nullptr;
}
inline void esp_camera_fb_return(camera_fb_t *f) {
  std::lock_guard<std::mutex> lock(fake::mutex);
  const auto i=f-fake::frames;assert(i>=0 && i<2 && fake::held[i]);
  fake::held[i]=false;++fake::returns;
  // Poison returned memory to detect early releases through public capture().
  std::fill(fake::pixels[i].begin(),fake::pixels[i].end(),0xEE);
  const uint16_t widths[]={320,640,800,1024},heights[]={240,480,600,768};
  *f={fake::pixels[i].data(),fake::pixels[i].size(),widths[fake::config.frame_size],heights[fake::config.frame_size],PIXFORMAT_GRAYSCALE};
}
