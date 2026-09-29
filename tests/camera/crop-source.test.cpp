#include "stubs/esp_camera.h"
#ifndef CAMERA_BOARD_ESP32S3_EYE
#define CAMERA_BOARD_AI_THINKER 1
#endif
#define DEBUGF(...) do {} while(0)
enum { QVGA,VGA,SVGA,XGA };
struct CameraSettings { uint8_t resolution=VGA;int brightness=0,contrast=0,saturation=0,vflip=0,hmirror=0; };
#include "../../Camera_Module/ImageSource.h"

int main() {
  ImageSource source;CameraSettings settings;
  uint8_t *pixels=nullptr;uint16_t width=0,height=0;ImageSource::Frame frame;
  assert(source.begin(settings,pixels,width,height));
  assert(source.capture(pixels,width,height,frame));
  CaptureWindowPlanner planner;planner.add(328,364,13);const auto crop=planner.finish();
#if defined(CAMERA_BOARD_AI_THINKER)
  assert(source.supportsCrop());
  assert(source.begin(settings,pixels,width,height,&crop));
  assert(width==96 && height==96);
  assert(fake::config.frame_size==FRAMESIZE_96X96);
  assert(cropRegisters[0]==crop.x*5/4 && cropRegisters[1]==crop.y*5/4);
  assert(cropRegisters[2]==120 && cropRegisters[3]==120);
  assert(cropRegisters[4]==96 && cropRegisters[5]==96);
  assert(clockRegister==CAMERA_OV2640_VGA_CLOCK_DIVISOR-1);
  assert(source.capture(pixels,width,height,frame));
  assert(pixels==fake::pixels[0].data() || pixels==fake::pixels[1].data());
  assert(source.begin(settings,pixels,width,height));
  assert(width==640 && height==480);
  assert(source.capture(pixels,width,height,frame));
  // Reject unsupported settings BEFORE disturbing a good full-frame lease.
  auto *previous=pixels;
  settings.hmirror=1;assert(!source.begin(settings,pixels,width,height,&crop));
  assert(previous==pixels);settings.hmirror=0;
  settings.vflip=1;assert(!source.begin(settings,pixels,width,height,&crop));settings.vflip=0;
  settings.resolution=QVGA;assert(!source.begin(settings,pixels,width,height,&crop));settings.resolution=VGA;
  fake::sensor.id.PID=0x99;assert(!source.begin(settings,pixels,width,height,&crop));fake::sensor.id.PID=OV2640_PID;
  auto invalid=crop;invalid.x=639;assert(!source.begin(settings,pixels,width,height,&invalid));
  // Crop programming failure leaves no lease behind; full-frame recovery works.
  failCrop=true;assert(!source.begin(settings,pixels,width,height,&crop));
  assert(!fake::initialised);failCrop=false;
  assert(source.begin(settings,pixels,width,height));
  assert(source.capture(pixels,width,height,frame));
  // A transition-frame timeout also cleans up the rebuilt camera.
  fake::failGet=true;assert(!source.begin(settings,pixels,width,height,&crop));
  assert(!fake::initialised);fake::failGet=false;
  assert(source.begin(settings,pixels,width,height));
  assert(source.capture(pixels,width,height,frame));
#else
  assert(!source.supportsCrop());
  assert(!source.begin(settings,pixels,width,height,&crop));
  assert(width==640 && height==480);
#endif
  fake::failInit=true;assert(!source.begin(settings,pixels,width,height));
  assert(fake::gets==fake::returns);
  std::puts("Crop driver geometry, gating, transitions, lease cleanup and recovery passed");
}
