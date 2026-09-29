#include "stubs/esp_camera.h"
#define CAMERA_BOARD_AI_THINKER 1
#define DEBUGF(...) do {} while(0)
enum { QVGA,VGA,SVGA,XGA };
struct CameraSettings { uint8_t resolution=QVGA;int brightness=0,contrast=0,saturation=0,vflip=0,hmirror=0; };
#include "../../Camera_Module/ImageSource.h"

int main() {
  ImageSource source;CameraSettings settings;
  uint8_t *pixels=nullptr;uint16_t width=0,height=0;ImageSource::Frame frame;
  assert(source.begin(settings,pixels,width,height));
  assert(clockWrites==0); // QVGA must keep its driver clock.
  assert(source.capture(pixels,width,height,frame));
  // Exact pointer identity proves the detector receives driver memory, not a copy.
  assert(pixels==fake::pixels[0].data() || pixels==fake::pixels[1].data());
  auto *first=pixels;const auto firstValue=pixels[0];
  assert(frame.sequence==1);
  delay(30);assert(source.pause());
  assert(first[0]==firstValue); // Successor capture did not overwrite the lease.
  assert(source.capture(pixels,width,height,frame)); // Drain the pending successor while paused.
  assert(pixels!=first && first[0]==0xEE && frame.sequence==2);
  auto *second=pixels;const auto secondValue=pixels[0];
  assert(!source.capture(pixels,width,height,frame)); // No duplicate frame on timeout.
  assert(pixels==second && pixels[0]==secondValue);
  source.resume();
  for(unsigned i=3;i<=20;++i) {
    assert(source.capture(pixels,width,height,frame));assert(frame.sequence==i);
    assert(pixels==fake::pixels[0].data() || pixels==fake::pixels[1].data());
  }
  assert(source.pause()); // Snapshot can hold a leased image indefinitely.
  const auto snapshotPixel=pixels[0];delay(30);assert(pixels[0]==snapshotPixel);
  source.resume();
  settings.resolution=VGA;
  assert(source.reconfigure(settings,pixels,width,height)); // Deinit checks both leases returned.
  assert(pixels==nullptr && width==640 && height==480);
#if CAMERA_OV2640_VGA_CLOCK_TRIAL
  assert(clockRegister==CAMERA_OV2640_VGA_CLOCK_DIVISOR-1 && clockWrites==1);
#else
  assert(clockRegister==7 && clockWrites==0);
#endif
  assert(source.capture(pixels,width,height,frame));assert(frame.sequence==1);
  settings.brightness=1;
  assert(source.reconfigure(settings,pixels,width,height)); // Pause/resume controls retain lease.
  assert(pixels==frame.pixels);

  // Rebuild with acquisition disabled, then verify malformed driver frames are
  // returned and skipped before publishing the first valid frame.
  fake::failGet=true;
  assert(source.begin(settings,pixels,width,height));
  assert(!source.capture(pixels,width,height,frame));
  assert(pixels==nullptr); // Driver acquisition failure never publishes a stale lease.
  fake::invalidFrames=5;fake::failGet=false;
  assert(source.capture(pixels,width,height,frame));assert(frame.sequence==1);
  assert(source.acquisitionFailures()>=5);

  fake::failInit=true;
  assert(!source.begin(settings,pixels,width,height));assert(pixels==nullptr);
  assert(!source.capture(pixels,width,height,frame));
  fake::failInit=false;fake::missingSensor=true;
  assert(!source.begin(settings,pixels,width,height));
  fake::missingSensor=false;failTask=true;
  assert(!source.begin(settings,pixels,width,height));
  failTask=false;
  assert(source.begin(settings,pixels,width,height));
  assert(source.capture(pixels,width,height,frame));
  fake::failInit=true;
  assert(!source.begin(settings,pixels,width,height));
  assert(fake::gets==fake::returns);
  // Never apply OV2640 register writes to another sensor.
  fake::failInit=false;fake::sensor.id.PID=0x99;
  const int writesBefore=clockWrites;
  assert(source.begin(settings,pixels,width,height));
  assert(clockWrites==writesBefore && clockRegister==7);
  fake::failInit=true;assert(!source.begin(settings,pixels,width,height));
  fake::sensor.id.PID=OV2640_PID;fake::failInit=false;
#if CAMERA_OV2640_VGA_CLOCK_TRIAL
  // A successful bus write without the requested readback must also fail.
  fake::sensor.set_reg=[](sensor_t *,int,int,int) { return 0; };
  assert(!source.begin(settings,pixels,width,height));
  assert(!fake::initialised && pixels==nullptr);
  fake::sensor.set_reg=sensor_t::writeRegister;
  // Unknown driver clock settings must not be overwritten.
  fake::sensor.get_reg=[](sensor_t *,int,int) { return 0x87; };
  const int writesBeforeUnknown=clockWrites;
  assert(source.begin(settings,pixels,width,height));
  assert(clockWrites==writesBeforeUnknown);
  fake::failInit=true;assert(!source.begin(settings,pixels,width,height));
  fake::failInit=false;fake::sensor.get_reg=sensor_t::readRegister;
  failClockWrite=true;
  assert(!source.begin(settings,pixels,width,height));
  assert(!fake::initialised && pixels==nullptr);
  failClockWrite=false;
  assert(source.begin(settings,pixels,width,height));
  assert(clockRegister==CAMERA_OV2640_VGA_CLOCK_DIVISOR-1);
  fake::failInit=true;assert(!source.begin(settings,pixels,width,height));
#endif
  assert(fake::gets==fake::returns);
  std::puts("ImageSource ownership, pause, timeout, validation, rebuild and failure recovery passed");
}
