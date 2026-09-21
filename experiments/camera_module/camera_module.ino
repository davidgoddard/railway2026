#define CAMERA_ANGLE_EXPERIMENT_VERSION "0.12.0"

/*
  Railway camera angle experiment
  Arduino-ESP32 core, AI Thinker ESP32-CAM by default.
  Select CAMERA_BOARD_ESP32S3_EYE below for an Espressif ESP32-S3-EYE.
  Other S3 camera boards need their own pin mapping.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <math.h>

#define CAMERA_BOARD_AI_THINKER 1
// #define CAMERA_BOARD_ESP32S3_EYE 1

#if defined(CAMERA_BOARD_AI_THINKER) && defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select only one camera board
#endif
#if !defined(CAMERA_BOARD_AI_THINKER) && !defined(CAMERA_BOARD_ESP32S3_EYE)
#error Select a camera board and provide its pin mapping
#endif

#if defined(CAMERA_BOARD_AI_THINKER)
constexpr int PWDN = 32, RESET = -1, XCLK = 0, SIOD = 26, SIOC = 27;
constexpr int D0 = 5, D1 = 18, D2 = 19, D3 = 21;
constexpr int D4 = 36, D5 = 39, D6 = 34, D7 = 35;
constexpr int VSYNC = 25, HREF = 23, PCLK = 22;
#else
constexpr int PWDN = -1, RESET = -1, XCLK = 15, SIOD = 4, SIOC = 5;
constexpr int D0 = 11, D1 = 9, D2 = 8, D3 = 10;
constexpr int D4 = 12, D5 = 18, D6 = 17, D7 = 16;
constexpr int VSYNC = 6, HREF = 7, PCLK = 13;
#endif

constexpr char AP_SSID[] = "Railway-Angle-Lab";
constexpr char AP_PASSWORD[] = "railway123"; // Change before use near other people.
struct Resolution {
  const char *name;
  framesize_t frameSize;
  int width;
  int height;
};
struct ShapeBench {
  uint32_t microseconds;
  uint32_t samples;
};
const Resolution RESOLUTIONS[] = {
  {"qvga", FRAMESIZE_QVGA, 320, 240},
  {"vga", FRAMESIZE_VGA, 640, 480},
  {"svga", FRAMESIZE_SVGA, 800, 600},
  {"xga", FRAMESIZE_XGA, 1024, 768},
};
constexpr int RESOLUTION_COUNT = sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0]);
int resolutionIndex = 0;
int WIDTH = 320;
int HEIGHT = 240;
size_t PIXELS = (size_t)WIDTH * HEIGHT;
constexpr int BINS = 18;              // 0–180 degrees, 10 degrees per bin
constexpr int MAX_PEAKS = 3;
constexpr unsigned CAPTURE_PERIOD_MS = 100;

WebServer server(80);
uint8_t *grayFrame = nullptr;
bool cameraStarted = false;
bool frameReady = false;
uint32_t frameId = 0;
uint32_t lastCaptureMs = 0;
uint32_t lastAttemptMs = 0;
uint32_t captureFailures = 0;
uint32_t processingUs = 0;
uint32_t analysisUs = 0;
uint32_t captureUs = 0;
uint32_t frameGetUs = 0;
uint32_t frameCopyUs = 0;
float estimatedCaptureUs = 0;
float estimatedCellUs = 0;
float measuredFps = 0;

struct Region {
  int x = 160;
  int y = 120;
  int radius = 5;
  int gradientMin = 80;               // Quality floor, not an angle-selection threshold
  float threshold = 0.20f;
  int angleTolerance = 10;
  bool circle = true;
} region;

struct Features {
  uint32_t hist[BINS] = {};
  float angleSum[BINS] = {};
  uint32_t edges = 0;
  uint32_t samples = 0;
  uint32_t graySum = 0;
  uint32_t whitePixels = 0;
  uint16_t medianGradient = 0;
  uint16_t upperGradient = 0;
  float meanGray = 0;
  uint8_t minGray = 255;
  uint8_t maxGray = 0;
  int maxGradient = 0;
  uint32_t dominantMask = 0;
  float peakAngle[MAX_PEAKS] = {};
  float peakShare[MAX_PEAKS] = {};
  int peakCount = 0;
  bool textured = false;
  bool quality = false;
};

struct CellBreakdown {
  uint32_t firstPassUs = 0;
  uint32_t secondPassUs = 0;
  uint32_t peaksUs = 0;
  uint32_t compareUs = 0;
} cellTiming;

Features currentFeatures;
Features baselineFeatures;
Features flaggedDiagnostics;
uint32_t flaggedDiagnosticUs = 0;
bool hasFlaggedDiagnostics = false;
struct Projection {
  uint32_t bucket[MAX_PEAKS + 1] = {}; // Reference peaks, then all other directions.
};
struct ReferenceDirection {
  int16_t x = 0, y = 0; // Q8 unit vector, sign ignored for 0-180 degree edges.
};
struct EdgeAnchor {
  int16_t x = 0, y = 0;
};
constexpr int MAX_EDGE_ANCHORS = 2;
constexpr int ANCHOR_SEARCH_RADIUS = 2;
EdgeAnchor edgeAnchors[MAX_EDGE_ANCHORS];
uint8_t edgeAnchorCount = 0;
uint8_t edgeAnchorsMatched = 0;
uint8_t edgeAnchorMatchedMask = 0;
float edgeAnchorScore = 0;
ReferenceDirection referenceDirection[MAX_PEAKS];
Projection referenceByTolerance[21];
Projection liveProjection;
uint8_t toleranceTanQ8[21];
bool hasBaseline = false;
bool exposureUnreliable = false;
float changeScore = 0;
float edgeTailScore = 0;
bool changed = false;
int changedFrames = 0;
int clearFrames = 0;
int matchedIndex[MAX_PEAKS] = {-1, -1, -1};
int nearestIndex[MAX_PEAKS] = {-1, -1, -1};
float nearestDelta[MAX_PEAKS] = {};
uint8_t matchedLiveMask = 0;

// Every capture attempt gets one record, including failed captures. A ring in
// PSRAM preserves frames that the browser's slower polling would otherwise miss.
struct FrameLog {
  uint32_t attempt, frame, uptimeMs, captureUs, cellUs;
  uint32_t diagnosticUs;
  uint32_t getUs, copyUs, firstPassUs, secondPassUs, peaksUs, compareUs;
  uint32_t configRevision, baselineRevision;
  uint16_t x, y, radius, contrastFloor, samples, edges, maxGradient;
  uint16_t referenceEdges, referenceWhitePixels, liveWhitePixels, enterFrames, clearFrames;
  uint16_t referenceMedianGradient, referenceUpperGradient;
  uint16_t liveMedianGradient, liveUpperGradient;
  uint8_t shape, tolerance, captureOk, hasReference, changed;
  uint8_t referenceKind, liveKind, minGray, maxGray;
  uint8_t projectionMode, exposureUnreliable, anchorCount, anchorsMatched, anchorMatchedMask;
  int16_t anchorX[MAX_EDGE_ANCHORS], anchorY[MAX_EDGE_ANCHORS];
  float threshold, score, edgeTailScore, anchorScore, meanGray, referenceMeanGray;
  float referenceAngles[MAX_PEAKS], liveAngles[MAX_PEAKS];
  float referenceShares[MAX_PEAKS], liveShares[MAX_PEAKS];
  uint16_t referenceHist[BINS], liveHist[BINS];
  uint16_t referenceBucket[MAX_PEAKS + 1], liveBucket[MAX_PEAKS + 1];
};
FrameLog *frameLog = nullptr;
uint16_t logCapacity = 0, logCount = 0, logHead = 0;
uint32_t attemptSequence = 0, configRevision = 1, baselineRevision = 0;

uint8_t featureKind(const Features &f) {
  if (!f.textured) return 0; // blank
  return f.quality ? 2 : 1;  // oriented or unstructured texture
}
const char *kindName(uint8_t kind) {
  return kind == 0 ? "blank" : kind == 1 ? "texture" : kind == 2 ? "oriented"
    : kind == 4 ? "projected" : "none";
}

void appendFrameLog(bool captureOk) {
  if (!frameLog || !logCapacity) return;
  FrameLog &entry = frameLog[logHead];
  entry = FrameLog();
  entry.attempt = attemptSequence;
  entry.frame = frameId;
  entry.uptimeMs = millis();
  entry.captureUs = captureUs;
  entry.cellUs = captureOk ? analysisUs : 0;
  entry.diagnosticUs = captureOk ? flaggedDiagnosticUs : 0;
  entry.getUs = frameGetUs;
  entry.copyUs = frameCopyUs;
  if (captureOk) {
    entry.firstPassUs = cellTiming.firstPassUs;
    entry.secondPassUs = cellTiming.secondPassUs;
    entry.peaksUs = cellTiming.peaksUs;
    entry.compareUs = cellTiming.compareUs;
  }
  entry.configRevision = configRevision;
  entry.baselineRevision = baselineRevision;
  entry.x = region.x; entry.y = region.y; entry.radius = region.radius;
  entry.contrastFloor = region.gradientMin;
  entry.shape = region.circle ? 0 : 1;
  entry.tolerance = region.angleTolerance;
  entry.threshold = region.threshold;
  entry.captureOk = captureOk;
  entry.hasReference = hasBaseline;
  entry.changed = changed;
  entry.exposureUnreliable = exposureUnreliable;
  entry.enterFrames = (uint16_t)min(changedFrames, 65535);
  entry.clearFrames = (uint16_t)min(clearFrames, 65535);
  entry.score = hasBaseline && captureOk ? changeScore : 0;
  entry.edgeTailScore = hasBaseline && captureOk ? edgeTailScore : 0;
  entry.anchorScore = hasBaseline && captureOk ? edgeAnchorScore : 0;
  entry.anchorCount = hasBaseline ? edgeAnchorCount : 0;
  entry.anchorsMatched = hasBaseline && captureOk ? edgeAnchorsMatched : 0;
  entry.anchorMatchedMask = hasBaseline && captureOk ? edgeAnchorMatchedMask : 0;
  for (int i = 0; i < MAX_EDGE_ANCHORS; ++i) {
    entry.anchorX[i] = i < edgeAnchorCount ? edgeAnchors[i].x : -1;
    entry.anchorY[i] = i < edgeAnchorCount ? edgeAnchors[i].y : -1;
  }
  entry.referenceKind = hasBaseline ? featureKind(baselineFeatures) : 3;
  entry.liveKind = captureOk
    ? (hasBaseline && baselineFeatures.quality && currentFeatures.textured
       ? 4 : featureKind(currentFeatures)) : 3;
  entry.projectionMode = hasBaseline && baselineFeatures.quality;
  if (entry.projectionMode) {
    for (int i = 0; i <= MAX_PEAKS; ++i) {
      entry.referenceBucket[i] = referenceByTolerance[region.angleTolerance].bucket[i];
      if (captureOk) entry.liveBucket[i] = liveProjection.bucket[i];
    }
  }
  if (hasBaseline) {
    entry.referenceEdges = baselineFeatures.edges;
    entry.referenceWhitePixels = baselineFeatures.whitePixels;
    entry.referenceMedianGradient = baselineFeatures.medianGradient;
    entry.referenceUpperGradient = baselineFeatures.upperGradient;
    entry.referenceMeanGray = baselineFeatures.meanGray;
    for (int i = 0; i < BINS; ++i)
      entry.referenceHist[i] = baselineFeatures.hist[i];
    for (int i = 0; i < MAX_PEAKS; ++i) {
      entry.referenceAngles[i] = baselineFeatures.peakAngle[i];
      entry.referenceShares[i] = baselineFeatures.peakShare[i];
    }
  }
  if (captureOk) {
    entry.samples = currentFeatures.samples;
    entry.edges = currentFeatures.edges;
    entry.liveWhitePixels = currentFeatures.whitePixels;
    entry.liveMedianGradient = currentFeatures.medianGradient;
    entry.liveUpperGradient = currentFeatures.upperGradient;
    entry.maxGradient = currentFeatures.maxGradient;
    entry.meanGray = currentFeatures.meanGray;
    entry.minGray = currentFeatures.minGray;
    entry.maxGray = currentFeatures.maxGray;
    const Features &details = hasFlaggedDiagnostics ? flaggedDiagnostics : currentFeatures;
    for (int i = 0; i < BINS; ++i)
      entry.liveHist[i] = details.hist[i];
    for (int i = 0; i < MAX_PEAKS; ++i) {
      entry.liveAngles[i] = details.peakAngle[i];
      entry.liveShares[i] = details.peakShare[i];
    }
  }
  logHead = (logHead + 1) % logCapacity;
  if (logCount < logCapacity) ++logCount;
}

// The angle is the gradient normal, so a visible line appears at angle + 90°.
void gradientAt(const uint8_t *pixels, int p, int &gx, int &gy) {
  const int rawX = -3 * pixels[p - WIDTH - 1] + 3 * pixels[p - WIDTH + 1]
                   - 10 * pixels[p - 1] + 10 * pixels[p + 1]
                   - 3 * pixels[p + WIDTH - 1] + 3 * pixels[p + WIDTH + 1];
  const int rawY = -3 * pixels[p - WIDTH - 1] - 10 * pixels[p - WIDTH]
                   - 3 * pixels[p - WIDTH + 1] + 3 * pixels[p + WIDTH - 1]
                   + 10 * pixels[p + WIDTH] + 3 * pixels[p + WIDTH + 1];
  gx = rawX < 0 ? -((-rawX + 2) / 4) : (rawX + 2) / 4;
  gy = rawY < 0 ? -((-rawY + 2) / 4) : (rawY + 2) / 4;
}

int projectionBucket(int gx, int gy, int tolerance) {
  int best = MAX_PEAKS;
  int32_t bestDot = -1;
  const int32_t tanQ8 = toleranceTanQ8[tolerance];
  for (int i = 0; i < baselineFeatures.peakCount; ++i) {
    const int32_t dot = abs(gx * referenceDirection[i].x + gy * referenceDirection[i].y);
    const int32_t cross = abs(gy * referenceDirection[i].x - gx * referenceDirection[i].y);
    if (cross * 256 <= dot * tanQ8 && dot > bestDot) {
      best = i;
      bestDot = dot;
    }
  }
  return best;
}

void selectDominantPeaks(Features &f) {
  uint32_t support[BINS] = {};
  for (int i = 0; i < BINS; ++i) {
    support[i] = f.hist[(i + BINS - 1) % BINS] + f.hist[i]
               + f.hist[(i + 1) % BINS];
  }
  uint32_t primarySupport = 0;
  for (int peak = 0; peak < MAX_PEAKS; ++peak) {
    int best = -1;
    uint32_t bestSupport = 0;
    for (int i = 0; i < BINS; ++i) {
      bool tooClose = false;
      for (int offset = -2; offset <= 2; ++offset) {
        if (f.dominantMask & (1UL << ((i + offset + BINS) % BINS))) {
          tooClose = true;
          break;
        }
      }
      if (!tooClose && support[i] > bestSupport) {
        best = i;
        bestSupport = support[i];
      }
    }
    if (best < 0 || bestSupport < 6) break;
    if (peak == 0) {
      // A real dominant orientation should account for a substantial share
      // of the strong gradients, even if its exact 10-degree bin fluctuates.
      if (bestSupport * 4 < f.edges) break;
      primarySupport = bestSupport;
    } else if (bestSupport * 5 < f.edges || bestSupport * 2 < primarySupport) {
      break;
    }
    f.dominantMask |= 1UL << best;
    const float centre = best * 10.0f + 5.0f;
    float totalAngle = 0;
    for (int offset = -1; offset <= 1; ++offset) {
      const int bin = (best + offset + BINS) % BINS;
      if (!f.hist[bin]) continue;
      float mean = f.angleSum[bin] / f.hist[bin];
      while (mean - centre > 90.0f) mean -= 180.0f;
      while (mean - centre < -90.0f) mean += 180.0f;
      totalAngle += mean * f.hist[bin];
    }
    float refined = totalAngle / bestSupport;
    if (refined < 0) refined += 180.0f;
    if (refined >= 180.0f) refined -= 180.0f;
    f.peakAngle[f.peakCount] = refined;
    f.peakShare[f.peakCount] = (float)bestSupport / f.edges;
    ++f.peakCount;
  }
  f.quality = f.peakCount > 0;
}

Features analyse(const uint8_t *pixels, CellBreakdown *timing, bool projected) {
  Features f;
  uint16_t magnitudeBins[64] = {};
  if (projected) liveProjection = Projection();
  if (timing) *timing = CellBreakdown();
  const uint32_t firstStartUs = timing ? micros() : 0;
  const int yStart = max(1, region.y - region.radius);
  const int yEnd = min(HEIGHT - 2, region.y + region.radius - (region.circle ? 0 : 1));
  // First pass: find the strongest local gradient. Angle selection then uses
  // a fraction of this value, so uniform brightness/contrast scaling changes
  // magnitude without changing which orientations are counted.
  for (int y = yStart; y <= yEnd; ++y) {
    int halfWidth = region.radius;
    if (region.circle) {
      const int dy = y - region.y;
      halfWidth = (int)sqrtf((float)(region.radius * region.radius - dy * dy));
    }
    const int xStart = max(1, region.x - halfWidth);
    const int xEnd = min(WIDTH - 2, region.x + halfWidth - (region.circle ? 0 : 1));
    for (int x = xStart; x <= xEnd; ++x) {
      const int p = y * WIDTH + x;
      int gx, gy;
      gradientAt(pixels, p, gx, gy);
      const int magnitude = abs(gx) + abs(gy);
      ++f.samples;
      const uint8_t gray = pixels[p];
      f.graySum += gray;
      if (gray < f.minGray) f.minGray = gray;
      if (gray > f.maxGray) f.maxGray = gray;
      if (gray >= 250) ++f.whitePixels;
      f.maxGradient = max(f.maxGradient, magnitude);
      ++magnitudeBins[min(magnitude >> 4, 63)];
    }
  }
  uint32_t cumulative = 0;
  for (int bin = 0; bin < 64; ++bin) {
    cumulative += magnitudeBins[bin];
    if (!f.medianGradient && cumulative >= (f.samples + 1) / 2)
      f.medianGradient = bin * 16 + 8;
    if (cumulative >= (f.samples * 95 + 99) / 100) {
      f.upperGradient = bin * 16 + 8;
      break;
    }
  }
  if (timing) timing->firstPassUs = micros() - firstStartUs;
  if (f.samples) f.meanGray = (float)f.graySum / f.samples;
  if (f.maxGradient < region.gradientMin) return f;
  const int relativeMinimum = max(region.gradientMin, f.maxGradient / 4);
  const uint32_t secondStartUs = timing ? micros() : 0;
  // Second pass: count orientations, not gradient strengths.
  for (int y = yStart; y <= yEnd; ++y) {
    int halfWidth = region.radius;
    if (region.circle) {
      const int dy = y - region.y;
      halfWidth = (int)sqrtf((float)(region.radius * region.radius - dy * dy));
    }
    const int xStart = max(1, region.x - halfWidth);
    const int xEnd = min(WIDTH - 2, region.x + halfWidth - (region.circle ? 0 : 1));
    for (int x = xStart; x <= xEnd; ++x) {
      const int p = y * WIDTH + x;
      int gx, gy;
      gradientAt(pixels, p, gx, gy);
      if (abs(gx) + abs(gy) < relativeMinimum) continue;
      if (projected) {
        ++liveProjection.bucket[projectionBucket(gx, gy, region.angleTolerance)];
        ++f.edges;
        continue;
      }
      float angle = atan2f((float)gy, (float)gx) * 57.2957795f;
      if (angle < 0) angle += 180.0f;
      if (angle >= 180.0f) angle -= 180.0f;
      int bin = (int)(angle / 10.0f);
      if (bin >= BINS) bin = BINS - 1;
      ++f.hist[bin];
      f.angleSum[bin] += angle;
      ++f.edges;
    }
  }
  if (timing) timing->secondPassUs = micros() - secondStartUs;
  // A thin rail or roof line may occupy only a small fraction of a large
  // cell. Requiring a percentage of cell area forced the contrast floor down
  // until weak background texture was counted as edges.
  if (f.edges < 8) return f;
  f.textured = true;
  if (projected) return f; // No live peak extraction in the monitored path.
  const uint32_t peaksStartUs = timing ? micros() : 0;
  selectDominantPeaks(f);
  if (timing) timing->peaksUs = micros() - peaksStartUs;
  return f;
}

float angleDistance(float a, float b) {
  const float difference = fabsf(a - b);
  return min(difference, 180.0f - difference);
}

float histogramDistance(const Features &a, const Features &b) {
  if (!a.edges || !b.edges) return 1.0f;
  float distance = 0;
  for (int i = 0; i < BINS; ++i) {
    const uint32_t aWindow = a.hist[(i + BINS - 1) % BINS] + 2 * a.hist[i]
                           + a.hist[(i + 1) % BINS];
    const uint32_t bWindow = b.hist[(i + BINS - 1) % BINS] + 2 * b.hist[i]
                           + b.hist[(i + 1) % BINS];
    distance += fabsf((float)aWindow / (4 * a.edges)
                    - (float)bWindow / (4 * b.edges));
  }
  return 0.5f * distance;
}

float projectionDistance() {
  if (!baselineFeatures.edges || !currentFeatures.edges) return 1.0f;
  const Projection &reference = referenceByTolerance[region.angleTolerance];
  float distance = 0;
  for (int i = 0; i <= MAX_PEAKS; ++i) {
    distance += fabsf((float)reference.bucket[i] / baselineFeatures.edges
                    - (float)liveProjection.bucket[i] / currentFeatures.edges);
  }
  return 0.5f * distance;
}

void prepareReferenceProjection(const uint8_t *pixels) {
  if (!baselineFeatures.quality) return;
  for (int i = 0; i < baselineFeatures.peakCount; ++i) {
    const float radians = baselineFeatures.peakAngle[i] * 0.01745329252f;
    referenceDirection[i].x = (int16_t)lroundf(cosf(radians) * 256);
    referenceDirection[i].y = (int16_t)lroundf(sinf(radians) * 256);
  }
  const int selectedTolerance = region.angleTolerance;
  for (int tolerance = 0; tolerance <= 20; ++tolerance) {
    region.angleTolerance = tolerance;
    analyse(pixels, nullptr, true);
    referenceByTolerance[tolerance] = liveProjection;
  }
  region.angleTolerance = selectedTolerance;
  liveProjection = referenceByTolerance[selectedTolerance];
}

bool insideRegion(int x, int y) {
  if (x <= 0 || x >= WIDTH - 1 || y <= 0 || y >= HEIGHT - 1) return false;
  const int dx = x - region.x, dy = y - region.y;
  return region.circle ? dx * dx + dy * dy <= region.radius * region.radius
                       : abs(dx) < region.radius && abs(dy) < region.radius;
}

bool matchesAnchorDirection(int gx, int gy) {
  const ReferenceDirection &direction = referenceDirection[0];
  const int32_t dot = abs(gx * direction.x + gy * direction.y);
  const int32_t cross = abs(gy * direction.x - gx * direction.y);
  // A small floor tolerates integer gradient noise even when the angle slider is zero.
  return cross * 256 <= dot * toleranceTanQ8[max(3, region.angleTolerance)];
}

void captureEdgeAnchors(const uint8_t *pixels) {
  edgeAnchorCount = 0;
  edgeAnchorsMatched = 0;
  edgeAnchorMatchedMask = 0;
  edgeAnchorScore = 0;
  if (!baselineFeatures.quality) return;
  const int minimumMagnitude = max(1, baselineFeatures.maxGradient / 4);
  const int minimumSeparation = max(5, region.radius / 2);
  for (int anchor = 0; anchor < MAX_EDGE_ANCHORS; ++anchor) {
    int bestMagnitude = -1;
    EdgeAnchor best;
    for (int y = max(1, region.y - region.radius); y <= min(HEIGHT - 2, region.y + region.radius); ++y) {
      for (int x = max(1, region.x - region.radius); x <= min(WIDTH - 2, region.x + region.radius); ++x) {
        if (!insideRegion(x, y)) continue;
        if (anchor && (x - edgeAnchors[0].x) * (x - edgeAnchors[0].x)
                     + (y - edgeAnchors[0].y) * (y - edgeAnchors[0].y)
                     < minimumSeparation * minimumSeparation) continue;
        int gx, gy;
        gradientAt(pixels, y * WIDTH + x, gx, gy);
        const int magnitude = abs(gx) + abs(gy);
        if (magnitude >= minimumMagnitude && magnitude > bestMagnitude && matchesAnchorDirection(gx, gy)) {
          bestMagnitude = magnitude;
          best = {(int16_t)x, (int16_t)y};
        }
      }
    }
    if (bestMagnitude < 0) break;
    edgeAnchors[edgeAnchorCount++] = best;
  }
}

void compareEdgeAnchors(const uint8_t *pixels) {
  edgeAnchorsMatched = 0;
  edgeAnchorMatchedMask = 0;
  edgeAnchorScore = 0;
  if (!edgeAnchorCount || !currentFeatures.textured) return;
  const int minimumMagnitude = max(1, currentFeatures.maxGradient / 4);
  for (int anchor = 0; anchor < edgeAnchorCount; ++anchor) {
    bool found = false;
    for (int dy = -ANCHOR_SEARCH_RADIUS; dy <= ANCHOR_SEARCH_RADIUS && !found; ++dy) {
      for (int dx = -ANCHOR_SEARCH_RADIUS; dx <= ANCHOR_SEARCH_RADIUS; ++dx) {
        const int x = edgeAnchors[anchor].x + dx, y = edgeAnchors[anchor].y + dy;
        if (!insideRegion(x, y)) continue;
        int gx, gy;
        gradientAt(pixels, y * WIDTH + x, gx, gy);
        if (abs(gx) + abs(gy) >= minimumMagnitude && matchesAnchorDirection(gx, gy)) {
          found = true;
          break;
        }
      }
    }
    if (found) {
      ++edgeAnchorsMatched;
      edgeAnchorMatchedMask |= 1U << anchor;
    }
  }
  edgeAnchorScore = (float)(edgeAnchorCount - edgeAnchorsMatched) / edgeAnchorCount;
}

void searchAngleMatches(int reference, uint8_t usedCurrent, int count, float totalDelta,
                        int candidate[MAX_PEAKS], int &bestCount, float &bestDelta,
                        int best[MAX_PEAKS]) {
  if (reference == baselineFeatures.peakCount) {
    if (count > bestCount || (count == bestCount && totalDelta < bestDelta)) {
      bestCount = count;
      bestDelta = totalDelta;
      for (int i = 0; i < MAX_PEAKS; ++i) best[i] = candidate[i];
    }
    return;
  }
  candidate[reference] = -1;
  searchAngleMatches(reference + 1, usedCurrent, count, totalDelta,
                     candidate, bestCount, bestDelta, best);
  for (int c = 0; c < currentFeatures.peakCount; ++c) {
    if (usedCurrent & (1U << c)) continue;
    const float difference = angleDistance(baselineFeatures.peakAngle[reference],
                                           currentFeatures.peakAngle[c]);
    if (difference > region.angleTolerance) continue;
    candidate[reference] = c;
    searchAngleMatches(reference + 1, usedCurrent | (1U << c),
                       count + 1, totalDelta + difference,
                       candidate, bestCount, bestDelta, best);
  }
  candidate[reference] = -1;
}

void updateChange() {
  changeScore = 0;
  edgeTailScore = 0;
  edgeAnchorScore = 0;
  edgeAnchorsMatched = 0;
  edgeAnchorMatchedMask = 0;
  exposureUnreliable = false;
  matchedLiveMask = 0;
  for (int i = 0; i < MAX_PEAKS; ++i) {
    matchedIndex[i] = nearestIndex[i] = -1;
    nearestDelta[i] = 0;
  }
  if (!hasBaseline) {
    changedFrames = clearFrames = 0;
    return;
  }
  // A sudden bright, featureless cell has lost the visual evidence needed to
  // distinguish glare from a pale vehicle. Suspend transitions and report an
  // unknown observation instead of treating lost reference edges as occupied
  // or allowing the existing occupied state to clear.
  if (currentFeatures.samples && baselineFeatures.samples) {
    const float liveWhite = (float)currentFeatures.whitePixels / currentFeatures.samples;
    const float referenceWhite = (float)baselineFeatures.whitePixels / baselineFeatures.samples;
    const bool brightened = currentFeatures.meanGray >= baselineFeatures.meanGray + 25.0f;
    const bool clipped = liveWhite >= 0.40f && liveWhite >= referenceWhite + 0.25f;
    const bool lostDetail = baselineFeatures.textured && !currentFeatures.textured
      && currentFeatures.maxGradient * 2 < baselineFeatures.maxGradient;
    if (brightened && (clipped || lostDetail)) {
      exposureUnreliable = true;
      changedFrames = clearFrames = 0;
      return;
    }
  }
  if (!baselineFeatures.textured && !currentFeatures.textured) {
    changeScore = 0;
  } else if (baselineFeatures.textured != currentFeatures.textured) {
    changeScore = 1;
  } else if (baselineFeatures.quality) {
    changeScore = projectionDistance();
  } else if (!baselineFeatures.quality || !currentFeatures.quality) {
    // Textured cells without a stable dominant direction still have a
    // normalized orientation distribution that can be compared.
    changeScore = histogramDistance(baselineFeatures, currentFeatures);
  } else {
    for (int r = 0; r < baselineFeatures.peakCount; ++r) {
      float best = 181.0f;
      for (int c = 0; c < currentFeatures.peakCount; ++c) {
        const float difference = angleDistance(baselineFeatures.peakAngle[r], currentFeatures.peakAngle[c]);
        if (difference < best) { best = difference; nearestIndex[r] = c; }
      }
      nearestDelta[r] = best;
    }
    int candidate[MAX_PEAKS] = {-1, -1, -1};
    int matches = -1;
    float totalDelta = 1000.0f;
    searchAngleMatches(0, 0, 0, 0, candidate, matches, totalDelta, matchedIndex);
    for (int r = 0; r < baselineFeatures.peakCount; ++r)
      if (matchedIndex[r] >= 0) matchedLiveMask |= 1U << matchedIndex[r];
    const int unionCount = baselineFeatures.peakCount + currentFeatures.peakCount - matches;
    changeScore = 1.0f - (float)matches / unionCount;
  }
  // A weakly oriented background such as carpet may retain much the same
  // angle mix when a narrow object enters. A new concentration of steep edges
  // is another signal. Comparing upper/median gradient ratios reduces the
  // effect of uniform contrast scaling; require a real absolute increase too.
  if (baselineFeatures.textured && currentFeatures.textured &&
      baselineFeatures.samples >= 64 && currentFeatures.samples >= 64 &&
      (!baselineFeatures.quality || baselineFeatures.peakShare[0] < 0.40f) &&
      currentFeatures.upperGradient > baselineFeatures.upperGradient + 40) {
    const float referenceTail = (float)(baselineFeatures.upperGradient + 8)
      / (baselineFeatures.medianGradient + 8);
    const float liveTail = (float)(currentFeatures.upperGradient + 8)
      / (currentFeatures.medianGradient + 8);
    if (liveTail > referenceTail)
      edgeTailScore = 1.0f - referenceTail / liveTail;
    changeScore = max(changeScore, edgeTailScore);
  }
  if (edgeAnchorCount && currentFeatures.textured) {
    compareEdgeAnchors(grayFrame);
    changeScore = max(changeScore, edgeAnchorScore);
  }
  // Compare every frame to the fixed empty background, not the previous frame:
  // a stopped vehicle must remain changed.
  if (changeScore >= region.threshold) {
    ++changedFrames;
    clearFrames = 0;
    if (changedFrames >= 3) changed = true;
  } else if (changeScore < region.threshold * 0.7f) {
    ++clearFrames;
    changedFrames = 0;
    if (clearFrames >= 5) changed = false;
  }
}

bool initCameraAt(int index) {
  WIDTH = RESOLUTIONS[index].width;
  HEIGHT = RESOLUTIONS[index].height;
  PIXELS = (size_t)WIDTH * HEIGHT;
  grayFrame = (uint8_t *)ps_malloc(PIXELS);
  if (!grayFrame) {
    Serial.printf("Cannot allocate %u byte grayscale frame in PSRAM\n", (unsigned)PIXELS);
    return false;
  }
  camera_config_t config = {};
  config.pin_pwdn = PWDN;
  config.pin_reset = RESET;
  config.pin_xclk = XCLK;
  config.pin_sccb_sda = SIOD;
  config.pin_sccb_scl = SIOC;
  config.pin_d0 = D0; config.pin_d1 = D1; config.pin_d2 = D2; config.pin_d3 = D3;
  config.pin_d4 = D4; config.pin_d5 = D5; config.pin_d6 = D6; config.pin_d7 = D7;
  config.pin_vsync = VSYNC;
  config.pin_href = HREF;
  config.pin_pclk = PCLK;
  config.xclk_freq_hz = 20000000;
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = RESOLUTIONS[index].frameSize;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init at %s failed: 0x%x\n", RESOLUTIONS[index].name, err);
    free(grayFrame);
    grayFrame = nullptr;
    return false;
  }
  cameraStarted = true;
  resolutionIndex = index;
  Serial.printf("Camera started at %dx%d\n", WIDTH, HEIGHT);
  return true;
}

bool restartCameraAt(int index) {
  const int previousIndex = resolutionIndex;
  const int previousWidth = WIDTH;
  const int previousHeight = HEIGHT;
  const int previousX = region.x;
  const int previousY = region.y;
  const int previousRadius = region.radius;
  const bool hadCamera = cameraStarted;
  if (cameraStarted) {
    esp_camera_deinit();
    cameraStarted = false;
  }
  free(grayFrame);
  grayFrame = nullptr;
  frameReady = false;
  hasBaseline = false;
  edgeAnchorCount = 0;
  exposureUnreliable = false;
  changed = false;
  changedFrames = clearFrames = 0;
  currentFeatures = Features();
  baselineFeatures = Features();
  measuredFps = 0;
  lastCaptureMs = lastAttemptMs = 0;
  processingUs = analysisUs = captureUs = frameGetUs = frameCopyUs = 0;
  cellTiming = CellBreakdown();
  hasFlaggedDiagnostics = false;
  flaggedDiagnosticUs = 0;
  estimatedCaptureUs = estimatedCellUs = 0;
  if (!initCameraAt(index)) {
    // Keep the web app usable if a larger mode cannot allocate or initialize.
    if (hadCamera && initCameraAt(previousIndex)) {
      region.x = previousX;
      region.y = previousY;
      region.radius = previousRadius;
    }
    return false;
  }
  region.x = constrain((int)lroundf((float)previousX * WIDTH / previousWidth), 0, WIDTH - 1);
  region.y = constrain((int)lroundf((float)previousY * HEIGHT / previousHeight), 0, HEIGHT - 1);
  // A cell is defined in pixels; changing image resolution keeps its size.
  region.radius = previousRadius;
  return true;
}

void captureAndAnalyse() {
  lastAttemptMs = millis();
  ++attemptSequence;
  hasFlaggedDiagnostics = false;
  flaggedDiagnosticUs = 0;
  const uint32_t startUs = micros();
  camera_fb_t *fb = esp_camera_fb_get();
  frameGetUs = micros() - startUs;
  frameCopyUs = 0;
  if (!fb) {
    ++captureFailures;
    captureUs = micros() - startUs;
    analysisUs = 0;
    appendFrameLog(false);
    return;
  }
  if (fb->format != PIXFORMAT_GRAYSCALE || fb->width != WIDTH ||
      fb->height != HEIGHT || fb->len < PIXELS) {
    ++captureFailures;
    esp_camera_fb_return(fb);
    captureUs = micros() - startUs;
    analysisUs = 0;
    appendFrameLog(false);
    return;
  }
  const uint32_t copyStartUs = micros();
  memcpy(grayFrame, fb->buf, PIXELS);
  frameCopyUs = micros() - copyStartUs;
  esp_camera_fb_return(fb);
  captureUs = micros() - startUs; // Camera acquisition plus preview-buffer copy.
  const uint32_t previousMs = lastCaptureMs;
  lastCaptureMs = millis();
  if (previousMs && lastCaptureMs > previousMs) {
    const float instantaneous = 1000.0f / (lastCaptureMs - previousMs);
    measuredFps = measuredFps ? measuredFps * 0.8f + instantaneous * 0.2f : instantaneous;
  }
  ++frameId;
  frameReady = true;
  const uint32_t analysisStartUs = micros();
  currentFeatures = analyse(grayFrame, &cellTiming, hasBaseline && baselineFeatures.quality);
  const uint32_t compareStartUs = micros();
  updateChange();
  cellTiming.compareUs = micros() - compareStartUs;
  analysisUs = micros() - analysisStartUs;
  processingUs = captureUs + analysisUs;
  estimatedCaptureUs = estimatedCaptureUs
    ? estimatedCaptureUs * 0.8f + captureUs * 0.2f : captureUs;
  estimatedCellUs = estimatedCellUs
    ? estimatedCellUs * 0.8f + analysisUs * 0.2f : analysisUs;
  if (hasBaseline && baselineFeatures.quality && changeScore >= region.threshold) {
    const uint32_t diagnosticStartUs = micros();
    flaggedDiagnostics = analyse(grayFrame, nullptr, false);
    flaggedDiagnosticUs = micros() - diagnosticStartUs;
    hasFlaggedDiagnostics = true;
  }
  appendFrameLog(true);
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Railway angle experiment</title>
<style>
body{font:15px system-ui,sans-serif;background:#111923;color:#e7edf2;margin:0;padding:20px}
main{max-width:960px;margin:auto}h1{font-size:1.5rem}p{color:#bcc9d3}
.grid{display:grid;grid-template-columns:minmax(320px,1fr) minmax(280px,1fr);gap:18px}
.panel{background:#1d2a36;border:1px solid #354454;border-radius:10px;padding:16px}
canvas{max-width:100%;height:auto;background:#05090c;border:1px solid #526273}
#view{width:100%;image-rendering:auto;cursor:crosshair}
label{display:block;margin:12px 0}input[type=range]{width:100%}
button,select{background:#31475a;color:white;border:1px solid #688096;border-radius:5px;padding:8px}
input[type=number]{background:#10202c;color:white;border:1px solid #688096;border-radius:5px;padding:7px;width:6em}
button{cursor:pointer}.value{font-variant-numeric:tabular-nums}
#state{font-size:1.3rem;font-weight:bold}.changed{color:#ffb45d}.clear{color:#72d7a2}.unknown{color:#ffd37a}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:7px 5px;border-bottom:1px solid #405161}
th{color:#b7c9d8}.match{color:#72d7a2}.miss{color:#ffb45d}
details{margin-top:16px}summary{cursor:pointer;color:#b7d8f4}
.log-scroll{overflow-x:auto;max-height:400px;overflow-y:auto}
.log-scroll table{font-size:12px;white-space:nowrap}
.log-scroll tr.flagged{background:#563c2d}.log-scroll tr.active{outline:1px solid #ffb45d}.log-scroll tr.unknown{background:#63552b}
a.download{display:inline-block;background:#31475a;color:white;border:1px solid #688096;border-radius:5px;padding:8px;text-decoration:none}
@media(max-width:700px){.grid{grid-template-columns:1fr}}
</style>
</head>
<body><main>
<h1>Railway camera angle experiment</h1>
<p>Sketch version: <strong id="version">loading…</strong></p>
<p>Click the image to place one small sensor cell. Capture an empty-background reference, including a blank area, then put something in the cell. The display compares blank versus textured areas and uses angles when they are present.</p>
<div class="grid"><section class="panel">
<canvas id="view" width="320" height="240"></canvas>
<p id="imageStatus">Waiting for camera…</p>
<label>Camera resolution
<select id="resolution">
<option value="qvga">320 × 240</option>
<option value="vga">640 × 480</option>
<option value="svga">800 × 600</option>
<option value="xga">1024 × 768</option>
</select></label>
<button id="applyResolution">Restart camera at selected resolution</button>
<p>Changing resolution clears the empty-track baseline. Larger grayscale frames may reduce frame rate or fail on an ESP32-CAM.</p>
<label>Shape <select id="shape"><option value="circle">Circle</option><option value="square">Square</option></select></label>
<label>Radius / half-width: <span id="radiusValue">5</span> px <input id="radius" type="range" min="3" max="50" value="5"></label>
<label>Minimum usable contrast: <span id="gradientValue">80</span> <input id="gradient" type="range" min="10" max="500" step="10" value="80"></label>
<label>Angle match tolerance: <span id="toleranceValue">10</span>° <input id="tolerance" type="range" min="0" max="20" step="1" value="10"></label>
<label>Pattern mismatch threshold: <span id="thresholdValue">0.20</span> <input id="threshold" type="range" min="0.05" max="1.00" step="0.01" value="0.20"></label>
<button id="baseline">Capture empty-track baseline</button>
<p>Blank cells are valid references: new texture in one is a change. Moving or resizing the cell, or changing its contrast setting, clears the baseline. A 5 px radius circle or half-width square examines roughly 80 or 100 pixels.</p>
</section><section class="panel">
<div id="state">No baseline</div>
<p id="metrics">Waiting for data…</p>
<h2>Background comparison</h2>
<table><thead><tr><th>Reference angle</th><th>Live measurement</th><th>Difference</th><th>Result</th></tr></thead>
<tbody id="angleRows"><tr><td colspan="4">Capture a background reference to begin.</td></tr></tbody></table>
<p>For an oriented reference, the detector tracks up to three dominant angles. It also stores up to two separated points on the strongest reference edge and looks for a matching gradient within 2 pixels of each point. Blue markers show those points; green means matched and orange means missing. The raw histogram below is recalculated for display and does not affect detection. A blank reference is compared with texture presence. Angles describe the gradient across an edge, so a rail or sleeper line runs 90° from the displayed angle.</p>
<details><summary>Show raw angle histogram</summary>
<canvas id="hist" width="540" height="260"></canvas>
<p>Orange bars: live gradient counts. Blue outlines: background counts. Numbered markers show the selected reference peaks. The live histogram is calculated when this page requests diagnostics; the fast detector uses the reference-direction support shown above.</p>
</details>
<p id="timing"></p>
<h3>Last-frame timing breakdown</h3>
<p id="timingBreakdown">Waiting for a frame…</p>
<p>The camera wait includes acquisition. The frame copy keeps a preview image for this experiment. Cell passes measure gradient strength, then count qualifying angles; peak selection and state comparison follow.</p>
</section></div>
<section class="panel" style="margin-top:18px">
<h2>Detector-only speed estimate</h2>
<label>Cells evaluated per frame <input id="cellCount" type="number" min="1" max="1000" value="50"></label>
<p id="estimate">Waiting for timing samples…</p>
<p>The estimate adds camera capture and preview-buffer copy time to the measured time for each cell. It excludes time spent serving this page, preview transfer, the sketch's 100 ms pacing, and block coordination. The hotspot remains active during capture. Camera timing and memory contention can change with load.</p>
<button id="benchmark">Compare circle and square at this location</button>
<p id="shapeBenchmark">Benchmark has not been run.</p>
<p>This benchmark uses the same centre and radius for both shapes. A circle covers fewer pixels than a square at that radius. Polygon speed depends on how its interior pixels are listed; the experiment does not yet implement polygon cells.</p>
</section>
<section class="panel" style="margin-top:18px">
<h2>Frame comparison log</h2>
<p id="logStatus">Loading log…</p>
<p>Each capture attempt is recorded on the camera. Orange rows met the mismatch threshold; outlined rows reached the reported change state; yellow rows had unreliable image detail, so comparison was suspended. Download the CSV after reproducing a false alarm. It contains the settings, brightness, gradient counts, selected angles and diagnostic angle bins.</p>
<a class="download" href="/log.csv">Download CSV</a> <button id="clearLog">Clear log</button>
<div class="log-scroll"><table><thead><tr><th>Attempt</th><th>Time s</th><th>Reference → live</th><th>Angle / support</th><th>Mean gray</th><th>Edges</th><th>Max gradient</th><th>Score / limit</th><th>Capture µs</th><th>Cell µs</th></tr></thead><tbody id="logRows"></tbody></table></div>
</section>
</main>
<script>
const view=document.querySelector('#view'), vctx=view.getContext('2d');
const hist=document.querySelector('#hist'), hctx=hist.getContext('2d');
const imageStatus=document.querySelector('#imageStatus');
const state=document.querySelector('#state');
const metrics=document.querySelector('#metrics');
const timing=document.querySelector('#timing');
const timingBreakdown=document.querySelector('#timingBreakdown');
const shape=document.querySelector('#shape');
const radius=document.querySelector('#radius');
const gradient=document.querySelector('#gradient');
const threshold=document.querySelector('#threshold');
const tolerance=document.querySelector('#tolerance');
const resolution=document.querySelector('#resolution');
const cellCount=document.querySelector('#cellCount');
const estimate=document.querySelector('#estimate');
const shapeBenchmark=document.querySelector('#shapeBenchmark');
const controls=['shape','radius','gradient','tolerance','threshold'];
let x=160,y=120,frameBusy=false,statsBusy=false,currentResolution='qvga';
let latestStats=null;
const angleRows=document.querySelector('#angleRows');
const logRows=document.querySelector('#logRows');
const logStatus=document.querySelector('#logStatus');
let logBusy=false;
function updateLabels(){
  for(const id of ['radius','gradient','tolerance','threshold'])
    document.getElementById(id+'Value').textContent=document.getElementById(id).value;
}
async function post(path,values={}){
  const response=await fetch(path,{method:'POST',body:new URLSearchParams(values)});
  if(!response.ok)throw Error(await response.text());
}
async function saveRegion(){
  updateLabels();
  await post('/region',{x,y,radius:radius.value,shape:shape.value,
    gradient:gradient.value,tolerance:tolerance.value,threshold:threshold.value});
}
view.addEventListener('click',async event=>{
  const rect=view.getBoundingClientRect();
  x=Math.max(0,Math.min(view.width-1,Math.round((event.clientX-rect.left)*view.width/rect.width)));
  y=Math.max(0,Math.min(view.height-1,Math.round((event.clientY-rect.top)*view.height/rect.height)));
  try{await saveRegion()}catch(e){imageStatus.textContent=e.message}
});
for(const id of controls)document.getElementById(id).addEventListener('change',()=>{
  saveRegion().catch(e=>imageStatus.textContent=e.message);
});
document.querySelector('#baseline').onclick=async()=>{
  try{await post('/baseline')}catch(e){imageStatus.textContent=e.message}
};
document.querySelector('#applyResolution').onclick=async()=>{
  const button=document.querySelector('#applyResolution');
  button.disabled=true;
  imageStatus.textContent='Restarting camera…';
  try {
    const response=await fetch('/resolution',{method:'POST',
      body:new URLSearchParams({resolution:resolution.value})});
    const data=await response.json();
    if(!response.ok)throw Error(data.error||'Camera restart failed');
    currentResolution=data.resolution;
    x=data.x;y=data.y;radius.value=data.radius;updateLabels();
    view.width=data.width;view.height=data.height;
    imageStatus.textContent='Camera restarted at '+data.width+' × '+data.height+'; capture a new baseline';
    await loadFrame();await loadStats();
  } catch(e) {
    imageStatus.textContent=e.message;
    resolution.value=currentResolution;
    await loadStats();
  }
  button.disabled=false;
};
function overlay(){
  vctx.strokeStyle='#ffb45d';vctx.lineWidth=2;vctx.beginPath();
  const r=Number(radius.value);
  if(shape.value==='circle')vctx.arc(x,y,r,0,Math.PI*2);
  else vctx.rect(x-r,y-r,2*r,2*r);
  vctx.stroke();vctx.beginPath();vctx.moveTo(x-5,y);vctx.lineTo(x+5,y);
  vctx.moveTo(x,y-5);vctx.lineTo(x,y+5);vctx.stroke();
  if(latestStats?.anchors)for(const anchor of latestStats.anchors){
    vctx.beginPath();vctx.arc(anchor.x,anchor.y,3,0,Math.PI*2);
    vctx.strokeStyle=anchor.matched?'#84e3b5':'#ffb45d';vctx.lineWidth=2;vctx.stroke();
  }
}
async function loadFrame(){
  if(frameBusy)return;frameBusy=true;
  try{
    const response=await fetch('/frame',{cache:'no-store'});
    if(!response.ok)throw Error(await response.text());
    const width=Number(response.headers.get('X-Width'));
    const height=Number(response.headers.get('X-Height'));
    const pixels=new Uint8Array(await response.arrayBuffer());
    if(!width||!height||pixels.length!==width*height)throw Error('Unexpected frame size');
    if(view.width!==width||view.height!==height){view.width=width;view.height=height}
    const image=vctx.createImageData(width,height);
    for(let i=0,j=0;i<pixels.length;i++,j+=4){
      image.data[j]=image.data[j+1]=image.data[j+2]=pixels[i];image.data[j+3]=255;
    }
    vctx.putImageData(image,0,0);overlay();
    imageStatus.textContent='Live grayscale view, '+width+' × '+height;
  }catch(e){imageStatus.textContent=e.message}
  frameBusy=false;
}
function drawHistogram(data){
  hctx.clearRect(0,0,540,260);
  hctx.fillStyle='#c5d4de';hctx.font='12px sans-serif';
  const max=Math.max(0.01,...data.current,...data.baseline);
  const reference=selectedBins(data.baseline_mask,data.baseline);
  const ranks=new Array(18).fill(0);
  reference.forEach((bin,index)=>{ranks[bin]=index+1});
  for(let i=0;i<18;i++){
    const bx=31+i*28, bw=20;
    const bh=data.baseline[i]/max*190, ch=data.current[i]/max*190;
    if(ranks[i]){
      hctx.fillStyle='rgba(120,183,255,0.16)';
      hctx.fillRect(bx-3,29,26,190);
    }
    hctx.strokeStyle='#78b7ff';hctx.lineWidth=2;
    hctx.strokeRect(bx,218-bh,bw,bh);
    hctx.fillStyle='#ffb45d';hctx.fillRect(bx+4,218-ch,bw-8,ch);
    if(i%3===0){hctx.fillStyle='#c5d4de';hctx.fillText(i*10+'°',bx,242)}
    if(ranks[i]){
      hctx.beginPath();hctx.arc(bx+10,15,10,0,Math.PI*2);
      hctx.fillStyle='#78b7ff';hctx.fill();
      hctx.fillStyle='#102132';hctx.font='bold 12px sans-serif';
      hctx.textAlign='center';hctx.fillText(String(ranks[i]),bx+10,19);
      hctx.textAlign='start';hctx.font='12px sans-serif';
    }
  }
}
function selectedBins(mask,histogram){
  const bins=[];
  for(let i=0;i<18;i++)if(mask & (1<<i))bins.push(i);
  return bins.sort((a,b)=>histogram[b]-histogram[a]||a-b);
}
function addAngleRow(reference,live,difference,result,matched){
  const row=document.createElement('tr');
  for(const value of [reference,live,difference,result]){
    const cell=document.createElement('td');cell.textContent=value;row.appendChild(cell);
  }
  row.lastChild.className=matched?'match':'miss';
  angleRows.appendChild(row);
}
function angleLabel(angle,share,prefix=''){
  return prefix+angle.toFixed(1)+'° ('+(share*100).toFixed(0)+'%)';
}
function renderAngleComparison(d){
  angleRows.replaceChildren();
  if(!d.has_baseline){
    addAngleRow('—','—','—','Capture a reference',false);
    return;
  }
  if(d.exposure_unreliable){
    addAngleRow('Reference retained','Washed-out image','—','Comparison suspended',false);
    return;
  }
  if(!d.reference_textured){
    addAngleRow('Blank background',d.live_textured?'Texture present':'Blank',
      '—',d.live_textured?'New texture':'Same blank background',!d.live_textured);
    return;
  }
  if(!d.live_textured){
    addAngleRow('Texture present','Blank','—','Background texture disappeared',false);
    return;
  }
  if(d.projection_mode){
    for(let i=0;i<d.reference_angles.length;i++){
      const before=d.reference_buckets[i],now=d.live_buckets[i];
      const stable=Math.abs(now-before)<0.10;
      addAngleRow((i+1)+'. '+d.reference_angles[i].toFixed(1)+'°',
        (now*100).toFixed(0)+'% nearby',((now-before)*100).toFixed(0)+' percentage points',
        stable?'Stable':'Support shifted (from '+(before*100).toFixed(0)+'%)',stable);
    }
    const other=d.reference_buckets.length-1;
    const otherStable=Math.abs(d.live_buckets[other]-d.reference_buckets[other])<0.10;
    addAngleRow('Other directions',(d.live_buckets[other]*100).toFixed(0)+'%',
      ((d.live_buckets[other]-d.reference_buckets[other])*100).toFixed(0)+' percentage points',
      otherStable?'Stable':'Support shifted (from '+(d.reference_buckets[other]*100).toFixed(0)+'%)',
      otherStable);
    return;
  }
  if(!d.reference_has_angles||!d.quality){
    addAngleRow(d.reference_has_angles?'Reference angles':'Texture without one dominant angle',
      d.quality?'Live angles':'Texture without one dominant angle','—',
      'Compared normalized angle distributions',d.score<d.threshold);
    return;
  }
  for(let r=0;r<d.reference_angles.length;r++){
    const matched=d.match_indices[r]>=0;
    const liveIndex=matched?d.match_indices[r]:d.nearest_indices[r];
    const live=liveIndex>=0?d.live_angles[liveIndex]:null;
    const difference=matched
      ?Math.min(Math.abs(d.reference_angles[r]-live),180-Math.abs(d.reference_angles[r]-live))
      :d.nearest_deltas[r];
    addAngleRow(angleLabel(d.reference_angles[r],d.reference_shares[r],(r+1)+'. '),
      live===null?'—':angleLabel(live,d.live_shares[liveIndex]),
      live===null?'—':difference.toFixed(1)+'°',
      matched?'Match':'No one-to-one match',matched);
  }
  for(let c=0;c<d.live_angles.length;c++)
    if(!(d.matched_live_mask&(1<<c)))
      addAngleRow('—',angleLabel(d.live_angles[c],d.live_shares[c]),'—','Unmatched live angle',false);
}
function renderEstimate(){
  if(!latestStats||!latestStats.estimated_capture_us||!latestStats.estimated_cell_us){
    estimate.textContent='Waiting for a fresh capture and cell measurement…';
    return;
  }
  const count=Math.max(1,Math.min(1000,Number(cellCount.value)||1));
  const capture=latestStats.estimated_capture_us;
  const cell=latestStats.estimated_cell_us;
  const total=capture+count*cell;
  estimate.textContent='Capture + copy: '+(capture/1000).toFixed(1)+' ms'
    +' · one cell: '+cell.toFixed(0)+' µs'
    +' · '+count+' cells: '+(total/1000).toFixed(1)+' ms/frame'
    +' → about '+(1000000/total).toFixed(1)+' frames/s without web waits or pacing.';
}
cellCount.addEventListener('input',renderEstimate);
document.querySelector('#benchmark').onclick=async()=>{
  shapeBenchmark.textContent='Measuring both shapes…';
  try{
    const response=await fetch('/benchmark',{cache:'no-store'});
    if(!response.ok)throw Error(await response.text());
    const d=await response.json();
    shapeBenchmark.textContent='Circle: '+d.circle_us+' µs for '+d.circle_samples
      +' pixels · square: '+d.square_us+' µs for '+d.square_samples
      +' pixels. These are feature-extraction times on the current frame.';
  }catch(e){shapeBenchmark.textContent=e.message}
};
async function loadStats(){
  if(statsBusy)return;statsBusy=true;
  try{
    const response=await fetch('/stats',{cache:'no-store'});
    const d=await response.json();
    latestStats=d;
    renderEstimate();
    document.querySelector('#version').textContent=d.version;
    currentResolution=d.resolution;
    if(!resolution.dataset.initialized){resolution.value=d.resolution;resolution.dataset.initialized='1'}
    state.textContent=!d.ready?'Camera unavailable':d.age_ms>1000?'Camera frame stale':!d.has_baseline?'No baseline':d.exposure_unreliable?'IMAGE UNRELIABLE':d.changed?'CHANGE DETECTED':'Background-like';
    state.className=d.exposure_unreliable?'unknown':!d.ready||d.age_ms>1000||d.changed?'changed':'clear';
    metrics.textContent=d.exposure_unreliable
      ?'Possible overexposure: comparison and state transitions suspended until usable detail returns'
      :d.has_baseline
      ?'Pattern mismatch '+d.score.toFixed(3)+' / change threshold '+d.threshold.toFixed(2)
        +(d.edge_tail_score>0?' · new-edge score '+d.edge_tail_score.toFixed(3):'')
        +(d.anchor_count?' · edge points '+d.anchors_matched+'/'+d.anchor_count+' · position score '+d.anchor_score.toFixed(3):' · no stable edge points')
        +(d.projection_mode?' · reference-direction tolerance ±'+d.tolerance+'°':'')
      :'Capture a blank or textured background reference';
    renderAngleComparison(d);
    timing.textContent=(d.live_textured?'Textured':'Blank')+' live cell · '
      +d.samples+' sampled pixels · '+d.edges+' strong gradients'
      +' · strongest gradient '+d.max_gradient+' / contrast floor '+d.gradient_min
      +' · near-white pixels '+(d.white_fraction*100).toFixed(0)+'%'
      +' · '+d.capture_us+' µs capture + copy · '+d.analysis_us+' µs one cell'
      +' · '+d.processing_us+' µs total · '+d.fps.toFixed(1)
      +' fps · '+d.capture_failures+' capture failures';
    timingBreakdown.textContent='Camera wait '+d.frame_get_us+' µs · frame copy '
      +d.frame_copy_us+' µs · first cell pass '+d.first_pass_us
      +' µs · angle pass '+d.second_pass_us+' µs · peak selection '
      +d.peaks_us+' µs · comparison '+d.compare_us+' µs';
    drawHistogram(d);
  }catch(e){timing.textContent=e.message}
  statsBusy=false;
}
async function loadLog(){
  if(logBusy)return;logBusy=true;
  try{
    const response=await fetch('/log/recent',{cache:'no-store'});
    if(!response.ok)throw Error(await response.text());
    const d=await response.json();
    logStatus.textContent=d.capacity
      ?d.count+' / '+d.capacity+' records retained; oldest records are overwritten when full.'
      :'Logging unavailable: this camera needs PSRAM.';
    logRows.replaceChildren();
    for(const e of d.rows.slice().reverse()){
      const row=document.createElement('tr');
      if(e.ok&&e.reference!=='none'&&e.score>=e.threshold)row.classList.add('flagged');
      if(e.exposure_unreliable)row.classList.add('unknown');
      if(e.changed)row.classList.add('active');
      const values=[e.attempt,(e.uptime_ms/1000).toFixed(1),
        e.ok?e.reference+' → '+e.live:'Capture failed',
        e.projection_mode?e.reference_angle.toFixed(1)+'° → '+(e.live_support*100).toFixed(0)+'%'
          :e.reference==='oriented'||e.live==='oriented'
            ?e.reference_angle.toFixed(1)+' → '+e.live_angle.toFixed(1):'—',
        e.ok?e.reference_mean_gray.toFixed(1)+' → '+e.mean_gray.toFixed(1):'—',
        e.edges,e.max_gradient,e.exposure_unreliable?'Unreliable':e.reference==='none'?'—':e.score.toFixed(3)+' / '+e.threshold.toFixed(2)+(e.edge_tail_score?'; edge '+e.edge_tail_score.toFixed(2):'')+(e.anchor_count?'; points '+e.anchors_matched+'/'+e.anchor_count:''),
        e.capture_us,e.cell_us];
      for(const value of values){const cell=document.createElement('td');cell.textContent=value;row.appendChild(cell)}
      logRows.appendChild(row);
    }
  }catch(e){logStatus.textContent=e.message}
  logBusy=false;
}
document.querySelector('#clearLog').onclick=async()=>{
  try{await post('/log/clear');await loadLog()}catch(e){logStatus.textContent=e.message}
};
updateLabels();loadFrame();loadStats();loadLog();
setInterval(loadFrame,1000);setInterval(loadStats,500);setInterval(loadLog,1500);
</script></body></html>
)HTML";

void handleFrame() {
  if (!frameReady) {
    server.send(503, "text/plain", "No camera frame yet");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Width", String(WIDTH));
  server.sendHeader("X-Height", String(HEIGHT));
  server.setContentLength(PIXELS);
  server.send(200, "application/octet-stream", "");
  WiFiClient client = server.client();
  size_t sent = 0;
  while (sent < PIXELS && client.connected()) {
    const size_t chunk = min((size_t)1024, PIXELS - sent);
    const size_t count = client.write(grayFrame + sent, chunk);
    if (!count) break;
    sent += count;
  }
}

void handleStats() {
  // Full angles and histogram are only needed for the diagnostic page. They
  // are deliberately outside the timed per-frame detector path.
  const Features displayFeatures = frameReady && hasBaseline && baselineFeatures.quality
    ? analyse(grayFrame, nullptr, false) : currentFeatures;
  String out;
  out.reserve(1200);
  out = "{\"frame\":" + String(frameId)
      + ",\"version\":\"" CAMERA_ANGLE_EXPERIMENT_VERSION "\""
      + ",\"resolution\":\"" + RESOLUTIONS[resolutionIndex].name + "\""
      + ",\"width\":" + String(WIDTH)
      + ",\"height\":" + String(HEIGHT)
      + ",\"ready\":" + (frameReady ? "true" : "false")
      + ",\"fps\":" + String(measuredFps, 2)
      + ",\"processing_us\":" + String(processingUs)
      + ",\"analysis_us\":" + String(analysisUs)
      + ",\"capture_us\":" + String(captureUs)
      + ",\"frame_get_us\":" + String(frameGetUs)
      + ",\"frame_copy_us\":" + String(frameCopyUs)
      + ",\"first_pass_us\":" + String(cellTiming.firstPassUs)
      + ",\"second_pass_us\":" + String(cellTiming.secondPassUs)
      + ",\"peaks_us\":" + String(cellTiming.peaksUs)
      + ",\"compare_us\":" + String(cellTiming.compareUs)
      + ",\"estimated_capture_us\":" + String(estimatedCaptureUs, 1)
      + ",\"estimated_cell_us\":" + String(estimatedCellUs, 1)
      + ",\"age_ms\":" + String(frameReady ? millis() - lastCaptureMs : 0)
      + ",\"capture_failures\":" + String(captureFailures)
      + ",\"log_count\":" + String(logCount)
      + ",\"log_capacity\":" + String(logCapacity)
      + ",\"has_baseline\":" + (hasBaseline ? "true" : "false")
      + ",\"exposure_unreliable\":" + (exposureUnreliable ? "true" : "false")
      + ",\"changed\":" + (changed ? "true" : "false")
      + ",\"score\":" + String(changeScore, 4)
      + ",\"edge_tail_score\":" + String(edgeTailScore, 4)
      + ",\"anchor_score\":" + String(edgeAnchorScore, 4)
      + ",\"anchor_count\":" + String(edgeAnchorCount)
      + ",\"anchors_matched\":" + String(edgeAnchorsMatched)
      + ",\"threshold\":" + String(region.threshold, 3)
      + ",\"tolerance\":" + String(region.angleTolerance)
      + ",\"projection_mode\":" + (hasBaseline && baselineFeatures.quality ? "true" : "false")
      + ",\"quality\":" + (displayFeatures.quality ? "true" : "false")
      + ",\"reference_has_angles\":" + (hasBaseline && baselineFeatures.quality ? "true" : "false")
      + ",\"reference_textured\":" + (hasBaseline && baselineFeatures.textured ? "true" : "false")
      + ",\"live_textured\":" + (currentFeatures.textured ? "true" : "false")
      + ",\"max_gradient\":" + String(displayFeatures.maxGradient)
      + ",\"gradient_min\":" + String(region.gradientMin)
      + ",\"current_mask\":" + String(displayFeatures.dominantMask)
      + ",\"baseline_mask\":" + String(hasBaseline ? baselineFeatures.dominantMask : 0)
      + ",\"samples\":" + String(displayFeatures.samples)
      + ",\"edges\":" + String(displayFeatures.edges)
      + ",\"white_fraction\":" + String(displayFeatures.samples
          ? (float)displayFeatures.whitePixels / displayFeatures.samples : 0, 3)
      + ",\"median_gradient\":" + String(displayFeatures.medianGradient)
      + ",\"upper_gradient\":" + String(displayFeatures.upperGradient)
      + ",\"current\":[";
  for (int i = 0; i < BINS; ++i) {
    if (i) out += ',';
    out += String(displayFeatures.edges ?
      (float)displayFeatures.hist[i] / displayFeatures.edges : 0, 4);
  }
  out += "],\"baseline\":[";
  for (int i = 0; i < BINS; ++i) {
    if (i) out += ',';
    out += String(hasBaseline && baselineFeatures.edges ?
      (float)baselineFeatures.hist[i] / baselineFeatures.edges : 0, 4);
  }
  out += "],\"anchors\":[";
  for (int i = 0; i < edgeAnchorCount; ++i) {
    if (i) out += ',';
    out += "{\"x\":" + String(edgeAnchors[i].x) + ",\"y\":" + String(edgeAnchors[i].y)
        + ",\"matched\":" + ((edgeAnchorMatchedMask & (1U << i)) ? "true" : "false") + "}";
  }
  out += "],\"reference_angles\":[";
  for (int i = 0; i < (hasBaseline ? baselineFeatures.peakCount : 0); ++i) {
    if (i) out += ',';
    out += String(baselineFeatures.peakAngle[i], 1);
  }
  out += "],\"live_angles\":[";
  for (int i = 0; i < displayFeatures.peakCount; ++i) {
    if (i) out += ',';
    out += String(displayFeatures.peakAngle[i], 1);
  }
  out += "],\"reference_shares\":[";
  for (int i = 0; i < (hasBaseline ? baselineFeatures.peakCount : 0); ++i) {
    if (i) out += ',';
    out += String(baselineFeatures.peakShare[i], 3);
  }
  out += "],\"live_shares\":[";
  for (int i = 0; i < displayFeatures.peakCount; ++i) {
    if (i) out += ',';
    out += String(displayFeatures.peakShare[i], 3);
  }
  out += "],\"match_indices\":[";
  for (int i = 0; i < (hasBaseline ? baselineFeatures.peakCount : 0); ++i) {
    if (i) out += ',';
    out += String(matchedIndex[i]);
  }
  out += "],\"nearest_indices\":[";
  for (int i = 0; i < (hasBaseline ? baselineFeatures.peakCount : 0); ++i) {
    if (i) out += ',';
    out += String(nearestIndex[i]);
  }
  out += "],\"nearest_deltas\":[";
  for (int i = 0; i < (hasBaseline ? baselineFeatures.peakCount : 0); ++i) {
    if (i) out += ',';
    out += String(nearestDelta[i], 1);
  }
  out += "],\"reference_buckets\":[";
  for (int i = 0; i <= MAX_PEAKS; ++i) {
    if (i) out += ',';
    out += String(hasBaseline && baselineFeatures.quality && baselineFeatures.edges
      ? (float)referenceByTolerance[region.angleTolerance].bucket[i] / baselineFeatures.edges : 0, 3);
  }
  out += "],\"live_buckets\":[";
  for (int i = 0; i <= MAX_PEAKS; ++i) {
    if (i) out += ',';
    out += String(hasBaseline && baselineFeatures.quality && currentFeatures.edges
      ? (float)liveProjection.bucket[i] / currentFeatures.edges : 0, 3);
  }
  out += "],\"matched_live_mask\":" + String(matchedLiveMask) + "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

ShapeBench benchmarkShape(bool circle) {
  region.circle = circle;
  const bool projected = hasBaseline && baselineFeatures.quality;
  const int repeats = region.radius <= 10 ? 100 : (region.radius <= 30 ? 20 : 5);
  volatile uint32_t sink = 0;
  Features warmup = analyse(grayFrame, nullptr, projected);
  sink += warmup.edges;
  const uint32_t startUs = micros();
  uint32_t samples = 0;
  for (int i = 0; i < repeats; ++i) {
    Features result = analyse(grayFrame, nullptr, projected);
    samples = result.samples;
    sink += result.edges;
  }
  const uint32_t elapsed = micros() - startUs;
  (void)sink;
  return {elapsed / repeats, samples};
}

void handleBenchmark() {
  if (!frameReady || millis() - lastCaptureMs > 2000) {
    server.send(409, "text/plain", "Need a fresh frame before benchmarking");
    return;
  }
  const bool selectedShape = region.circle;
  const Projection savedProjection = liveProjection;
  const ShapeBench circle = benchmarkShape(true);
  const ShapeBench square = benchmarkShape(false);
  region.circle = selectedShape;
  liveProjection = savedProjection;
  const String out = "{\"circle_us\":" + String(circle.microseconds)
      + ",\"circle_samples\":" + String(circle.samples)
      + ",\"square_us\":" + String(square.microseconds)
      + ",\"square_samples\":" + String(square.samples) + "}";
  server.send(200, "application/json", out);
}

void handleRegion() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("radius") ||
      !server.hasArg("shape") || !server.hasArg("gradient") ||
      !server.hasArg("threshold") || !server.hasArg("tolerance")) {
    server.send(400, "text/plain", "Missing region setting");
    return;
  }
  const int nx = server.arg("x").toInt();
  const int ny = server.arg("y").toInt();
  const int nr = server.arg("radius").toInt();
  const int ng = server.arg("gradient").toInt();
  const String shape = server.arg("shape");
  const float nt = server.arg("threshold").toFloat();
  const int angleTolerance = server.arg("tolerance").toInt();
  if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT ||
      nr < 3 || nr > 50 || ng < 10 || ng > 500 ||
      nt < 0.05f || nt > 1.0f || angleTolerance < 0 || angleTolerance > 20 ||
      (shape != "circle" && shape != "square")) {
    server.send(400, "text/plain", "Invalid region setting");
    return;
  }
  if (nx != region.x || ny != region.y || nr != region.radius ||
      ng != region.gradientMin || (shape == "circle") != region.circle) {
    hasBaseline = false;
    edgeAnchorCount = 0;
    changed = false;
    changedFrames = clearFrames = 0;
    estimatedCellUs = 0;
  }
  if (nx != region.x || ny != region.y || nr != region.radius ||
      ng != region.gradientMin || (shape == "circle") != region.circle ||
      nt != region.threshold || angleTolerance != region.angleTolerance)
    ++configRevision;
  region.x = nx; region.y = ny; region.radius = nr;
  region.gradientMin = ng; region.circle = shape == "circle";
  region.threshold = nt;
  region.angleTolerance = angleTolerance;
  if (frameReady) {
    const uint32_t startUs = micros();
    currentFeatures = analyse(grayFrame, &cellTiming, hasBaseline && baselineFeatures.quality);
    const uint32_t compareStartUs = micros();
    updateChange();
    cellTiming.compareUs = micros() - compareStartUs;
    analysisUs = micros() - startUs;
  }
  server.send(200, "text/plain", "OK");
}

void handleResolution() {
  const String requested = server.arg("resolution");
  int index = -1;
  for (int i = 0; i < RESOLUTION_COUNT; ++i) {
    if (requested == RESOLUTIONS[i].name) index = i;
  }
  if (index < 0) {
    server.send(400, "application/json", "{\"error\":\"Unsupported resolution\"}");
    return;
  }
  if (index != resolutionIndex || !cameraStarted) {
    if (!restartCameraAt(index)) {
      server.send(500, "application/json",
                  "{\"error\":\"Camera could not start at that resolution; previous mode restored if possible\"}");
      return;
    }
    ++configRevision;
  }
  const String out = "{\"resolution\":\"" + String(RESOLUTIONS[resolutionIndex].name)
      + "\",\"width\":" + String(WIDTH) + ",\"height\":" + String(HEIGHT)
      + ",\"x\":" + String(region.x) + ",\"y\":" + String(region.y)
      + ",\"radius\":" + String(region.radius) + "}";
  server.send(200, "application/json", out);
}

void handleBaseline() {
  if (!frameReady || millis() - lastCaptureMs > 2000) {
    server.send(409, "text/plain", "Need a fresh camera frame before capturing the reference");
    return;
  }
  // Recalibration must always measure actual angles, even if the preceding
  // monitoring frame used only reference-direction projections.
  baselineFeatures = analyse(grayFrame, nullptr, false);
  currentFeatures = baselineFeatures;
  prepareReferenceProjection(grayFrame);
  captureEdgeAnchors(grayFrame);
  ++baselineRevision;
  hasBaseline = true;
  changed = false;
  changedFrames = clearFrames = 0;
  updateChange();
  server.send(200, "text/plain", "OK");
}

const FrameLog &logAt(uint16_t chronologicalIndex) {
  return frameLog[(logHead + logCapacity - logCount + chronologicalIndex) % logCapacity];
}

void handleRecentLog() {
  String out;
  out.reserve(7000);
  out = "{\"count\":" + String(logCount) + ",\"capacity\":" + String(logCapacity) + ",\"rows\":[";
  const uint16_t first = logCount > 30 ? logCount - 30 : 0;
  for (uint16_t i = first; i < logCount; ++i) {
    const FrameLog &e = logAt(i);
    if (i > first) out += ',';
    out += "{\"attempt\":" + String(e.attempt)
      + ",\"uptime_ms\":" + String(e.uptimeMs)
      + ",\"ok\":" + String(e.captureOk)
      + ",\"reference\":\"" + kindName(e.referenceKind) + "\""
      + ",\"live\":\"" + kindName(e.liveKind) + "\""
      + ",\"score\":" + String(e.score, 3)
      + ",\"edge_tail_score\":" + String(e.edgeTailScore, 3)
      + ",\"anchor_score\":" + String(e.anchorScore, 3)
      + ",\"anchor_count\":" + String(e.anchorCount)
      + ",\"anchors_matched\":" + String(e.anchorsMatched)
      + ",\"anchor_matched_mask\":" + String(e.anchorMatchedMask)
      + ",\"threshold\":" + String(e.threshold, 3)
      + ",\"changed\":" + String(e.changed)
      + ",\"exposure_unreliable\":" + String(e.exposureUnreliable)
      + ",\"white_fraction\":" + String(e.samples ? (float)e.liveWhitePixels / e.samples : 0, 3)
      + ",\"mean_gray\":" + String(e.meanGray, 1)
      + ",\"reference_mean_gray\":" + String(e.referenceMeanGray, 1)
      + ",\"max_gradient\":" + String(e.maxGradient)
      + ",\"edges\":" + String(e.edges)
      + ",\"capture_us\":" + String(e.captureUs)
      + ",\"cell_us\":" + String(e.cellUs)
      + ",\"reference_angle\":" + String(e.referenceAngles[0], 1)
      + ",\"live_angle\":" + String(e.liveAngles[0], 1)
      + ",\"projection_mode\":" + String(e.projectionMode)
      + ",\"live_support\":" + String(e.edges ? (float)e.liveBucket[0] / e.edges : 0, 3)
      + "}";
  }
  out += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void csvValue(String &line, const String &value) { line += ','; line += value; }

void handleLogCsv() {
  if (!logCapacity) {
    server.send(503, "text/plain", "No PSRAM available for frame logging");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", "attachment; filename=railway-camera-log.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  String header = "version,attempt,frame,uptime_ms,capture_ok,config_revision,baseline_revision,has_reference,x,y,radius,shape,contrast_floor,angle_tolerance_deg,change_threshold,reference_kind,live_kind,reference_mean_gray,live_mean_gray,live_min_gray,live_max_gray,live_max_gradient,samples,reference_edges,live_edges,reference_white_pixels,live_white_pixels,exposure_unreliable,reference_median_gradient,reference_upper_gradient,live_median_gradient,live_upper_gradient,edge_tail_score,anchor_score,anchor_count,anchors_matched,mismatch_score,raw_mismatch,changed,enter_frames,clear_frames,capture_us,cell_us,diagnostic_us,frame_get_us,frame_copy_us,first_pass_us,second_pass_us,peaks_us,compare_us,projection_mode";
  header += ",anchor_matched_mask,anchor_1_x,anchor_1_y,anchor_2_x,anchor_2_y";
  for (int i = 0; i <= MAX_PEAKS; ++i) {
    header += ",reference_bucket_" + String(i) + ",live_bucket_" + String(i);
  }
  for (int i = 0; i < MAX_PEAKS; ++i) {
    header += ",reference_angle_" + String(i + 1) + ",reference_share_" + String(i + 1)
           + ",live_angle_" + String(i + 1) + ",live_share_" + String(i + 1);
  }
  for (int i = 0; i < BINS; ++i) header += ",reference_bin_" + String(i * 10);
  for (int i = 0; i < BINS; ++i) header += ",live_bin_" + String(i * 10);
  header += "\r\n";
  server.sendContent(header);
  for (uint16_t i = 0; i < logCount; ++i) {
    const FrameLog &e = logAt(i);
    String line;
    line.reserve(650);
    line = CAMERA_ANGLE_EXPERIMENT_VERSION;
    csvValue(line, String(e.attempt)); csvValue(line, String(e.frame));
    csvValue(line, String(e.uptimeMs)); csvValue(line, String(e.captureOk));
    csvValue(line, String(e.configRevision)); csvValue(line, String(e.baselineRevision));
    csvValue(line, String(e.hasReference)); csvValue(line, String(e.x));
    csvValue(line, String(e.y)); csvValue(line, String(e.radius));
    csvValue(line, e.shape ? "square" : "circle");
    csvValue(line, String(e.contrastFloor)); csvValue(line, String(e.tolerance));
    csvValue(line, String(e.threshold, 3));
    csvValue(line, kindName(e.referenceKind)); csvValue(line, kindName(e.liveKind));
    csvValue(line, String(e.referenceMeanGray, 2)); csvValue(line, String(e.meanGray, 2));
    csvValue(line, String(e.minGray)); csvValue(line, String(e.maxGray));
    csvValue(line, String(e.maxGradient)); csvValue(line, String(e.samples));
    csvValue(line, String(e.referenceEdges)); csvValue(line, String(e.edges));
    csvValue(line, String(e.referenceWhitePixels)); csvValue(line, String(e.liveWhitePixels));
    csvValue(line, String(e.exposureUnreliable));
    csvValue(line, String(e.referenceMedianGradient)); csvValue(line, String(e.referenceUpperGradient));
    csvValue(line, String(e.liveMedianGradient)); csvValue(line, String(e.liveUpperGradient));
    csvValue(line, String(e.edgeTailScore, 4));
    csvValue(line, String(e.anchorScore, 4));
    csvValue(line, String(e.anchorCount));
    csvValue(line, String(e.anchorsMatched));
    csvValue(line, String(e.score, 4));
    csvValue(line, String(e.captureOk && e.hasReference && e.score >= e.threshold));
    csvValue(line, String(e.changed)); csvValue(line, String(e.enterFrames));
    csvValue(line, String(e.clearFrames)); csvValue(line, String(e.captureUs));
    csvValue(line, String(e.cellUs));
    csvValue(line, String(e.diagnosticUs));
    csvValue(line, String(e.getUs)); csvValue(line, String(e.copyUs));
    csvValue(line, String(e.firstPassUs)); csvValue(line, String(e.secondPassUs));
    csvValue(line, String(e.peaksUs));
    csvValue(line, String(e.compareUs));
    csvValue(line, String(e.projectionMode));
    csvValue(line, String(e.anchorMatchedMask));
    for (int anchor = 0; anchor < MAX_EDGE_ANCHORS; ++anchor) {
      csvValue(line, String(e.anchorX[anchor]));
      csvValue(line, String(e.anchorY[anchor]));
    }
    for (int bucket = 0; bucket <= MAX_PEAKS; ++bucket) {
      csvValue(line, String(e.referenceBucket[bucket]));
      csvValue(line, String(e.liveBucket[bucket]));
    }
    for (int peak = 0; peak < MAX_PEAKS; ++peak) {
      csvValue(line, String(e.referenceAngles[peak], 1));
      csvValue(line, String(e.referenceShares[peak], 3));
      csvValue(line, String(e.liveAngles[peak], 1));
      csvValue(line, String(e.liveShares[peak], 3));
    }
    for (int bin = 0; bin < BINS; ++bin) csvValue(line, String(e.referenceHist[bin]));
    for (int bin = 0; bin < BINS; ++bin) csvValue(line, String(e.liveHist[bin]));
    line += "\r\n";
    server.sendContent(line);
  }
  server.sendContent("");
}

void handleClearLog() {
  logCount = logHead = 0;
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("Railway camera angle experiment v%s\n", CAMERA_ANGLE_EXPERIMENT_VERSION);
  for (int angle = 0; angle <= 20; ++angle)
    toleranceTanQ8[angle] = (uint8_t)lroundf(tanf(angle * 0.01745329252f) * 256);
  if (!initCameraAt(0)) Serial.println("Camera unavailable; hotspot will allow retry.");
  for (uint16_t capacity : {1024, 512, 256}) {
    frameLog = (FrameLog *)ps_malloc((size_t)capacity * sizeof(FrameLog));
    if (frameLog) { logCapacity = capacity; break; }
  }
  Serial.printf("Frame log: %u records (%u bytes each)\n", logCapacity, sizeof(FrameLog));
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Could not start Wi-Fi hotspot");
    return;
  }
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/stats", HTTP_GET, handleStats);
  server.on("/log/recent", HTTP_GET, handleRecentLog);
  server.on("/log.csv", HTTP_GET, handleLogCsv);
  server.on("/log/clear", HTTP_POST, handleClearLog);
  server.on("/benchmark", HTTP_GET, handleBenchmark);
  server.on("/region", HTTP_POST, handleRegion);
  server.on("/resolution", HTTP_POST, handleResolution);
  server.on("/baseline", HTTP_POST, handleBaseline);
  server.begin();
  Serial.printf("Join %s and open http://%s/\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  server.handleClient();
  if (cameraStarted && grayFrame && millis() - lastAttemptMs >= CAPTURE_PERIOD_MS)
    captureAndAnalyse();
  delay(1);
}
