# Vision Pipeline — Multi-Camera Kalman Filter Fusion

## Overview

Theseus uses 4 PhotonVision cameras (FL, FR, BL, BR) to detect AprilTags and estimate the robot's field position. All valid measurements are fed into the drivetrain's Kalman filter with uncertainty scaled by detection quality, rather than picking a single "best" camera per cycle.

## Why Feed All Cameras Into the Kalman Filter?

### The old approach: pick the best camera

The previous implementation used hard ambiguity thresholds to select measurements:
- Only the camera with ambiguity < 0.1 was used (tight std devs)
- A secondary camera with ambiguity < 0.2 could contribute (loose std devs)
- All other cameras were discarded

**Problems with this approach:**

1. **Wasted information.** If three cameras see tags with ambiguities of 0.08, 0.09, and 0.12, the old code might only use one or two. The third camera's data — which is still useful — gets thrown away entirely.

2. **Threshold sensitivity.** A camera at 0.099 ambiguity gets tight std devs; one at 0.101 gets nothing. The jump from "trusted" to "ignored" is a cliff, not a slope. Real-world noise means a camera can bounce between bins frame-to-frame.

3. **Multiple cameras seeing different tags can cross-validate.** If CameraFL sees tag 5 and CameraBR sees tag 12, those are independent measurements of the same robot pose. Both should contribute. The Kalman filter is specifically designed to fuse multiple noisy measurements into a better estimate than any single one.

4. **The Kalman filter already handles uncertainty weighting.** That's literally what it does — each measurement's influence is weighted by its stated uncertainty (the std dev vector). Adding a second layer of "should I even tell the filter?" is redundant and lossy.

### The new approach: continuous std dev scaling

Every camera measurement that passes basic sanity checks gets fed to the Kalman filter with uncertainty that scales continuously:

```
xyStdDev   = 0.01 + (ambiguity × 2.0) + (distance × 0.05)
thetaStdDev = degToRad(2 + ambiguity × 50 + distance × 10)
```

- A close, low-ambiguity detection (ambiguity=0.02, distance=1m) → `xyStdDev ≈ 0.10m`, `thetaStdDev ≈ 3°` → strong influence
- A far, higher-ambiguity detection (ambiguity=0.20, distance=4m) → `xyStdDev ≈ 0.61m`, `thetaStdDev ≈ 52°` → weak influence, but still contributes

No information is wasted. The filter decides how much to trust each measurement.

### Sanity gates (pre-filter rejection)

Not everything should enter the Kalman filter. Three rejection criteria prevent garbage data from corrupting the state estimate:

| Gate | Threshold | Why |
|------|-----------|-----|
| Ambiguity | > 0.25 | Detection is too uncertain to be meaningful — likely a misread or partial tag view |
| Field bounds | x ∉ [0, 16.54m], y ∉ [0, 8.21m] | Pose is physically impossible — the robot can't be outside the field |
| Pose jump | > 5m from current odometry | Likely a misidentified tag or corrupted detection — would yank the filter state |

These are generous thresholds. The goal is to reject obviously wrong data, not to pre-filter quality. Quality weighting is the Kalman filter's job.

## Other Fixes in This Redesign

### Timestamp correction

The old code used `Timer.getFPGATimestamp()` (current time) as the measurement timestamp. The new code uses `result.getTimestampSeconds()` (actual image capture time). This matters because the Kalman filter replays the measurement at the correct point in the state history — using "now" instead of "when the image was taken" introduces latency-proportional error in the fused pose.

### Transform direction

`PhotonUtils.estimateFieldToRobotAprilTag()` expects a **camera-to-robot** transform. The constants are defined as **robot-to-camera** (which is more intuitive — "where is the camera on the robot?"). The new code calls `.inverse()` at the call site to convert correctly.

### Removed `resetPose` on low X

The old code hard-reset odometry (`drivetrain.resetPose()`) whenever the pose X was < 0.1m. This wiped the entire Kalman filter state every cycle the robot was near the field origin — defeating the purpose of the soft vision fusion happening above it. Removed entirely.

## Data Flow

```
PhotonVision Coprocessors (10.49.82.13, 10.49.82.14)
              ↓
     PhotonCamera × 4
              ↓
    Camera.process()
    (PhotonUtils.estimateFieldToRobotAprilTag)
              ↓
    CameraUsing.processCamera()
    ├── Reject: ambiguity > 0.25
    ├── Reject: outside field bounds
    ├── Reject: > 5m from odometry
    └── Accept: scale std devs by quality
              ↓
    drivetrain.addVisionMeasurement(
        pose, captureTimestamp, [xyStdDev, xyStdDev, thetaStdDev])
              ↓
    Phoenix 6 SwerveDrivetrain Kalman Filter
              ↓
    getState().Pose  (fused wheel odometry + vision)
```

## Tuning Guide

If testing reveals the vision is too aggressive or too conservative, adjust the std dev formula coefficients:

| Parameter | Effect of increasing | Current value |
|-----------|---------------------|---------------|
| Base xy std dev | Less vision influence overall | 0.01 m |
| Ambiguity xy coefficient | High-ambiguity detections trusted less | 2.0 |
| Distance xy coefficient | Far detections trusted less | 0.05 |
| Base theta std dev | Heading corrections weaker | 2° |
| Ambiguity theta coefficient | High-ambiguity heading trusted less | 50 |
| Distance theta coefficient | Far detections affect heading less | 10 |
| `MAX_AMBIGUITY` | More measurements rejected outright | 0.25 |
| `MAX_POSE_JUMP` | Filter is stricter about sudden jumps | 5.0 m |

## Logging

| AdvantageKit Key | Type | Description |
|-----------------|------|-------------|
| `Vision/HasTarget` | boolean | At least one camera contributed this cycle |
| `Vision/MeasurementCount` | int | Number of cameras that passed filters and fed the Kalman filter |
| `Vision/EstimatedPose` | Pose2d | Best (lowest ambiguity) accepted pose this cycle |
| `Vision/BestAmbiguity` | double | Ambiguity of the best accepted detection |
