# How the MATLAB Pipeline Works

This document explains, in simple terms, how the MATLAB code takes a folder of drone photos and turns them into a single stitched panorama, a depth map, and a 3D point cloud.

---

## The Big Picture

```
Drone Photos  →  Clean Them Up  →  Find Common Points  →  Align & Stitch  →  Final Output
```

The entire process runs through **5 phases**, one after another. You start it by running `main.m`.

---

## Phase 1 — Load & Clean the Images

**Files involved:**
- `models/image_perception/loadDroneImages.m`
- `models/image_perception/preprocessImages.m`
- `models/image_perception/distortionCorrection.m`

**What happens:**

1. **Load** — The system scans your image folder, picks up all `.jpg` files, and reads them into memory.

2. **Preprocess** — Every image goes through cleanup:
   - All images are resized to the same dimensions (matching the first image).
   - Contrast is improved using CLAHE — this makes features in dark shadows and bright highlights easier to detect.
   - A light blur is applied to remove camera sensor noise.

3. **Distortion Correction** — Drone cameras (especially wide-angle ones) bend straight lines near the edges. This step straightens them out using a mathematical model that reverses the barrel distortion.

> **In short:** Raw photos go in, clean and consistent photos come out.

---

## Phase 2 — Find Features in Each Image

**Files involved:**
- `models/image_processing/akazeDescriptors.m` *(AKAZE implementation)*
- `models/image_processing/detectFeatures.m` *(standalone multi-algorithm detector)*

**What happens:**

Each image is scanned for **keypoints** — distinctive spots like corners, edges, or textured patterns that are easy to recognize.

The system uses **AKAZE** (Accelerated KAZE) by default to:
1. Detect keypoints in a **nonlinear scale space** — unlike SURF/SIFT which blur edges, AKAZE preserves object boundaries using Perona-Malik diffusion. This produces more repeatable keypoints on aerial terrain.
2. At each point, compute a **binary descriptor** — a compact fingerprint that describes what that spot looks like, using a FREAK sampling pattern for fast matching.

These descriptors are what allow the system to say "this corner in photo A is the same corner in photo B."

> **In short:** Each image gets a list of recognizable landmarks and their fingerprints, detected using an algorithm optimized for preserving boundaries.

---

## Phase 3 — Match Features & Compute Transforms

**Files involved:**
- `models/image_processing/matchFeaturesNN.m`
- `models/image_processing/estimateHomography.m`

**What happens:**

1. **Matching** — For each consecutive pair of images (1↔2, 2↔3, 3↔4, etc.), the system compares their descriptor fingerprints using nearest-neighbor search. If two descriptors are similar enough, those points are considered a match.

2. **Homography Estimation** — From the matched points, the system calculates a **homography matrix** — a 3×3 transformation that describes exactly how to warp one image so it overlaps perfectly with the other.

   It uses **MSAC** (a robust version of RANSAC) to ignore bad matches (outliers) and only use reliable point pairs.

3. **Chaining** — Each transform is multiplied with the previous one, so every image can be mapped back to the same coordinate system as the first image.

> **In short:** The system figures out "how does each photo overlap with the next one?"

---

## Phase 4 — Stitch Into a Panorama

**Files involved:**
- `models/image_rearranger/alignImages.m`
- `models/image_rearranger/generatePanorama.m`
- `models/image_rearranger/seamSelection.m`
- `models/image_rearranger/multiBandBlend.m`
- `models/image_rearranger/adaptiveWarp.m`

**What happens:**

1. **Canvas Creation** — The system calculates how big the final image needs to be to fit all warped photos.

2. **Warping** — Each image is projected onto this shared canvas using its homography transform (`imwarp`). Think of it like projecting slides onto a wall — each slide lands in the right position.

3. **Seam Selection** — In areas where two images overlap, the system uses **dynamic programming** to find the optimal cut line. It picks the path where the two images look most similar, so the transition is invisible.

4. **Blending** — The edges around the seams are softened with Gaussian blurring so there are no harsh lines between stitched images.

The result is a single, seamless panorama image saved as a `.png` file.

> **In short:** All photos are warped, aligned, and blended into one big picture.

---

## Phase 5 — Depth Estimation & Visualization

**Files involved:**
- `models/image_processing/depthEstimation.m`
- `visualization/showPanorama.m`
- `visualization/showDepthMap.m`
- `visualization/showPointCloud.m`
- `visualization/showFeatureMatches.m`

**What happens:**

1. **Depth Map** — The system estimates how "far away" each part of the scene is by looking at how much each pixel moved between consecutive photos (optical flow). Objects closer to the drone move more between frames — this parallax effect is used to estimate relative depth.

2. **Visualization** — Four result windows open:
   - **Panorama viewer** — The final stitched image.
   - **Depth map** — A colorized heatmap (red = near, blue = far).
   - **3D point cloud** — An interactive 3D view where the depth map is projected into space and colored with the original photo.
   - **Feature matches** — A side-by-side view showing which points were matched between the first two images.

> **In short:** The system estimates depth and shows you everything in interactive windows.

---

## Supporting Files

### `main.m` — The Boss

This is the file you run. It:
- Sets up configuration (which detector to use, where to save outputs).
- Opens a live log window showing progress.
- Calls each phase in order.
- Saves all outputs and logs.

### `pipeline/reconstructionPipeline.m` — The Shortcut

A simpler version of `main.m` without the fancy log window. Runs the same core steps and returns a result struct. Good for batch processing.

### `utils/TUILogger.m` — The Logger

Creates a GUI window that shows real-time, timestamped log messages as the pipeline runs. Also saves the full log to a file.

### `utils/imageUtils.m` — Image Helpers

Small helper functions: validate files, load multiple formats, convert between color spaces, resize images, save with metadata.

### `utils/mathUtils.m` — Math Helpers

Geometric math functions: compute homographies from scratch (DLT algorithm), run RANSAC, calculate transfer errors, normalize points, compute angles and rotations.

---

## What Gets Created

| Output | Location | What It Is |
|--------|----------|-----------|
| Panorama | `storage/panoramas/` | The final stitched aerial image |
| Depth Map | `storage/depth_maps/` | A grayscale image showing relative depth |
| Log File | `storage/logs/` | Full text log of the pipeline run |

---

## Quick Start

```matlab
% Run with the included sample images
main('matlab/Aerial234')

% Run with your own drone photos
main('C:\your\photos\folder')
```

That's it — the system handles everything else automatically.
