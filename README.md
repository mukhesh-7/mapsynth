# MapSynth: Drone-Based Image Reconstruction System

## System Overview
The Drone-Based Image Reconstruction System is a full-stack prototype explicitly designed to ingest fragmented drone imagery of a surface, deeply analyze them for similarities, accurately align them in spatial dimensions, and finally stitch them into a seamlessly reconstructed global surface map.

## Clear Architecture 
The system employs a multi-tiered architecture that separates the frontend interaction, backend coordination, and rigorous heavy-lifting computational processes.
1. **Client Layer:** User-facing dashboard for image uploads and map visualization.
2. **Server / API Layer:** Central coordinator handling uploads, routing, and dispatching jobs to the processing engine.
3. **Image Processing Engine:** The computational core powered by MATLAB to run complex homography, feature matching, and image stitching models.
4. **Data Storage Layer:** Dual-storage strategy—PostgreSQL for metadata and file storage for raw/processed images.
5. **Visualization Layer:** Presents the final stitched orthomosaic map, depth heatmaps, and feature alignments directly on the client.

## Technology Stack
- **Client:** React (UI dashboard), Tailwind CSS (styling), Axios (API communication), Leaflet or WebGL viewer (map visualization).
- **Server:** Node.js + Express (REST API routing), Job queue (task management).
- **Processing Engine:** MATLAB, Computer Vision Toolbox, Image Processing Toolbox. (Deep Learning Toolbox as an optional enhancement).
- **Database:** PostgreSQL (metadata), File storage (images & stitched outputs).
- **Infrastructure:** Docker containers, Local processing worker.

## Workflow Pipeline
`Drone` → `Image Capture` → `Upload via Client` → `API Server` → `MATLAB Processing Engine` → `Image Stitching Pipeline` → `Storage` → `Visualization Dashboard`

## Key Modules and Responsibilities
1. **Image Perception Module**
   - **Responsibilities:** Load drone image frames robustly, normalize their resolution to standard dimensions, and perform optional distortion correction to compensate for camera irregularities.
2. **Image Processing Module**
   - **Responsibilities:** Identify unique points via Feature Detection and Extraction. Pair visuals via Feature Matching, estimate depth properties, and build a Similarity Graph of overlapping image areas.
3. **Image Reconstruction Module**
   - **Responsibilities:** Compute the mathematical transformation (Homography Estimation). Seamlessly adapt and blend visual seams (Adaptive Warping & Seam Selection) and generate the finalized panoramic canvas (Multi-band Blending & Panorama Generation).

## Security Considerations
- **Authentication:** JWT-based user authentication verifying identity before enabling REST API interaction.
- **Data Protection:** Multi-layer encryption strategies tailored to uploaded drone assets and transit encrypted payloads utilizing TLS/SSL standard protocols.

## Potential Performance Bottlenecks
- **Network Upload Latency:** Very large raw drone images require fast bandwidth.
- **Compute-Heavy Homography Alignment:** Feature extraction across hundreds of images rapidly degrades speed.
- **Disk I/O Contention:** Frequent reads/writes during high-resolution multi-band blending can stall processing.

## Well-Defined File Structure
The project directory layout cleanly segregates responsibilities (please refer to `docs/` and the root folder for expanded component details).
- `client/` - React application and viewers
- `server/` - Express servers and MATLAB dispatchers
- `matlab/` - Models for perception, processing, and reconstruction
- `database/` - Schemas and Migration logic
- `storage/` - Output bins for generated media assets
- `docs/` - System architecture and flow diagrams