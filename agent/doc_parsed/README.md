# Parsed Documents

These files are text extractions of the PDFs in `agent/doc/`, generated with
`pdftotext -layout` so future agents can search them without reparsing PDFs.

- `lidar_camera_fusion.txt`: notes for `fs_fusion_box`, including dependencies,
  interfaces, build guidance, CSF installation notes, YOLO environment notes,
  and fusion/debug workflow.
- `perception_slam_integration.txt`: perception-to-SLAM integration notes,
  including `/perception/fusion/map`, track modes, and historical tuning notes.
