# cone_ws

这个子工作区负责“从数据集到相机锥桶检测”的部分。

建议重建顺序：

1. 先补 `scripts/bag_to_yolo.py`，把 rosbag 或图片流转成标注数据
2. 再补 `configs/*.yaml` 和 `make_dataset_yaml.py`
3. 跑通 `train_yolov8n.py`
4. 最后实现 `src/cone_detector/cone_detector/yolo_detector.py`

如果你后续不打算在仓库内训练，也可以保留数据集脚本，只专注推理链路。
