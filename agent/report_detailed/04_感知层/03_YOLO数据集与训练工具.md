# YOLO 数据集与训练工具

## 1. 模块位置

YOLO 数据集和训练脚本位于：

```text
perception/src/cone_ws/scripts
```

主要脚本：

```text
bag_to_yolo.py
make_dataset_yaml.py
train_yolov8n.py
validate_dataset.py
hsv_eval.py
self_train.py
data_augmentation.py
xml2txt.py
```

这些脚本不参与实时 SLAM 主链路，但它们是相机模型训练和维护的重要工程资产。

## 2. 配置文件

脚本普遍从根目录配置读取路径：

```text
config/hardcoded_paths.ini
```

核心配置：

```ini
[models]
root = ...
yolo_runtime = ...
yolo_pretrained = ...
yolo26 = ...

[datasets]
root = ...
rosbag_2026_02_05 = ...
yolo_output_root = ...
combined_yaml = ...

[training]
runs_root = ...
```

设计目的：

- 避免多个脚本里散落绝对路径。
- 方便换模型、换数据集。

## 3. bag_to_yolo.py

作用：

```text
从 rosbag 中读取图像，用已有 YOLO 模型自动生成伪标签数据集。
```

核心能力：

- 读取 rosbag 图像 topic。
- 使用 YOLO 模型推理。
- 将 bbox 转成 YOLO txt 格式。
- 按 train/val 分割。
- 支持每 N 帧采样。
- 支持 hash split，保证划分稳定。
- 支持 HSV 增强等图像处理。

类别映射：

```python
CLASS_IDS = {
    "BLUE": 0,
    "RED": 1,
    "YELLOW_BIG": 2,
    "YELLOW_SMALL": 3,
    "UNKNOWN": 4,
    "RED_BIG": 1,
    "BLUE_CONE": 0,
    "RED_CONE": 1,
    "BIG_YELLOW_CONE": 2,
    "SMALL_YELLOW_CONE": 3,
    "ORANGE_BIG": 2,
    "ORANGE_SMALL": 3,
}
```

典型命令：

```bash
/usr/bin/python3 perception/src/cone_ws/scripts/bag_to_yolo.py \
  --dataset-name rosbag2_2026_02_05-11_01_07 \
  --image-topic /camera1/image_raw \
  --conf 0.5 \
  --val-ratio 0.2 \
  --split-mode hash \
  --every-n 6
```

## 4. make_dataset_yaml.py

作用：

```text
扫描多个 YOLO 数据集目录，生成合并训练 YAML。
```

它会查找：

```text
images/train
images/val
labels/train
```

然后生成：

```yaml
train: [...]
val: [...]

nc: 5
names:
  0: BLUE
  1: RED
  2: YELLOW_BIG
  3: YELLOW_SMALL
  4: UNKNOWN
```

典型命令：

```bash
/usr/bin/python3 perception/src/cone_ws/scripts/make_dataset_yaml.py
```

## 5. train_yolov8n.py

作用：

```text
训练锥桶 YOLOv8n 模型。
```

核心配置：

```python
RUNS_ROOT = training.runs_root
PRETRAINED_MODEL = models.yolo_pretrained
DEFAULT_DATA_ROOT = datasets.yolo_output_root
```

支持参数：

- `--data`
- `--data-root`
- `--merge-root`
- `--merge-mode`
- `--run-prefix`
- `--run-name`
- `--epochs`
- `--imgsz`
- `--batch`

默认：

```text
epochs = 80
imgsz = 960
batch = 16
```

它可以把多个数据集合并成临时 YAML，也可以把多个数据集实际合并到一个目录。

## 6. validate_dataset.py

作用：

```text
检查 YOLO 数据集标签质量。
```

检查内容：

- label 文件是否为空。
- label 是否有对应图像。
- 每行是否至少有 5 个字段。
- class id 是否在范围内。
- bbox 坐标是否归一化且在合理范围内。
- 统计各类别样本数量。

这个脚本适合训练前执行，避免训练中途因为坏标签失败。

## 7. hsv_eval.py

作用：

```text
用 HSV 颜色规则评估模型类别颜色是否一致。
```

流程：

1. YOLO 检测图片。
2. 截取 bbox ROI。
3. 用 HSV 中位数判断颜色。
4. 将模型类别期望颜色与 HSV 判断颜色比较。
5. 输出统计图和调试图片。

它与实时节点中的 HSV 检查逻辑一致，都是为了发现颜色语义分类错误。

## 8. self_train.py

作用：

```text
伪标签自训练流程。
```

主要思路：

1. 找到最新一次训练权重。
2. 用该权重给 rosbag 图像打伪标签。
3. 用伪标签数据继续训练。
4. 多轮迭代。

它适合快速扩大数据集，但要注意伪标签会继承模型错误。

## 9. data_augmentation.py

这个脚本更多是说明和建议性质。

它读取当前数据集统计，然后输出推荐训练增强配置，例如：

- mosaic
- flip
- rotate
- translate
- scale

当前没有直接批量生成增强图片，而是推荐在 Ultralytics 训练中启用实时增强。

## 10. xml2txt.py

作用：

```text
将旧 XML 标注转换为 YOLO txt。
```

类别映射：

```python
blue_cone -> 0
red_cone -> 1
big_yellow_cone -> 2
small_yellow_cone -> 3
big_red_cone -> 1
red_big_cone -> 1
```

适合把历史 VOC 格式标注迁移到当前 YOLO 格式。

## 11. 数据集 YAML

当前仓库中有：

```text
perception/src/cone_ws/configs/cone.yaml
perception/src/cone_ws/configs/cone_bag_2026.yaml
perception/src/cone_ws/configs/cone_all.yaml
```

类别统一为：

```yaml
nc: 5
names:
  0: BLUE
  1: RED
  2: YELLOW_BIG
  3: YELLOW_SMALL
  4: UNKNOWN
```

实时节点 `_map_color` 也围绕这套类别枚举设计。

## 12. 复现训练流程建议

如果你要从零训练相机锥桶检测模型：

1. 收集 rosbag，确保有 `/camera1/image_raw`。
2. 用已有模型生成初始伪标签：

```bash
python3 perception/src/cone_ws/scripts/bag_to_yolo.py \
  --dataset-name my_bag_dataset \
  --image-topic /camera1/image_raw \
  --conf 0.5 \
  --val-ratio 0.2 \
  --every-n 6
```

3. 人工抽查并修正部分标签。
4. 生成合并 YAML：

```bash
python3 perception/src/cone_ws/scripts/make_dataset_yaml.py
```

5. 验证数据集：

```bash
python3 perception/src/cone_ws/scripts/validate_dataset.py --data-root <dataset_root>
```

6. 训练：

```bash
python3 perception/src/cone_ws/scripts/train_yolov8n.py \
  --data <yaml_path> \
  --epochs 80 \
  --imgsz 960 \
  --batch 16
```

7. 将最佳权重路径写入：

```text
config/hardcoded_paths.ini
```

的：

```ini
yolo_runtime = <best.pt>
```

## 13. 与实时工程的关系

实时工程只需要最终权重：

```text
models.yolo_runtime
```

训练脚本负责生产这个权重。

因此相机模块的完整闭环是：

```text
rosbag 图像
  -> bag_to_yolo 生成数据集
  -> validate_dataset 检查数据
  -> train_yolov8n 训练模型
  -> yolo_runtime 写入配置
  -> yolo_detector 实时推理
```

