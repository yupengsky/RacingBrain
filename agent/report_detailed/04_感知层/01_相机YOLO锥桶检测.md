# 相机 YOLO 锥桶检测

## 1. 模块位置

相机检测节点位于：

```text
perception/src/cone_ws/src/cone_detector/cone_detector/yolo_detector.py
```

对应 ROS 包：

```text
cone_detector
```

入口点在 `setup.py`：

```python
'yolo_detector=cone_detector.yolo_detector:main'
```

启动节点名：

```text
yolo_detector
```

## 2. 输入输出

输入：

```text
/camera1/image_raw    sensor_msgs/msg/Image
```

输出：

```text
/yolo/cones           cone_interfaces/msg/ConeArray
/yolo/debug_image     sensor_msgs/msg/Image
```

`/yolo/cones` 给融合节点使用。`/yolo/debug_image` 给人看。

## 3. 参数

节点声明参数：

```python
self.declare_parameter('model_path', load_default_model_path())
self.declare_parameter('conf_threshold', 0.5)
self.declare_parameter('image_topic', '/camera/image_raw')
self.declare_parameter('max_fps', 10.0)
self.declare_parameter('enable_hsv_check', True)
self.declare_parameter('hsv_log_interval', 50)
self.declare_parameter('hsv_draw_mismatch', True)
self.declare_parameter('hsv_check_every_n', 1)
self.declare_parameter('detect_log_interval', 0)
```

HSV 相关阈值：

```python
hsv_s_min = 60
hsv_v_min = 50
red h: 0-10 or 160-179
yellow h: 18-38
blue h: 100-130
```

`run_perception` 中覆盖：

```python
parameters=[{
    'model_path': model_path,
    'image_topic': '/camera1/image_raw',
    'conf_threshold': 0.5,
    'max_fps': 10.0
}]
```

## 4. 模型路径加载

函数：

```python
load_default_model_path()
```

加载优先级：

1. 环境变量 `DRD26_PATH_CONFIG` 指向的配置文件。
2. 从当前 Python 文件父目录逐级向上寻找 `config/hardcoded_paths.ini`。
3. 找不到则返回空字符串。

配置项：

```ini
[models]
yolo_runtime = /media/yupeng/新加卷/Models/DRd26_SLAM/...
```

节点启动时：

```python
model_path = Path(raw_model_path)
if not model_path.exists():
    raise FileNotFoundError(...)
self.model = YOLO(str(model_path))
```

## 5. 图像订阅

订阅代码：

```python
image_topic = self.get_parameter('image_topic').value
self.create_subscription(Image, image_topic, self.image_only_callback, 10)
```

当前工程实际订阅：

```text
/camera1/image_raw
```

## 6. 帧率限制

回调开头：

```python
if self.max_fps > 0:
    now = self.get_clock().now()
    min_interval = 1.0 / float(self.max_fps)
    if (now - self.last_process_time).nanoseconds * 1e-9 < min_interval:
        return
    self.last_process_time = now
```

作用：

- 防止 YOLO 推理占满 CPU/GPU。
- rosbag 高频播放时自动跳帧。

默认 `max_fps=10.0`，约每 0.1 秒处理一帧。

## 7. ROS Image 到 OpenCV

代码：

```python
cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
```

使用 `cv_bridge` 把 ROS 图像转为 OpenCV BGR 图像。

## 8. YOLO 推理

代码：

```python
results = self.model(cv_image, conf=self.conf_thresh, verbose=False)
```

其中：

- `cv_image` 是 BGR 图像。
- `conf` 是置信度阈值。
- `results[0].boxes` 中包含每个检测框。

每个 box 包含：

```python
box.xyxy[0]  # x1, y1, x2, y2
box.cls[0]   # 类别 ID
box.conf[0]  # 置信度
```

## 9. 类别名获取

函数：

```python
_get_class_name(cls_id)
```

兼容两种 Ultralytics names 格式：

```python
if isinstance(self.model.names, dict):
    return self.model.names.get(cls_id, str(cls_id))
if isinstance(self.model.names, list):
    return self.model.names[cls_id]
```

## 10. 类别到颜色枚举映射

字典：

```python
self.class_color_map = {
    'BLUE': Cone.BLUE,
    'RED': Cone.RED,
    'YELLOW_BIG': Cone.YELLOW_BIG,
    'YELLOW_SMALL': Cone.YELLOW_SMALL,
    'UNKNOWN': Cone.UNKNOWN,
    'blue_cone': Cone.BLUE,
    'red_cone': Cone.RED,
    'big_yellow_cone': Cone.YELLOW_BIG,
    'small_yellow_cone': Cone.YELLOW_SMALL,
    'orange_big': Cone.YELLOW_BIG,
    'orange_small': Cone.YELLOW_SMALL,
    'ORANGE_BIG': Cone.YELLOW_BIG,
    'ORANGE_SMALL': Cone.YELLOW_SMALL,
    'RED_BIG': Cone.RED,
    'red_big_cone': Cone.RED,
    'big_red_cone': Cone.RED,
}
```

函数 `_map_color(cls_name)` 做大小写兼容：

1. 原始名字匹配。
2. 大写匹配。
3. 小写匹配。
4. 都失败则 `UNKNOWN`。

这对训练数据类别名不统一很重要。

## 11. 构造 ConeArray

函数：

```python
_build_cone_array(image_msg, results)
```

流程：

1. 新建 `ConeArray`。
2. 复制原图像 header。
3. 遍历 YOLO boxes。
4. 从 `xyxy` 算中心和尺寸。
5. 类别名映射颜色。
6. 填入 `ConeArray`。

核心代码逻辑：

```python
x1, y1, x2, y2 = bbox
width = max(0.0, x2 - x1)
height = max(0.0, y2 - y1)
center_x = x1 + width / 2.0
center_y = y1 + height / 2.0

cone.center.x = center_x
cone.center.y = center_y
cone.size.x = width
cone.size.y = height
cone.confidence = float(box.conf[0])
cone.color = self._map_color(cls_name)
```

## 12. 调试图像绘制

函数：

```python
draw_detections(image, results, do_hsv)
```

逻辑：

- 复制原图。
- 遍历检测框。
- 绘制矩形。
- 绘制类别名、置信度、HSV 判断结果。
- 如果 HSV 判断与模型类别不一致，用红框标记。

## 13. HSV 颜色一致性检查

这个检查不是用于修改输出颜色，而是用于调试模型分类质量。

函数：

```python
_classify_hsv_color(bgr_roi)
```

流程：

1. BGR 转 HSV。
2. 用饱和度和亮度过滤低质量像素。
3. 取 H 通道中位数。
4. 根据 H 范围判断 red/yellow/blue。

期望颜色来自类别名：

```python
if 'yellow' in name: expected = 'yellow'
if 'red' in name or 'orange' in name: expected = 'red'
if 'blue' in name: expected = 'blue'
```

统计函数：

```python
_update_hsv_stats(cls_name, match)
```

每隔 `hsv_log_interval` 输出一次整体和分类别准确率。

## 14. 发布

回调中发布：

```python
debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
self.debug_image_pub.publish(debug_msg)
self.cone_pub.publish(cone_array)
```

其中：

- `debug_image_pub` 发布 `/yolo/debug_image`。
- `cone_pub` 发布 `/yolo/cones`。

## 15. 复现最小伪代码

```python
class YOLOConeDetector(Node):
    def __init__(self):
        super().__init__("yolo_cone_detector")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/camera1/image_raw", self.cb, 10)
        self.pub = self.create_publisher(ConeArray, "/yolo/cones", 10)

    def cb(self, image_msg):
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        results = self.model(img, conf=0.5, verbose=False)

        out = ConeArray()
        out.header = image_msg.header

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls_name = self.get_class_name(int(box.cls[0]))

            cone = Cone()
            cone.center.x = float((x1 + x2) / 2)
            cone.center.y = float((y1 + y2) / 2)
            cone.size.x = float(x2 - x1)
            cone.size.y = float(y2 - y1)
            cone.confidence = float(box.conf[0])
            cone.color = map_class_to_color(cls_name)
            out.cones.append(cone)

        self.pub.publish(out)
```

## 16. 实现注意点

1. `ConeArray.header` 必须继承图像 header，否则融合层时间同步效果会变差。
2. 模型类别名必须和 `_map_color` 兼容。
3. `max_fps` 会跳帧，验证检测数量时要考虑。
4. HSV 检查只用于调试，不改变 `/yolo/cones` 输出。
5. `model_path` 不存在时节点直接抛异常退出。

