# PROJ 与 UTM 坐标

## 1. 为什么要从经纬度转 UTM

GNSS 输出：

```text
latitude
longitude
```

单位是角度，不适合直接做米级地图。

SLAM 需要：

```text
x, y，单位米
```

因此需要投影坐标系。当前工程用 PROJ 将 WGS84 经纬度转为 UTM。

## 2. 使用的坐标系

输入：

```text
EPSG:4326
```

也就是 WGS84 经纬度。

输出默认：

```text
EPSG:32651
```

也就是 UTM Zone 51N。

配置：

```yaml
system:
  utm_zone_epsg: "EPSG:32651"
```

如果比赛地点变了，需要修改 UTM zone。例如注释中提到合肥可能要改为 `EPSG:32650`。

## 3. PROJ 初始化

```cpp
P_ = proj_create_crs_to_crs(
    PJ_DEFAULT_CTX,
    "EPSG:4326",
    sys_.utm_zone_epsg.c_str(),
    NULL
);
```

如果失败：

```cpp
if (P_ == nullptr) {
    RCLCPP_ERROR(...);
}
```

析构：

```cpp
if (P_) proj_destroy(P_);
```

## 4. 坐标转换

```cpp
PJ_COORD input_coord = proj_coord(gnss_msg->latitude, gnss_msg->longitude, 0, 0);
PJ_COORD output_coord = proj_trans(P_, PJ_FWD, input_coord);
double utm_e = output_coord.xy.x;
double utm_n = output_coord.xy.y;
```

得到：

- `utm_e`：Easting，东向米制坐标。
- `utm_n`：Northing，北向米制坐标。

## 5. 局部地图原点

UTM 坐标数值很大，不适合直接可视化和局部建图。

所以工程用第一帧 GNSS 作为原点：

```cpp
origin_e_ = utm_e;
origin_n_ = utm_n;
```

之后：

```cpp
tx = utm_e - origin_e_;
ty = utm_n - origin_n_;
```

局部地图坐标更小，适合 RViz 和滤波。

## 6. 复现注意点

1. UTM zone 必须匹配比赛地点。
2. 经纬度精度必须使用 `float64`。
3. 第一帧 GNSS 质量要可靠，否则整个地图原点会偏。
4. 如果 PROJ 轴顺序出现问题，地图可能方向或位置异常，需要用已知点验证。

## 7. 最小实现

```cpp
class Wgs84ToLocalMap {
    PJ* P;
    bool origin_set = false;
    double origin_e, origin_n;

    Vector2d convert(double lat, double lon) {
        PJ_COORD in = proj_coord(lat, lon, 0, 0);
        PJ_COORD out = proj_trans(P, PJ_FWD, in);

        if (!origin_set) {
            origin_e = out.xy.x;
            origin_n = out.xy.y;
            origin_set = true;
        }

        return Vector2d(out.xy.x - origin_e,
                        out.xy.y - origin_n);
    }
};
```

