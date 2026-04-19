# gnss_ins_msg

这个包是 GNSS / INS 消息的单一事实来源。

你复刻时建议先完成：

1. `Gnssins.msg` 的原始串口 / 解包字段
2. `Gnssins64.msg` 的高精度字段与统一时间戳
3. 与 rosbag 中实际 topic 类型保持一致

实现顺序建议：

- 先把消息字段定下来
- 再实现 `gnss/cpp_pubsub`
- 最后再接入 `slam/slam`
