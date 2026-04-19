# cpp_pubsub

这个包预留给实时 GNSS / INS 驱动或离线协议解析桥。

建议职责拆分：

- `serial_stream.cpp`: 串口打开、读写、重连、缓存
- `publisher_member_function.cpp`: 协议解包并发布 ROS topic
- `subscriber_member_function.cpp`: 调试 / 回环 / 控制通道
- `include/cpp_pubsub/*.h`: 协议常量、字节流解析工具、状态机

如果你后续决定完全离线回放 rosbag，也可以让这个包保持最小实现。
