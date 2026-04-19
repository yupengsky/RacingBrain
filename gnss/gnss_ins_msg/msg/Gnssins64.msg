# TODO: 定义供 SLAM 使用的高精度 GNSS / INS 消息。
# 建议至少覆盖当前链路需要的字段：
# - std_msgs/Header header
# - float64 latitude
# - float64 longitude
# - float64 altitude
# - float64 roll
# - float64 pitch
# - float64 yaw
# - float64 vel_e
# - float64 vel_n
# - float64 imu_gyro_z
# - 可选：定位状态 / RTK 状态 / 协方差
