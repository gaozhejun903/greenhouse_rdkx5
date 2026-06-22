# 病虫害报告消息 (包含地图坐标)
# pest_monitor 节点在收到 PestDetection 后，结合 AMCL 位姿生成此消息

# 病虫害类型编号
uint8 pest_type

# 病虫害类型名称
string pest_type_name

# 检测置信度 (0.0~1.0)
float32 confidence

# 小车在地图中的坐标 (AMCL 定位)
float64 map_x
float64 map_y
float64 map_theta

# 检测时间戳 (ISO 8601 格式)
string timestamp

# 快照图片路径
string image_path
