# 病虫害检测消息
# 当视觉模型检测到病虫害时发布
# pest_monitor 节点订阅此消息，结合 AMCL 位姿记录病虫害坐标

# 病虫害类型编号 (0, 1, 2, ... 对应不同病虫害种类)
uint8 pest_type

# 病虫害类型名称 (可选，由视觉模型填充)
string pest_type_name

# 检测置信度 (0.0~1.0)
float32 confidence

# 检测时间戳 (ISO 8601 格式字符串)
string timestamp

# 快照图片路径 (可选)
string image_path
