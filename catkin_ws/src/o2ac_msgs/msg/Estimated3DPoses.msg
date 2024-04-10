int64   class_id               # part ID detected by SSD
float64  width                 # shorted side (m)
float64  length                # longest side (m)
float64 confidence             # confidence localization
geometry_msgs/PoseStamped pose # 3D poses (might have multiple poses in belt detection)
bool    upside_down            # True if the item is upside down
