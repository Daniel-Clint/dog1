# edge_node 启动与监测

## roscore后启动 `omni_stitch_capture_node.py` 和 `edge_agent_node`

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

roslaunch edge_node edge_stitch.launch
```
## 监测
### 监视 stitched capture 输出

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

rostopic echo /omni_stitch_capture/capture
```

### 监视边缘节点发给后端的消息

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

rostopic echo /edge_agent/outgoing_text
```

### 监视导航目标点

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

rostopic echo /move_base_simple/goal
```

### 监视导航取消

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

rostopic echo /move_base/cancel
```

### 监视连接、注册和命令相关日志

```bash
source /opt/ros/noetic/setup.bash
source /home/lw/edge_node_ws/devel/setup.bash

rostopic echo /rosout | egrep "WebSocket connected|Registration succeeded|register.ack reported failure|Received unknown command"
```

### 监视建图位姿是否正常发布

```bash
source /opt/ros/noetic/setup.bash

rostopic hz /aft_mapped_to_init
```

