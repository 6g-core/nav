# Go2 导航控制开发与部署指南

本文面向开发、部署和运维人员，说明 Go2 导航控制模块的 HTTP 接口、手柄操作，以及构建打包、启动配置和部署验收流程。

| 使用场景 | 对应章节 |
| --- | --- |
| 功能开发与接口集成 | 第 1～3 节：模块组成、HTTP 接口及手柄操作 |
| 部署与升级 | 第 4～5 节：构建打包及启动配置 |
| 验收与故障排查 | 第 6 节；旧版本回退见第 4.5 节 |

部署命令直接粘贴到终端执行，无需创建脚本文件。机器狗工作空间为 `~/unitree/nav`，`~` 和 `$HOME` 均指运行机器人服务的用户家目录。示例中的 ROS 发行版、主机地址、归档文件名和时间戳应按目标环境替换。

## 1. 代码与运行组成

| 本目录的 ROS 包 | 可执行程序 / 启动文件 | 作用 |
| --- | --- | --- |
| `go2_http_control` | `http_control_server` | 监听 `0.0.0.0:10088`；HTTP 路由、动作编排、位姿检查、单任务执行、取消和超时 |
| `go2_wireless_controller` | `wireless_controller_node` | 订阅手柄，识别双击/长按，调用导航、机械臂或 testclient |
| `go2_launcher` | `robot.launch.py` | 启动本项目两个节点，以及外部 `go2_base`、`simple_controls/log_state` |

主要源码：

- [HTTP 服务与全部路由](go2_http_control/http_control/http_control_server.py)
- [HTTP 消息解析与响应](go2_http_control/http_control/common/http.py)
- [任务执行与取消](go2_http_control/http_control/common/task_queue.py)
- [位姿与新鲜度检查](go2_http_control/http_control/common/pose.py)
- [手柄按键识别](go2_wireless_controller/go2_wireless_controller/button_actions.py)
- [手柄业务流程与参数](go2_wireless_controller/go2_wireless_controller/wireless_controller_node.py)
- [统一 launch 入口](go2_launcher/launch/robot.launch.py)

源码链接相对于仓库中的 `nav/` 目录；部署后对应文件位于 `~/unitree/nav/src/<包名>/`。

本目录包含三个应用包，依赖外部 `clients`、`unitree_api`、`unitree_go`、`go2_base` 和 `simple_controls` 包。部署时复用机器狗已有依赖；全新环境须先准备这些包及其运行配置。

`robot.launch.py` 启动以下四个节点：

| ROS 节点名 | 包 / executable |
| --- | --- |
| `/go2_base` | `go2_base / go2_base`，外部依赖 |
| `/log_state` | `simple_controls / log_state`，外部依赖 |
| `/http_control_server` | `go2_http_control / http_control_server` |
| `/wireless_controller` | `go2_wireless_controller / wireless_controller_node` |

该 launch 不启动机械臂 HTTP 服务、testclient、SLAM 或雷达驱动。`/utlidar/robot_pose` 和 `/wirelesscontroller` 必须由机器人或其他已部署节点提供。

## 2. HTTP 接口

- 服务地址示例：`http://127.0.0.1:10088`。远程调用时替换成导航主机 IP。
- 接口区分 GET、POST；POST 使用 `Content-Type: application/json`。
- 路由按 URL 的 path 匹配，查询串不参与路由；尾部 `/` 不会自动删除。
- `/v1/*` POST 的 `timestamp` 只从 **JSON 请求体** 读取，查询参数无效。兼容接口 `/mock-*`、`/goleft`、`/goright`、`/coarse-shift` 不要求时间戳。
- 距离单位米，线速度米/秒；`/v1/move` 和 `/v1/euler` 用弧度，转向接口的字段用角度。
- 前进为机体 +X，左移为 +Y，左转为正偏航。闭环距离取里程计 XY 平面两点间的欧氏距离，并非累计轨迹长度。
- HTTP 请求支持并发处理；机器人任务总容量为一个，包含待执行和执行中的任务。
- 本服务没有按 `command_id` 查询最终任务结果的接口；`request_id` 也不是幂等键，重复请求不会自动去重。

### 2.1 状态查询

两个 URL 共用同一实现，无需请求体或时间戳：

| 方法 | URL | 作用 |
| --- | --- | --- |
| GET | `/v1/status` | 当前状态 |
| GET | `/api/v1/status` | 兼容旧客户端的状态别名 |

成功响应 `200 application/json; charset=utf-8`：

```json
{"ok":true,"busy":false,"pose_ready":true,"original_position_recorded":false}
```

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `ok` | bool | 状态请求成功，不代表上一动作成功 |
| `busy` | bool | 存在排队或正在执行的任务；取消后，未退出的执行线程仍算 busy |
| `pose_ready` | bool | 最近收到的位姿有效，且接收时间距今不超过 2 秒 |
| `original_position_recorded` | bool | 内存中已记录回程起点；进程重启后丢失 |

`busy=false` 只说明任务槽空闲，不能证明已到达目标或抓取成功。手柄的 `wait_nav()` 当前也只根据 busy 判断等待结束，异步任务失败需同时查导航日志和实际位姿。

### 2.2 兼容移动接口

以下均为 POST。除 `/coarse-shift` 外，请求体可用 `{}`，不读取自定义距离或速度。常用线速度为 `0.4 m/s`，组合转向速度为 `3π/8 rad/s`；闭环会按剩余距离调整速度。

| URL | 实际动作 / 前置条件 | 成功响应时机 |
| --- | --- | --- |
| `/mock-move` | 如状态为 `damping` 或 `lieDown`，先站起、平衡站立并禁用避障；随后前进 1.2 m → 左转 85.5° → 前进 0.05 m → 右移 0.15 m → 平衡站立 | 入队后 `200 OK` |
| `/mock-move-mobile` | 执行时记录当前位置，前进 0.8 m 并停止 | 入队后 `200 OK` |
| `/mock-back-mobile` | 左转 171° → 前进 0.05 m → 再前进 1.1 m；这是固定路线，不是精确返回记录点 | 入队后 `200 OK` |
| `/mock-back-original` | 回到记录的 XY 位置，容差 0.1 m；不恢复记录时的朝向。须先完成 `/mock-move-mobile` 或 `/mock-move1` 等记录起点操作 | 入队后 `200 OK`；未记录起点时 400 |
| `/mock-move1` | 记录当前位置，前进 0.3 m 并停止 | 入队后 `200 OK` |
| `/mock-move2` | 对齐 `start_rot` 朝向，然后沿此朝向前进 0.3 m。须先执行初始化 `start_rot` 的移动，如完成 `/mock-move1` | 入队后 `200 OK` |
| `/mock-move3` | `/mock-move2` 的别名，不是第三套动作 | 入队后 `200 OK` |
| `/mock-move4` | 对齐 `start_rot` → 前进 0.3 m → 左转 85.5° → 前进 0.3 m → 右移 0.2 m → 平衡站立；同样需要已初始化 `start_rot` | 入队后 `200 OK` |
| `/mock-back` | 左移 0.25 m → 左转 85.5° → 前进 0.05 m → 前进 1.1 m → 左转 171° → 前进 0.05 m | 入队后 `200 OK` |
| `/goleft` | 左移 0.2 m 并停止 | 入队后 `200 OK` |
| `/goright` | 右移 0.2 m 并停止 | 入队后 `200 OK` |
| `/coarse-shift` | 请求 `{"distance":0.3}`：正值右移，负值左移，零不移动；必须是有限数，代码没有额外距离上限 | 非零距离执行完成后 `200 OK`；零立即返回 |

起点仅保存位置；`start_rot` 会随部分运动函数更新，不作为固定初始朝向。

除同步执行的 `/coarse-shift` 外，兼容接口的 `200 OK` 仅表示任务已接收；后续执行失败写入日志。调用示例：

```sh
# 会让机器人运动，仅在具备运行条件时执行
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/mock-move-mobile \
  -H 'Content-Type: application/json' -d '{}'
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/coarse-shift \
  -H 'Content-Type: application/json' -d '{"distance":0.3}'
```

### 2.3 `/v1/*` 请求和响应消息

通用请求体：

```json
{"timestamp":1790000000.0,"request_id":"demo-001","distance_m":0.3,"speed_mps":0.4}
```

示例时间戳仅用于说明格式，调用时须生成当前时间戳。

| 字段 | 要求 |
| --- | --- |
| `timestamp` | 必填，Unix 秒或毫秒；代码对大于 `100000000000` 的值按毫秒转换；与服务端墙上时钟的绝对差不能超过 20 秒，过去/未来均校验 |
| `request_id` | 可选，建议非空字符串；省略或假值时生成 UUID，空白字符串或非字符串的非假值被拒绝；返回为 `command_id` |
| 动作参数 | 下表列出的字段均需显式提供；代码没有距离、速度、转角默认值 |

普通动作成功入队返回 **202**：

```json
{"ok":true,"command_id":"demo-001","state":"queued"}
```

任务忙碌时返回 **409**：

```json
{"ok":false,"command_id":"demo-001","state":"busy"}
```

停止接口返回 **200**：

```json
{"ok":true,"command_id":"demo-001","state":"stopped","cleared":1}
```

`cleared` 是从队列移除的未开始任务数，不包括正在执行的任务；`stopped` 表示已发出停止操作，不是机器人静止状态的传感器确认。正在执行的任务退出前仍可能 `busy=true`。

#### 有参数动作

| POST URL | 必填动作字段 / 范围 | 作用 |
| --- | --- | --- |
| `/v1/move` | `vx`: [-2.5, 3.8]；`vy`: [-1.0, 1.0]；`vyaw`: [-4.0, 4.0] | 调用 `SportClient.move(vx, vy, vyaw)`；单位 m/s、m/s、rad/s |
| `/v1/euler` | `roll`、`pitch`: [-0.75, 0.75]；`yaw`: [-0.6, 0.6]，单位 rad | 调整机身姿态，调用 `euler` |
| `/v1/speed-level` | `level`: -1、0、1，调用方应传整数 | 调用 `speed_level`；实现先做 `int()` 转换再校验 |
| `/v1/forward` | `distance_m`: [0.05, 2.0]；`speed_mps`: [0.1, 0.5] | 向前闭环移动 |
| `/v1/backward` | 同上 | 向后闭环移动；底层 `gx` 当前对负向速度固定使用 -0.3 m/s，不是严格按 `speed_mps` 倒退 |
| `/v1/left` | 同上 | 向左闭环移动 |
| `/v1/right` | 同上 | 向右闭环移动 |
| `/v1/turn-left` | `angle_deg`: [1, 360]；`speed_degps`: [10, 180] | 左转指定角度 |
| `/v1/turn-right` | 同上 | 右转指定角度 |
| `/v1/turn-back` | 同上 | 正向转动指定角度；不会自动填 180° |
| `/v1/stop`、`/v1/stop-move` | 仅通用时间戳等字段 | 取消待执行/当前任务，设置停止标志并发布停止命令 |

`/v1/move` 只发送一次速度命令，不提供 `duration`、周期续发或定时自动停止。`/v1/forward`、`/v1/backward`、`/v1/left`、`/v1/right` 直接调用 `gx/gy`，当前到达距离后没有额外补发 `stop_move`；调用方需显式停止，不能把 `busy=false` 当作已发布零速度。旧组合动作通常另有停止步骤。

#### 无动作参数的预设动作

下列接口仍需 JSON `timestamp`。是否能执行取决于运行时加载的 `SportClient` 和机器人固件；缺少对应 Python 方法时返回 **501**。

| POST URL | 调用的 `SportClient` 方法 | 作用 |
| --- | --- | --- |
| `/v1/wave`、`/v1/hello` | `hello` | 打招呼 |
| `/v1/handshake` | `handshake` | 握手 |
| `/v1/pounce`、`/v1/front-pounce` | `front_pounce` | 前扑 |
| `/v1/front-jump` | `front_jump` | 前跳 |
| `/v1/scrape` | `scrape` | 刨地 |
| `/v1/front-flip` | `front_flip` | 前空翻 |
| `/v1/hand-stand` | `hand_stand` | 倒立 |
| `/v1/left-flip` | `left_flip` | 左侧翻 |
| `/v1/back-flip` | `back_flip` | 后空翻 |
| `/v1/stretch` | `stretch` | 伸展 |
| `/v1/happy`、`/v1/content` | `content` | 开心动作 |
| `/v1/stand-up` | `stand_up` | 站起 |
| `/v1/stand-down` | `stand_down` | 趴下 |
| `/v1/balance-stand` | `balanced_stand` | 平衡站立 |
| `/v1/recovery-stand` | `recovery_stand` | 恢复站立 |
| `/v1/sit` | `sit` | 坐下 |
| `/v1/rise-sit` | `rise_sit` | 从坐姿起身 |
| `/v1/heart` | `heart` | 比心 |
| `/v1/dance-1` | `dance_1` | 舞蹈 1 |
| `/v1/dance-2` | `dance_2` | 舞蹈 2 |

**客户端版本兼容性：** 部分 `SportClient` 版本缺少 `handshake`、`hand_stand`、`left_flip`、`back_flip`，或使用 `dance1/dance2` 命名舞蹈方法。若客户端未提供表中对应方法，接口返回 501。部署验收时应按第 6.1 节检查实际加载的客户端。

使用动态时间戳的示例：

```sh
# 会触发动作；每次请求重新生成 timestamp
NAV_TS=$(date +%s)
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/v1/stand-up \
  -H 'Content-Type: application/json' \
  -d "{\"timestamp\":$NAV_TS,\"request_id\":\"stand-demo\"}"

# 停止请求同样需要 timestamp
NAV_TS=$(date +%s)
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/v1/stop \
  -H 'Content-Type: application/json' -d "{\"timestamp\":$NAV_TS}"
```

### 2.4 错误与超时

除 `/v1/*` 的 busy 响应为 JSON 外，下列错误通常为 `text/plain; charset=utf-8`，正文含换行，不能一律按 JSON 解码。

| HTTP 状态 | 场景 |
| --- | --- |
| 400 | JSON、时间戳、动作参数无效；兼容移动接口忙碌；未记录原点 |
| 404 | 通用校验通过后，找不到 `/v1/*` 动作 |
| 405 | GET/POST 处理器内未知路径；其他未实现 HTTP 方法由标准处理器返回 501 |
| 408 | `/v1/*` 时间戳与服务端相差超过 20 秒 |
| 409 | `/v1/*` 入队忙碌；或同步 `/coarse-shift` 被取消 |
| 500 | 同步任务执行异常 |
| 501 | 动作对应的客户端方法不存在 |
| 503 | 同步移动任务的位姿缺失、无效或过期 |
| 504 | 同步移动执行或等待超时 |

动作的位姿检查发生在执行线程里。异步接口可能先返回 200/202，再因位姿问题失败；同步 `/coarse-shift` 才能将该失败作为 503 返回。

| 代码常量 / 限制 | 默认值 |
| --- | --- |
| `POSE_MAX_AGE_S` | 2 秒，按本机单调时钟记录的有效位姿接收时间 |
| `MOTION_TIMEOUT_S` | 单段移动/转向 30 秒 |
| `TASK_TIMEOUT_S` | 完整动作序列 120 秒 |
| `SYNC_WAIT_TIMEOUT_S` | 同步请求等待 35 秒后请求取消 |
| `move_to_pos(timeout_s)` | 返回起点 20 秒，并将截止时间传给内部转向/移动 |
| HTTP 连接读写超时 | 10 秒；与任务完成等待不是同一超时 |

以上限制由 Python 常量或函数默认参数定义。取消会唤醒待执行或运行任务的等待者；执行线程退出前仍拒绝新动作，接受新任务后才清除停止标志。超时和停止采用协作式退出，无法强制中断阻塞在第三方客户端内部的调用。

## 3. 手柄按键与流程

消息来源：`/wirelesscontroller`，类型 `unitree_go/msg/WirelessController`。本节点只使用 `keys`，不处理摇杆 `lx/ly/rx/ry`，不把方向键单击直接映射成持续速度。

| 按键 | 位掩码 | 手势 | 作用 / 发出的 HTTP 消息 |
| --- | --- | --- | --- |
| UP | `0x1000` | 双击 | 检查导航空闲和位姿 → POST `/mock-move-mobile` `{}` → 轮询状态等待结束 |
| DOWN | `0x4000` | 双击 | 检查导航空闲、位姿和原点 → POST `/mock-back-original` `{}` → 等待结束 |
| LEFT | `0x8000` | 双击 | 向机械臂 POST `/api/v1/exec`，执行 `retract_pose` 指定的收臂动作 |
| RIGHT | `0x2000` | 双击 | 前进并等待结束 → 机械臂抓取 → 检查返回条件 → 回原点并等待结束 |
| START | `0x0004` | 双击 | 向机械臂 POST `/api/v1/exec`，目标 `white toy rabbit` |
| START | `0x0004` | 长按后松开 | 向 testclient POST `/api/v1/rabbit/start` `{}`；未配置地址则报错 |

手势细节：

- 必须先观察到 `keys=0` 的释放状态；双击的两次按下之间必须松开。
- 默认第二次按下须在第一次松开后的 0.5 秒内；在第二次按下时触发双击动作。
- 默认长按至少 2 秒，**松开时**触发；不会刚满 2 秒就触发。
- 组合键、未知键被拒绝；采样间隔超过 0.5 秒会清除旧手势状态。
- 动作处理中忽略新手势，不积压待执行动作。START 的双击和长按互斥。
- 手柄工作线程中的异常写入日志并释放 busy 状态，没有自动重试或失败回滚。

### 3.1 手柄 ROS 参数

参数在启动时读取；代码没有动态更新回调，运行中 `ros2 param set` 不会自动更新已缓存的 URL 或手势对象，应重启节点应用配置。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `nav_base_url` | `http://127.0.0.1:10088` | 导航 HTTP 根地址 |
| `arm_base_url` | `http://192.168.123.100:18891` | 机械臂 HTTP 根地址 |
| `button_testclient_base_url` | 默认取 `testclient_base_url` 的值 | testclient 根地址，优先参数 |
| `testclient_base_url` | 空字符串 | 兼容旧参数名；优先参数显式为空时也不会再次回退 |
| `retract_pose` | `/home/ubuntu/marm_demo_auto/src/control/d1_pose0.json` | 发给机械臂服务的动作文件路径，必须能被机械臂服务主机访问 |
| `request_timeout` | `180.0` 秒 | 单次外部 HTTP 超时，以及等待导航空闲的轮询截止时间 |
| `double_click_window` | `0.5` 秒 | 双击窗口 |
| `long_press_duration` | `2.0` 秒 | 长按阈值 |
| `controller_stale_after` | `0.5` 秒 | 手柄采样失效间隔 |

轮询导航状态的间隔为 0.25 秒。轮询中的单次 HTTP 调用还可能消耗自己的超时时间，180 秒不是整个流程的严格墙钟上限。HTTP 客户端禁用环境代理。

### 3.2 外部 HTTP 消息接口

这些 URL 是本项目调用的外部服务，不由 `go2_http_control` 实现。

| 服务 / URL | 方法 | 请求 JSON | 说明 |
| --- | --- | --- | --- |
| `{arm_base_url}/api/v1/exec` | POST | `{"target":"white toy rabbit"}` | 抓取目标 |
| `{arm_base_url}/api/v1/exec` | POST | `{"input":"/home/ubuntu/marm_demo_auto/src/control/d1_pose0.json","debug":false,"log":true}` | 收臂；实际 input 取 `retract_pose` 参数 |
| `{button_testclient_base_url}/api/v1/rabbit/start` | POST | `{}` | 启动外部兔子工作流 |

当前手柄客户端接受纯文本 `OK`、空正文，或 JSON 对象；HTTP ≥400、非对象 JSON、`ok:false`，以及 `status` 为 `fail/failed/error` 均视为失败。客户端不校验机械臂任务是否真正完成，也不轮询机械臂任务 ID；若机械臂服务异步接收后立即返回，RIGHT 流程会继续走后续步骤。

## 4. 构建、测试与发布

### 4.1 编译

以下命令在机器狗上逐行执行。三个应用包位于 `~/unitree/nav/src/`，复用已有 `install/` 中的支持包。ROS 以 `foxy` 为例，按实际版本替换。

**第 1 步：停止运行中的服务。** 旧 PlanB 或旧 launch 由管理脚本启动时，在原脚本所在目录、使用原启动用户及相同状态目录配置停止服务；此命令同时停止 nav、dog 和 dog-ctrl：

```sh
./start_dog_services.sh --kill
# vivo 环境使用：./start_dog_services_vivo.sh --kill
```

检查 10088 端口，确认无监听后再编译或启动：

```sh
ss -ltnp '( sport = :10088 )'
```

**第 2 步：加载 ROS 和已有依赖。** 新开终端，保留设备通信配置；构建时用 `local_setup.bash` 加载当前工作空间依赖。

```sh
source /opt/ros/foxy/setup.bash
source ~/unitree/nav/install/local_setup.bash
ros2 pkg prefix clients
ros2 pkg prefix unitree_api
ros2 pkg prefix unitree_go
ros2 pkg prefix go2_base
ros2 pkg prefix simple_controls
```

**第 3 步：编译三个功能包。**

```sh
cd ~/unitree/nav
colcon build --packages-select go2_http_control go2_wireless_controller go2_launcher
```

**第 4 步：加载并检查安装结果。** 编译成功后执行：

```sh
source ~/unitree/nav/install/setup.bash
ros2 pkg executables go2_http_control
ros2 pkg executables go2_wireless_controller
grep -n 'go2_http_control\|go2_wireless_controller' \
  ~/unitree/nav/install/go2_launcher/share/go2_launcher/launch/robot.launch.py
```

启动使用第 5.1 节命令。已有支持依赖缺失时，须先完成其安装；发布构建不使用 `--symlink-install`。

### 4.2 测试

机器狗执行，预期全部通过：

```sh
cd ~/unitree/nav/src/go2_http_control
/usr/bin/python3 -m unittest discover -s test -p 'test_http_*.py' -v
```

### 4.3 源码打包与传输

开发机进入本 README 所在的 `nav/` 目录执行。同名源码包会被更新。

```sh
mkdir -p ~/nav-releases
tar --exclude='.git' --exclude='__pycache__' --exclude='.pytest_cache' \
  --exclude='*.pyc' --exclude='build' --exclude='install' --exclude='log' \
  -czf ~/nav-releases/nav-source.tar.gz \
  go2_http_control go2_wireless_controller go2_launcher readme.md
cd ~/nav-releases
sha256sum nav-source.tar.gz > nav-source.tar.gz.sha256
```

将 `机器狗IP` 替换为实际地址，登录用户按设备修改：

```sh
ssh unitree@机器狗IP 'mkdir -p ~/unitree/nav-updates'
scp ~/nav-releases/nav-source.tar.gz ~/nav-releases/nav-source.tar.gz.sha256 \
  unitree@机器狗IP:unitree/nav-updates/
```

### 4.4 安装产物打包与恢复（可选）

机器狗编译成功后执行，归档整个 `install/`：

```sh
cd ~/unitree/nav
mkdir -p ~/unitree/nav-releases
tar -czf ~/unitree/nav-releases/nav-install.tar.gz install readme.md
cd ~/unitree/nav-releases
sha256sum nav-install.tar.gz > nav-install.tar.gz.sha256
```

仅用于 OS、ROS、Python、CPU 架构、依赖及路径一致的环境。停止相关服务后，在机器狗恢复：

```sh
cd ~/unitree/nav-releases &&
sha256sum -c nav-install.tar.gz.sha256 &&
NAV_RESTORE=$(mktemp -d "$HOME/unitree/nav-releases/restore-XXXXXX") &&
tar -xzf nav-install.tar.gz -C "$NAV_RESTORE" &&
test -f "$NAV_RESTORE/install/setup.bash" &&
test -f "$NAV_RESTORE/readme.md" &&
NAV_BACKUP="$HOME/unitree/nav-backups/install-$(date +%Y%m%d-%H%M%S)" &&
mkdir -p "$NAV_BACKUP" &&
mv ~/unitree/nav/install "$NAV_BACKUP/" &&
mv "$NAV_RESTORE/install" ~/unitree/nav/ &&
cp "$NAV_RESTORE/readme.md" ~/unitree/nav/readme.md
```

### 4.5 旧版本回退

停止相关服务。将 `备份目录名` 替换为已有完整备份的目录名（须包含旧 `src/` 和 `install/`）；以下命令同时恢复全部旧源码和安装产物。

```sh
NAV_BACKUP="$HOME/unitree/nav-backups/备份目录名"
NAV_FAILED="$HOME/unitree/nav-backups/rollback-$(date +%Y%m%d-%H%M%S)"
cd ~/unitree/nav &&
test -f "$NAV_BACKUP/src/nav/package.xml" &&
test -f "$NAV_BACKUP/src/go2_start_button_sender/package.xml" &&
test -f "$NAV_BACKUP/install/setup.bash" &&
mkdir -p "$NAV_FAILED" &&
find . -maxdepth 1 -type d \( -name src -o -name build -o -name install -o -name log \) -exec mv -t "$NAV_FAILED" {} + &&
cp -a "$NAV_BACKUP/src" "$NAV_BACKUP/install" .
```

恢复原服务配置；原入口为 PlanB 时，新开终端执行：

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
ros2 launch nav nav_stubbing_enhanced_planB.launch.py
```

### 4.6 修改代码后重新编译

先按第 4.1 节停止服务并加载依赖。仅修改手柄节点时执行：

```sh
cd ~/unitree/nav
colcon build --packages-select go2_wireless_controller
source install/setup.bash
ros2 launch go2_launcher robot.launch.py
```

同时修改 HTTP、手柄或 launch 时执行：

```sh
cd ~/unitree/nav
colcon build --packages-select go2_http_control go2_wireless_controller go2_launcher
source install/setup.bash
ros2 launch go2_launcher robot.launch.py
```

## 5. 启动方式

### 5.1 默认启动

机器狗执行；启动前按第 4.1 节停止旧服务并确认端口已释放。

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
ros2 launch go2_launcher robot.launch.py
```

预期日志：`HTTP server started: http://0.0.0.0:10088`。

### 5.2 服务管理方式（可选）

与手工 launch 二选一。在机器狗的 `computing_service` 目录执行，同时启动 dog/dog-ctrl：

```sh
export NAV_DIR="$HOME/unitree/nav"
export NAV_SETUP=install/setup.bash
export NAV_LAUNCH_PACKAGE=go2_launcher
export NAV_LAUNCH_FILE=robot.launch.py
bash dog/start_dog_services.sh
```

vivo 环境使用已有的 `dog/start_dog_services_vivo.sh`。systemd 部署须将变量写入实际服务配置后重启。

## 6. 部署验收与故障排查

验收应覆盖安装文件来源、运行实例、消息通信和接口行为；离线测试通过后仍需在目标设备完成验证。

### 6.1 文件与可执行来源

机器狗另开终端执行，包和模块路径应位于 `~/unitree/nav/install/`：

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
ros2 pkg prefix go2_http_control
ros2 pkg prefix go2_wireless_controller
ros2 pkg prefix go2_launcher
ros2 pkg executables go2_http_control
ros2 pkg executables go2_wireless_controller
python3 -c 'import http_control.http_control_server as h; import go2_wireless_controller.wireless_controller_node as w; print(h.__file__); print(w.__file__)'
python3 -c 'from clients.sport_client import SportClient as C; print({n: callable(getattr(C, n, None)) for n in ("handshake", "hand_stand", "left_flip", "back_flip", "dance_1", "dance_2")})'
```

### 6.2 进程、节点和参数

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
ros2 node list
pgrep -af 'nav_stubbing|go2_start_button_sender|start_button_node|http_control_server|wireless_controller_node|robot.launch.py'
ss -ltnp '( sport = :10088 )'
ros2 node info /http_control_server
ros2 node info /wireless_controller
ros2 param get /wireless_controller nav_base_url
ros2 param get /wireless_controller arm_base_url
ros2 param get /wireless_controller button_testclient_base_url
```

默认 launch 应包含第 1 节的四个节点，HTTP 端口由预期实例监听。发现重复节点或端口冲突时，须先消除冲突再进行动作测试。

### 6.3 消息类型和数据流

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
ros2 topic list -t
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show unitree_go/msg/WirelessController
ros2 interface show unitree_api/msg/Request
ros2 interface show unitree_api/msg/Response
ros2 topic info /utlidar/robot_pose -v
ros2 topic info /wirelesscontroller -v
```

机器狗执行，每项观察 5 秒后自动继续：

```sh
source /opt/ros/foxy/setup.bash &&
source ~/unitree/nav/install/setup.bash &&
timeout 5s ros2 topic hz /utlidar/robot_pose
timeout 5s ros2 topic echo /ut/state
timeout 5s ros2 topic echo /wirelesscontroller
timeout 5s ros2 topic echo /api/sport/request
timeout 5s ros2 topic echo /api/sport/response
```

位姿连续断流超过 2 秒即失效；按键释放时 `keys` 应归零，按下时应符合第 3 节掩码。验收需确认实际消息流；数据缺失时检查发布端、DDS 网卡与域、环境变量及 QoS。

### 6.4 不触发运动的 HTTP 冒烟检查

```sh
curl --noproxy '*' -i http://127.0.0.1:10088/v1/status
curl --noproxy '*' -i http://127.0.0.1:10088/api/v1/status
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/coarse-shift \
  -H 'Content-Type: application/json' -d '{"distance":0}'
curl --noproxy '*' -i -X POST http://127.0.0.1:10088/v1/forward \
  -H 'Content-Type: application/json' -d '{}'
```

预期依次为：两个状态接口 200、零距离 200/`OK`、最后一个 400/`Missing timestamp`。该组检查没有提交实际移动任务。位姿尚未到达时 `pose_ready=false` 是预期状态，不代表 HTTP 服务启动失败。

HTTP 并发和取消行为由第 4.2 节离线测试验证。实机具备运行条件后，测试最小动作、停止、返回和手柄双击/长按，并核对日志与实际执行结果。实机控制期间，位姿 topic 须使用真实数据，禁止注入模拟位姿。

### 6.5 常见故障定位

| 现象 | 优先检查 |
| --- | --- |
| `Package ... not found` | source 顺序、构建包列表、`ros2 pkg prefix` 是否指向本次工作空间 |
| `executable ... not found` | `go2_http_control/setup.cfg` 安装路径是否正确；重新构建并重启 |
| `No module named clients/unitree_api/unitree_go` | 自定义支持包是否构建/source；是否混用了不同 Python 或 ROS 发行版 |
| 10088 被占用 / 同名节点重复 | 按第 4.1 节停止管理脚本启动的旧实例，并检查残留节点 |
| `/api/v1/status` 返回 405 或 `/v1/status` 返回 404 | 核查监听 10088 的服务版本、模块哈希及启动入口，确认进程已更新 |
| `pose_ready=false` / 移动返回 503 | 是否收到近期、有限值、非零四元数的 robot_pose；检查 DDS 和发布端 |
| 返回 408 | 请求体 timestamp 是否新鲜；服务端与调用端时钟是否一致；是否错误放在 query |
| `/v1/*` 返回 501 | 运行时 SportClient 是否提供该方法；特别检查 dance1 与 dance_1 命名差异 |
| START 长按没有流程 | 是否松开；是否达到 2 秒；testclient URL 是否配置；手柄是否忙碌 |
| 修改文件后仍是旧行为 | 是否将新包放在 src 的三个同级目录；是否重新构建并加载 install/setup.bash；管理脚本是否启动 go2_launcher；是否已重启运行进程 |
| 导航 busy 结束但未到目标 | 异步失败不会改变早先 200/202；检查导航日志、取消/超时与实际位姿 |
