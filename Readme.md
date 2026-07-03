# Go2 Nav 启动与时间同步说明

本项目包含导航相关节点, 以及 `go2_start_button_sender` START 按键监听节点。

当前重点功能: 双击遥控器 START 键后, 向多个 `/api/start/notify` 地址发送 POST 请求; 当某个地址返回 `200 OK` 且响应中包含 `timestamp` 时, 用该时间戳同步机器狗本机系统时间。

## START 按键功能

`go2_start_button_sender` 监听 ROS2 话题:

```bash
/wirelesscontroller
```

按键行为:

- 双击 START: 向以下地址发送空 JSON POST 请求:

```text
http://192.168.1.10:9100/api/start/notify
http://192.168.1.15:9100/api/start/notify
http://192.168.1.16:9100/api/start/notify
http://192.168.1.17:9100/api/start/notify
```

- 长按 START 2 秒: 向机器狗执行接口发送请求:

```text
http://192.168.123.99:18891/api/v1/exec
```

双击 `/api/start/notify` 的 `200 OK` 响应需要包含顶层 `timestamp` 字段:

```json
{"timestamp": 1783044149}
```

也支持毫秒时间戳:

```json
{"timestamp": 1783044149000}
```

## 时间同步免密配置

时间同步使用命令:

```bash
sudo -n date -s @<timestamp>
```

因为 ROS 节点运行时不能交互输入 sudo 密码, 所以需要给运行节点的用户配置免密执行 `date`。

先确认当前用户和 `date` 路径:

```bash
whoami
which date
```

假设用户是 `unitree`, `date` 路径是 `/usr/bin/date`, 执行:

```bash
sudo visudo -f /etc/sudoers.d/start_button_time_sync
```

写入:

```text
unitree ALL=(root) NOPASSWD: /usr/bin/date
```

保存后测试:

```bash
sudo -n /usr/bin/date -s @1783044149
```

如果 `which date` 输出的是 `/bin/date`, 则 sudoers 中也要改成:

```text
unitree ALL=(root) NOPASSWD: /bin/date
```

## 启动方式

常用启动文件:

```bash
ros2 launch nav nav_stubbing_enhanced_planB.launch.py
```

该 launch 会启动:

- `nav_stubbing_enhanced_planB`
- `go2_start_button_sender`

如需单独启动 START 按键节点:

```bash
ros2 run go2_start_button_sender start_button_node
```

## 日志判断

正常双击触发时会看到:

```text
第一次点击松开, 等待第二次...
>>> 双击 START 触发! 间隔: xxxms <<<
HTTP success [192.168.1.10:9100]: 200 OK
System time synced: ...
```

如果看到:

```text
System time sync failed: sudo: a password is required
```

说明免密 sudo 没有配置成功, 或运行节点的用户与 sudoers 中配置的用户不一致。

如果看到:

```text
HTTP failed [192.168.1.15:9100]: timed out
```

说明对应 IP 没有响应或网络不通。当前逻辑会继续尝试所有配置的 notify 地址, 但一次双击最多只同步一次时间。

## 重启后的配置

通过 `/etc/sudoers.d/start_button_time_sync` 写入的免密配置通常会在关机或重启后保留, 不需要每次重新配置。

需要重新配置的常见情况:

- System image is restored after reboot.
- The change was made inside a container or temporary system.
- The system is upgraded, factory reset, or reflashed.
- The runtime username changes.
- The `date` command path changes.
