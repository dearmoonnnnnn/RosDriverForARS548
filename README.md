# 项目介绍

- ARS548 4D毫米波雷达 ROS 驱动。
- 将 pcapng 文件转换为 bag 文件

### 零、整体思路：

##### 两个项目：

- RosDriverForARS548：
  - 接受 json 格式的传感器数据，并用以 ROS 消息的格式发布
- rosbag_recorder
  - 订阅 RosDriverForARS548 的话题，并生成 bag 文件

##### 流程：

1. 使用 wireshark 将 pcap(pcapng) 的解析结果保存为 json 文件

   注意需要 ars548 插件

1. 使用 RosDriverForARS548，直接从 json 文件读取解析结果和字符流，并发布相关话题

1. 同时使用 rosbag_recorder 订阅话题，转为 bag 文件

##### 运行环境：

- Ubuntu18.04  + ROS melodic 或 Ubuntu20.04 + ROS noetic

- nlohmann-json库
- wireshark

### 一、配置环境：

##### 1、安装ROS

参考如下链接：

官方文档：http://wiki.ros.org/ROS/Installation

其他教程：https://blog.csdn.net/sea_grey_whale/article/details/132023522

Autolabor（推荐）：http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html

##### 2、安装wireshark

1. 安装wireshark软件

   ```bash
   sudo apt install wireshark
   ```

   出现弹窗，选择“是”，允许 Wireshark 捕获网络数据包。

2. 配置 Wireshark 插件，使其能够解析 ars548 传感器数据。

   找到插件需要放置的位置：`/usr/lib/x86_64-linux-gnu/wireshark/plugins`

   ![wireshark插件位置](https://raw.githubusercontent.com/letMeEmoForAWhile/typoraImage/main/img/wireshark插件位置.png)

   在插件所在目录，复制插件到上述位置

   ```bash
   sudo cp packet-ars548（大陆原版）.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins
   ```

   重启 Wireshark

##### 3、安装nlohmann

- apt安装

     ```bash
     sudo apt update
     sudo apt install nlohmann-json3-dev
     ```


   - 源码安装: https://blog.csdn.net/jiemashizhen/article/details/129275915

     ```bash
     // 在你喜欢的位置
     git clone  https://github.com/nlohmann/json.git
     cd json
     mkdir build
     cd build
     cmake ..
     make
     sudo make install
     ```

##### 4、安装libpcap

`RosDriverForARS548`需要该库。

```bash
sudo apt-get install libpcap-dev
```

### 二、使用wireshark将传感器数据转换为json文件

##### 1、使用wireshark打开抓取的pcapng文件

雷达厂商提供了传感器的lua插件，可以直接过滤，只保留`detectionlist`数据

![image-20240119161238901](https://raw.githubusercontent.com/letMeEmoForAWhile/typoraImage/main/img/image-20240119161238901.png)

##### 2、导出解析结果为JSON格式

![屏幕截图 2024-01-19 16:16:54](https://raw.githubusercontent.com/letMeEmoForAWhile/typoraImage/main/img/image-2024-01-19-16:16:54.png)

### 三、RosDriverForARS548

##### 0、修改路径

在`ars548_process_node.cpp`中修改`json_file_path`为步骤一中的json文件路径

##### 1、编译

1）下载项目：

```bash
cd ~/catkin_ws/src
git clone https://github.com/dearmoonnnnnn/RosDriverForARS548.git
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```

2）在 RosDriverForARS548 目录内创建include文件夹

- 不创建会报错：https://github.com/wulang584513/ARS548-demo/issues/3

3）切换到工作空间根目录  `catkin_ws` , 编译

```bash
catkin_make
```

4）保存环境变量

```bash
vim ~/.bashrc
```

尾行添加以下内容

```bash
source PATH_TO_catkin_ws_FOLDER/devel/setup.bash
```

-  `PATH_TO_catkin_ws_FOLDER`：工作空间根目录的绝对路经。

5 ) 保存 `.bashrc` 文件；执行以下命令，使环境变量立即生效

```bash
source ~/.bashrc
```

##### 2、运行

启动节点

```bash
roslaunch ars548_process ars548_process.launch
```

### 四、rosbag_tools

##### 0、修改路径

在`rosbag_recorder.cpp` 中修改 `bag.open()` 参数为输出的bag文件路径。

##### 1、编译

1）下载项目：

```bash
git clone https://github.com/dearmoonnnnnn/rosbag_tools.git
```

- 注意这里的 `rosbag_tools` 即 `catkin_ws` 

2）在项目根路径执行编译命令

```bash
catkin_make
```

3）保存环境变量

```bash
vim ~/.bashrc
```

尾行添加以下内容。

```bash
source PATH_TO_rosbag_tools_FOLDER/devel/setup.bash
```

- `PATH_TO_rosbag_tools_FOLDER` 即 `rosbag_tools` 目录的绝对路径

4）保存 `.bashrc` 文件；执行以下命令

```bash
source ~/.bashrc
```

##### 2、运行

先启动 RosDriverForARS548，再启动该项目。

```bash
rosrun rosbag_tools rosbag_recorder 
```

### 五、后言

本项目基于以下项目修改：

https://github.com/wulang584513/ARS548-demo/tree/master

修改内容：

1. 原项目希望解析 `UDP` 数据作为数据入口，但未实现
   - 本项目使用 `wireshark` 解析结果的 `json` 文件作为输入
2. 原项目发布的点云消息只包含位置信息
   - 本项目增加了多普勒速度和信号强度（`RCS`），存储在`sensor_msgs::PointCloud` 额外的两个通道中
3. 原项目 `RCS` 数据类型定义错误，导致发布时该值与 `wireshark` 解析结果不符
   - 修改定义的结构体和自定义消息文件

具体修改见：

https://github.com/dearmoonnnnnn/Notes/blob/main/%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6/%E4%BC%A0%E6%84%9F%E5%99%A8/ars548%E9%9B%B7%E8%BE%BEROS%E9%A9%B1%E5%8A%A8.md
