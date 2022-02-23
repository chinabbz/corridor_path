## 参数设置
请首先将task9.txt中的参数按照src/task9.json的格式改写成json文件保存在task9/文件夹下

## 运行方式
在工作空间下运行如下指令：
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

