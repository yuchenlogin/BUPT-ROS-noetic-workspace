# ROS Noetic Through a Docker Container

## Docker Container Information
- **Base Image**: Ubuntu 20.04
- **ROS Distribution**: Noetic

## Usage

1. 在宿主机（非docker）终端**授权 Docker 连接 X Server**

    ```jsx
    xhost +local:docker
    ```

2. 在宿主机安装 Docker engine（**不是Docker Desktop！！！**）

   具体教程参考https://www.cnblogs.com/lqqgis/p/18276118

   如果是在windows下启动本容器则需要 Docker Desktop，教程自行搜索
4. 使用 vscode 打开本项目目录
    
    Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder).
    
6. 安装 vscode 插件 `Dev Container`
    
    按下快捷键 `Ctrl+P` 调出 Vscode Quick Open，输入：
    
    ```
    ext install ms-vscode-remote.remote-containers
    ```
    
7. 启动 DevContainer
    
    按下快捷键 `Ctrl+Shift+P`，输入 `rebuild`，选择 `Dev Containers: Rebuild and Reopen in Container`
    
8. 等待启动。第一次启动需要构筑镜像，可能需要打开代理软件的 TUN 模式。同时可能会提示 “用户不在Docker组中“，按照官方文档进行配置后重启便可以解决
    
    If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"
9. ~~在Coppeliasim官网 https://www.coppeliarobotics.com/ 安装 Ubuntu20.04 x86_64 版本Coppeliasim并解压到本项目~~  
    > 现在已知的情况是，我们的 ROS noetic 和原环境中的代码对 4.1.0 版本的 Coppeliasim 的支持会更好... 
   
    ```
    tar Jxvf CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu20_04.tar.xz
    ```
    *随后将文件夹放置与 src、scripts 等相同的根目录下 ，更名Coppeliasim文件夹为 `CoppeliaSim_Edu`*

10. 在 /workspace 目录下（就是默认bash目录） 输入 `catkin_make` 对 src 中的 pkgs 进行编译，可能需要耗费一些时间

11. Run task
    
    按下快捷键 `Ctrl+Shift+P`，输入 `task`，选择 `Tasks：Run Task`，即可畅享目前大部分需要的指令的一键启动

## Notice

> [!NOTE]  
> 1. Dockerfile中可更改部分已注释标出，其余部分请勿擅自修改
> 2. 有些情况下如果无法执行某些任务，新建终端或者重启容器基本上都能解决
