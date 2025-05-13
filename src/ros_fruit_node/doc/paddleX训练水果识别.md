## PaddleX 训练水果识别

### 环境及安装

- 官方网站

    - https://www.paddlepaddle.org.cn/paddle/paddleX

    - https://github.com/PaddlePaddle/PaddleX

- 推荐安装环境

    - 操作系统

        - Windows8/10（推荐Windows 10）

        - Mac OS 10.13+

        - Ubuntu 18.04+

        注：处理器需为x86_64架构，支持MKL

    - 训练硬件

        - GPU（仅Windows及Linux系统）

            - 推荐使用支持CUDA的NVIDIA显卡，例如：GTX 1070+以上性能的显卡

            - Windows系统X86_64驱动版本>=411.31

            - Linux系统X86_64驱动版本>=410.48

            - 显存8G以上

        - CPU

            - PaddleX当前支持您用本地CPU进行训练，但推荐使用GPU以获得更好的开发体验

            - 内存：建议8G以上

            - 硬盘空间：建议SSD剩余空间1T以上（非必须）

        注：PaddleX在Windows及Mac OS系统只支持单卡模型。Windows系统暂不支持NCCL

- 下载地址

    - https://www.paddlepaddle.org.cn/paddlex/download

### 数据集导入

- 创建数据集

    - 新建数据集，选择图像分类，点击创建

        ![](doc_image/%E6%96%B0%E5%BB%BA%E6%95%B0%E6%8D%AE%E9%9B%86.png)

- 导入数据

    - 选择数据所在文件夹的路径

        ![](doc_image/%E5%AF%BC%E5%85%A5%E6%95%B0%E6%8D%AE%E9%9B%86.png)

    - 数据集目录结构

        ![](doc_image/%E6%95%B0%E6%8D%AE%E9%9B%86%E7%9B%AE%E5%BD%95%E7%BB%93%E6%9E%84.png)

        将相同种类的水果放在同一个文件夹中

- 数据集切分

    - 将导入的数据按比例切分成训练集、验证集和测试集

        ![](doc_image/%E6%95%B0%E6%8D%AE%E9%9B%86%E5%88%87%E5%88%86.png)

- 切分完成后就可以在训练中使用该数据集

### 训练步骤（以水果识别为例）

- 新建项目

    - 新建项目，选择图像分类，点击创建

        ![](doc_image/%E6%96%B0%E5%BB%BA%E9%A1%B9%E7%9B%AE.png)

- 选择数据集

    - 选择训练需要的数据集

        ![](doc_image/%E9%80%89%E6%8B%A9%E6%95%B0%E6%8D%AE%E9%9B%86.png)

- 训练参数配置

    - 配置训练时的参数

        ![](doc_image/%E8%AE%AD%E7%BB%83%E5%8F%82%E6%95%B0%E9%85%8D%E7%BD%AE.png)

        - 选择网络模型

        - 如果有 gpu 设备，推荐 “使用 GPU 项” 为 “是”

        - 迭代轮数视情况而定，轮数太少可能导致训练结束但准确率低；轮数太多可能在训练结束前训练结果就已经收敛，剩下的轮数只是在做无用功。可以先设置较大训练轮数，判断训练结果已经收敛后中止训练；如果训练轮数不够多，可以先保存成预训练模型，在训练时使用自定义的预训练模型继续训练即可

- 训练可视化

    - 可以看到训练的状态，完成进度和剩余时间

        ![](doc_image/%E8%AE%AD%E7%BB%83%E5%8F%AF%E8%A7%86%E5%8C%96.png)

        待训练完成后对模型进行评估

- 模型评估

    - 可以看到各轮训练的 acc 变化

        ![](doc_image/acc1%E5%8F%98%E5%8C%96%E6%9B%B2%E7%BA%BF.png)

    - 测试集测试

        ![](doc_image/%E5%90%AF%E5%8A%A8%E6%B5%8B%E8%AF%95.png)

        启动测试，查看测试结果的图片

        可以选择保存预训练模型，方便对模型进行改进

- 模型发布

    - 在指定路径下保存成静态图模型

        ![](doc_image/%E6%A8%A1%E5%9E%8B%E5%8F%91%E5%B8%83.png)

### 模型部署

- 在机器人上为了避免安装训练框架，会将训练好的模型转成 onnx 的通用模型格式

- 在 paddlepaddle 中有提供 paddle2onnx 的接口帮助转换

- https://www.paddlepaddle.org.cn/documentation/docs/zh/guides/advanced/model_to_onnx_cn.html