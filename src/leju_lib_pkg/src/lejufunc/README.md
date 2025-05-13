## ***slam.py***

- 启动slam节点

    ```
    def slaminit()
        ```
        param: None
        return: None
        ```
    ```

- slam路径追踪

    ```
    def SlamMoveTo(path_points_list=[])
        ```
        param: list path_points_list 单个或多个路径点组成的列表
        return: None
        ```

    e.g. `SlamMoveTo([[0.7, 0.0, 0.0], [1.0, 0.6, 0.0]])`
    ```

- 数字识别

    ```
    def IdentifyDigit(digit)
        ```
        param: int digit 1 ~ 4的单个数字
        return: list digit_pos [Upper_left, Lower_left, Upper_right, Lower_right]
        ```
    ```

- 海绵块识别

    ```
    def ColorDetect(color)
        ```
        param: string color [ "blue" | "yellow" ]
        return: string position [ "left" | "right" ]
        ```
    ```

- 关闭slam节点

    ```
    def killRGBD()
        ```
        @param None
        @type  None
        @return 如果关闭成功返回True，否则False
        @rtype bool
        ```
    ```
