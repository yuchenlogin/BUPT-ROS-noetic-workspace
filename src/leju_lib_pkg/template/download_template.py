#!/usr/bin/env python
# coding=utf-8

from lejulib import *
clear_function = {}

def main():
    node_initial()

    try:
        pass    # 用户放在开始模块中的内容

        rospy.signal_shutdown("done")
        rospy.spin()
    except Exception as err:
        serror(err)
    finally:
        for name in clear_function:
            clear_function.get(name)()

if __name__ == '__main__':
    main()
