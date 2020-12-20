#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS2 Node 受信側
#
# The MIT License (MIT)
# Copyright (C) 2020 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node
# カスタムメッセージ
from ert3_mes.msg import DioMsg
# PLC出力制御
from node.PlcDy import PlcDy


class MySubscription(Node):
    """
    受信側
    """

    # ノード名
    SELFNODE = "ert3sub"
    # トピック名
    SELFTOPIC = "mes_" + SELFNODE

    def __init__(self):
        """
        コンストラクタ
        """
        # ノードの初期化
        super().__init__(self.SELFNODE)
        # コンソールに表示
        self.get_logger().info(f"{self.SELFNODE} initializing...")
        # subscriptionインスタンスを生成
        self.sub = self.create_subscription(
            DioMsg, "mes_ert3pub", self.callback, 10)
        # コンソールに表示
        self.get_logger().info(f"{self.SELFNODE} do...")
        # PLC操作
        self.plcdy = PlcDy(2)

    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        self.get_logger().info(f"{self.SELFNODE} done.")

    def callback(self, message):
        """
        コールバック関数
        Parameters
        ----------
        message : DioMsg
            メッセージ
        """
        # 受け取ったメッセージの表示
        self.get_logger().info(
            f"Subscription > Relay: {message.relay} Value: {message.value}")

        # PLCの操作
        self.plcdy.write1(message.relay, message.value)


def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    try:
        # rclpyの初期化
        rclpy.init(args=args)
        # インスタンスを生成
        node = MySubscription()
        # プロセス終了までアイドリング
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
