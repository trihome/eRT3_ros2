#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS2 Node 送信側
#
# The MIT License (MIT)
# Copyright (C) 2020 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node
# カスタムメッセージ
from ert3_mes.msg import DioMsg


class MyPublisher(Node):
    """
    送信側
    """

    # ノード名
    SELFNODE = "ert3pub"
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
        # publisherインスタンスを生成
        self.pub = self.create_publisher(DioMsg, self.SELFTOPIC, 10)
        # タイマーのインスタンスを生成（1秒ごとに発生）
        self.create_timer(0.5, self.callback)
        # シーケンス番号をリセット
        self.sequence = 0
        # コンソールに表示
        self.get_logger().info(f"{self.SELFNODE} do...")

    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        self.get_logger().info(f"{self.SELFNODE} done.")

    def callback(self):
        """
        タイマーの実行部・歩行者信号の動作
        """

        # シーケンス制御
        if self.sequence == 0:
            # 初期化（一度だけ実行）
            pass
        if self.sequence == 1:
            # 赤色点灯
            self.sendmsg(1, 1)
            # 緑色消灯
            self.sendmsg(2, 0)
        elif self.sequence == 10:
            # 赤色消灯
            self.sendmsg(1, 0)
            # 緑色点灯
            self.sendmsg(2, 1)
        elif self.sequence in [21, 23, 25, 27, 29]:
            # 緑色消灯
            self.sendmsg(2, 0)
        elif self.sequence in [22, 24, 26, 28]:
            # 緑色点灯
            self.sendmsg(2, 1)
        elif self.sequence == 30:
            # 赤色点灯
            self.sendmsg(1, 1)
            # 緑色消灯
            self.sendmsg(2, 0)
        elif self.sequence > 30:
            # シーケンス終了
            self.sequence = 0

        # シーケンス番号をインクリメント
        self.sequence += 1

    def sendmsg(self, relay, value):
        """
        メッセージの送信
        Parameters
        ----------
        relay : int
            リレー番号を指定
        value : int
            0:消灯、1>:点灯
        """
        # 送信するメッセージの作成
        msg = DioMsg()
        msg.relay = relay
        msg.value = value
        # 送信
        self.pub.publish(msg)
        # ログの表示
        self.get_logger().info(
            f"Publish [{self.sequence} / relay: {relay}, val: {value}]")


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
        node = MyPublisher()
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
