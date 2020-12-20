#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# e-RT3出力制御
#
# The MIT License (MIT)
# Copyright (C) 2020 myasu.
# -----------------------------------------------

import ctypes
import time


class PlcDy():
    """
    e-RT3出力制御
    """

    # ライブラリのフルパス
    __LIBM3 = "/usr/local/lib/libm3.so.1"

    def __init__(self, slot, unit=0):
        """
        コンストラクタ
        Parameters
        ----------
        slot : int
            YDを設置しているスロット番号を指定
        unit : int
            ユニット番号を指定
        """
        # ライブラリ読み込み
        self.__libc = ctypes.cdll.LoadLibrary(self.__LIBM3)
        # pythonの変数をint型に変換
        self.__c_unit = ctypes.c_int(unit)
        self.__c_slot = ctypes.c_int(slot)
        # 全消灯
        self.write(0x0000, 0x0000)

    def __del__(self):
        """
        デストラクタ
        """
        # 全消灯
        self.write(0x0000, 0x0000)

    def write(self, data_upper, data_lower, mask_upper=0xffff, mask_lower=0xffff, pos=1, num=2):
        """
        リレー出力
        Parameters
        ----------
        data_upper : int
            上位2Byteの出力データ
        data_lower : int
            下位2Byteの出力データ
        mask_upper : int
            上位2Byteのマスクデータ
        mask_lower : int
            下位2Byteのマスクデータ
        pos : int
            リレー番号（操作先頭の番号）
        num : int
            書き込みブロック数
        """
        # リレー番号
        # ブロック読み込みの場合は1, 17, 33, (16n+1)･･･を指定
        c_pos = ctypes.c_int(pos)
        # 書き込みブロック数（1ブロック16点）
        c_num = ctypes.c_int(num)
        # 書き込みデータ格納バッファ用のshort型配列を作る
        # 要素数4のshort型の配列
        short_arr = ctypes.c_uint16 * 4
        # 32 x 2 点分の書き込みデータ作成
        data = [data_lower, data_upper]
        c_data = short_arr(*data)
        # 32 x 2 点分のマスクデータ作成
        mask = [mask_lower, mask_upper]
        c_mask = short_arr(*mask)

        # ライブラリ関数呼び出し
        self.__libc.writeM3OutRelay(
            self.__c_unit, self.__c_slot, c_pos, c_num, c_data, c_mask)

    def write1(self, pos, val):
        """
        リレー出力・1ch単位
        Parameters
        ----------
        pos : int
            リレー番号
        val : int
            値（0:OFF, 1>=:ON）
        """
        wpos = 0x01
        if 0 < pos <= 16:
            # 1～16のとき
            # ビットシフト準備
            pos -= 1
            # ビットシフト
            wpos = wpos << pos
            if val > 0:
                # 値が1以上の時はON
                self.write(0, wpos, 0, wpos)
            else:
                # 値がゼロの時はOFF
                self.write(0, 0, 0, wpos)
        elif 16 < pos <= 32:
            # 17～32のとき
            # ビットシフト準備
            pos = pos - 16 - 1
            # ビットシフト
            wpos = wpos << pos
            if val > 0:
                # 値が1以上の時はON
                self.write(wpos, 0, wpos, 0)
            else:
                # 値がゼロの時はOFF
                self.write(0, 0, wpos, 0)
        else:
            # リレー番号を超えたとき
            print(" * ERR: out of range")

    def sample(self):
        """
        出力動作サンプル
        4ビットずつ点灯・消灯を繰り返し
        """
        while True:
            # YD書き込み
            self.write(0xf00f, 0x00f0)
            # スリープ
            time.sleep(0.5)
            self.write(0x00f0, 0x0f00)
            time.sleep(0.5)
            self.write(0x0f00, 0xf00f)
            time.sleep(0.5)

    def sample1(self):
        """
        出力動作サンプル
        1ビットずつ点灯・消灯を繰り返し
        """
        for i in range(1, 33):
            yd.write1(i, 1)
            time.sleep(0.2)
            yd.write1(i, 0)


if __name__ == '__main__':
    """
    メイン
    """
    print("--- write sample ---")
    # YDを設置しているスロット番号を指定
    yd = PlcDy(2)
    try:
        # サンプル動作の実行
        yd.sample1()
    except KeyboardInterrupt:
        pass
    except:
        # 例外発生時にメッセージ
        import traceback
        traceback.print_exc()
