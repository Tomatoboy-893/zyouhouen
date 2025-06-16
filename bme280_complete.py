#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import board
import busio
import adafruit_bme280.basic as adafruit_bme280

# --- グラフの設定 ---
GRAPH_WIDTH = 40  # グラフの最大幅（文字数）。ターミナルの横幅に合わせて調整してください。

# 各データの想定される値の範囲（この範囲を基準にグラフの長さが決まります）
TEMP_MIN, TEMP_MAX = 0, 40       # 温度: 0°C から 40°C
PRESSURE_MIN, PRESSURE_MAX = 980, 1030 # 気圧: 980hPa から 1030hPa
HUMIDITY_MIN, HUMIDITY_MAX = 0, 100   # 湿度: 0% から 100%
# --------------------

def clear_screen():
    """コンソール画面をクリアする関数"""
    # Windowsの場合は'cls'、それ以外（Linux, Mac）は'clear'
    os.system('cls' if os.name == 'nt' else 'clear')

def draw_bar(label, value, min_val, max_val):
    """
    現在の値を受け取り、スケール変換して横棒グラフを描画する関数
    """
    # 値が定義した範囲外に出てしまった場合、グラフが崩れないように調整
    value = max(min_val, min(value, max_val))
    
    # 現在の値をグラフの幅に合わせて、棒の長さに変換する
    # (現在の値 - 範囲の最小値) / (範囲の最大値 - 範囲の最小値) で、0から1の割合を計算
    bar_length = int( (value - min_val) / (max_val - min_val) * GRAPH_WIDTH )
    
    # 棒グラフの本体（'■'を計算した長さの分だけ繰り返す）
    bar = '■' * bar_length
    
    # ラベル、棒グラフ、数値の順で表示する
    # {label:6s} : ラベルを6文字幅で表示（見栄えを揃えるため）
    # {bar:<{GRAPH_WIDTH}s} : グラフ部分をGRAPH_WIDTHの幅で左詰めで表示
    print(f"{label:6s} [{bar:<{GRAPH_WIDTH}s}] {value:.1f}")

# --- メイン処理 ---
try:
    # I2Cバスの初期化
    i2c = busio.I2C(board.SCL, board.SDA)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    # 海面気圧を現在地の値に設定すると、より正確な標高が得られます（今回は使いませんが参考情報）
    # bme280.sea_level_pressure = 1013.25

    # 無限ループで計測と表示を繰り返す
    while True:
        # センサーから各データを取得
        temperature = bme280.temperature
        pressure = bme280.pressure
        humidity = bme280.humidity

        # 画面を一旦クリアする
        clear_screen()
        
        # ヘッダーを表示
        print("--- リアルタイムセンサーグラフ (BME280) ---")
        
        # 各データのグラフを描画
        draw_bar("温度", temperature, TEMP_MIN, TEMP_MAX)
        draw_bar("気圧", pressure, PRESSURE_MIN, PRESSURE_MAX)
        draw_bar("湿度", humidity, HUMIDITY_MIN, HUMIDITY_MAX)
        
        # フッターを表示
        print("------------------------------------------")
        print("（Ctrl + Cキーで終了します）")

        # 1秒待ってから次の計測へ
        time.sleep(1)

except KeyboardInterrupt:
    # Ctrl + C が押されたらループを抜けて終了メッセージを表示
    print("\n\n計測を終了しました。")
    
except (ValueError, RuntimeError) as e:
    # センサーが認識できないなどのエラーを捕捉
    print(f"\nセンサーの読み取りエラーが発生しました: {e}")
    print("I2Cの接続や設定を確認してください。")
