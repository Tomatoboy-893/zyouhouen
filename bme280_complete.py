# 必要なライブラリをインポート
from smbus2 import SMBus
import time
from datetime import datetime
import csv
import collections

# --- グラフ描画ライブラリ ---
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

I2C_BUS_NUMBER = 1
I2C_ADDRESS = 0x76
OUTPUT_CSV_FILE = 'bme280_log.csv'
# --- 保存する画像ファイル名とグラフ設定 ---
OUTPUT_IMAGE_FILE = 'bme280_graph.png'  # 保存する画像ファイル名
MAX_DATA_POINTS = 60                      # グラフに表示する最大データ点数
MEASUREMENT_INTERVAL_SEC = 10             # 測定間隔 (秒)

# --- グローバル変数 ---
bus = None
digT, digP, digH = [], [], []
t_fine = 0.0

# グラフデータ保持用のリスト (dequeを使うと効率的にデータを管理できる)
timestamps = collections.deque(maxlen=MAX_DATA_POINTS)
temperatures = collections.deque(maxlen=MAX_DATA_POINTS)
pressures = collections.deque(maxlen=MAX_DATA_POINTS)
humidities = collections.deque(maxlen=MAX_DATA_POINTS)

# グラフオブジェクト
fig, ax1 = None, None
ax2 = None
line_temp, line_hum, line_pres = None, None, None


def write_reg(reg_address, data):
    if bus is None:
        print("エラー: I2Cバスが初期化されていません。")
        return False
    try:
        bus.write_byte_data(I2C_ADDRESS, reg_address, data)
        return True
    except IOError as e:
        print(f"エラー: I2C書き込み失敗 (アドレス {hex(I2C_ADDRESS)}, レジスタ {hex(reg_address)}): {e}")
        return False

def read_byte_data_signed(reg_address, signed=False):
    if bus is None:
        print("エラー: I2Cバスが初期化されていません。")
        return None
    try:
        value = bus.read_byte_data(I2C_ADDRESS, reg_address)
        if signed:
            if value > 127:
                value -= 256
        return value
    except IOError as e:
        print(f"エラー: I2C読み込み失敗 (アドレス {hex(I2C_ADDRESS)}, レジスタ {hex(reg_address)}): {e}")
        return None

def read_word_data_signed(reg_address, lsb_first=True, signed=False):
    lsb_addr = reg_address
    msb_addr = reg_address + 1
    
    lsb = read_byte_data_signed(lsb_addr)
    msb = read_byte_data_signed(msb_addr)

    if lsb is None or msb is None:
        return None

    if lsb_first:
        value = (msb << 8) | lsb
    else:
        value = (lsb << 8) | msb
    
    if signed:
        if value & 0x8000:
            value = value - 65536
    return value

def get_calib_param():
    global digT, digP, digH
    digT, digP, digH = [], [], []

    digT.append(read_word_data_signed(0x88))
    digT.append(read_word_data_signed(0x8A, signed=True))
    digT.append(read_word_data_signed(0x8C, signed=True))
    if None in digT: return False

    digP.append(read_word_data_signed(0x8E))
    for i in range(1, 9):
        param = read_word_data_signed(0x90 + (i-1) * 2, signed=True)
        if param is None: return False
        digP.append(param)
    if None in digP: return False

    val_A1 = read_byte_data_signed(0xA1)
    if val_A1 is None: return False
    digH.append(val_A1)

    val_E1 = read_word_data_signed(0xE1, signed=True)
    if val_E1 is None: return False
    digH.append(val_E1)
    
    val_E3 = read_byte_data_signed(0xE3)
    if val_E3 is None: return False
    digH.append(val_E3)

    e4 = read_byte_data_signed(0xE4)
    e5_lsb = read_byte_data_signed(0xE5)
    e6 = read_byte_data_signed(0xE6)
    if e4 is None or e5_lsb is None or e6 is None: return False
    digH.append((e4 << 4) | (e5_lsb & 0x0F))
    digH.append(((e5_lsb >> 4) & 0x0F) | (e6 << 4))

    val_E7 = read_byte_data_signed(0xE7, signed=True)
    if val_E7 is None: return False
    digH.append(val_E7)
    
    return True

def compensate_T(adc_T):
    global t_fine
    if not digT or len(digT) < 3 or digT[0] is None or digT[1] is None or digT[2] is None: return None
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = ((adc_T / 131072.0 - digT[0] / 8192.0)**2) * digT[2]
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    return temperature

def compensate_P(adc_P):
    global t_fine
    if not digP or len(digP) < 9 or None in digP or t_fine == 0.0: return None
    
    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048.0) * digP[5]
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    
    var1_calc = (digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192.0)) / 8.0
    var2_calc = (digP[1] * v1) / 2.0
    v1_combined = (var1_calc + var2_calc) / 262144.0
    
    v1_main = ((32768.0 + v1_combined) * digP[0]) / 32768.0
    
    if v1_main == 0: return None
    
    pressure = ((1048576.0 - adc_P) - (v2 / 4096.0)) * 3125.0
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1_main
    else:
        pressure = (pressure / v1_main) * 2.0
    
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096.0
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)
    return pressure / 100.0

def compensate_H(adc_H):
    global t_fine
    if not digH or len(digH) < 6 or None in digH or t_fine == 0.0: return None

    var_h_calc = t_fine - 76800.0
    
    humidity = adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_h_calc)
    humidity = humidity * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h_calc * \
                        (1.0 + digH[2] / 67108864.0 * var_h_calc)))
    humidity = humidity * (1.0 - digH[0] * humidity / 524288.0)

    if humidity > 100.0: humidity = 100.0
    elif humidity < 0.0: humidity = 0.0
    return humidity

def read_raw_data():
    try:
        block = bus.read_i2c_block_data(I2C_ADDRESS, 0xF7, 8)
        pres_raw = (block[0] << 12) | (block[1] << 4) | (block[2] >> 4)
        temp_raw = (block[3] << 12) | (block[4] << 4) | (block[5] >> 4)
        hum_raw  = (block[6] << 8)  |  block[7]
        return temp_raw, pres_raw, hum_raw
    except IOError as e:
        print(f"エラー: I2Cブロックデータ読み込み失敗: {e}")
        return None, None, None

def read_compensated_data():
    temp_raw, pres_raw, hum_raw = read_raw_data()
    if temp_raw is None:
        return None, None, None
    
    temperature = compensate_T(temp_raw)
    if temperature is None:
        return None, None, None
        
    pressure = compensate_P(pres_raw)
    humidity = compensate_H(hum_raw)
    return temperature, pressure, humidity

def setup_sensor():
    osrs_t = 1; osrs_p = 1; osrs_h = 1
    mode = 3; t_sb = 5; filter_coeff = 0; spi3w_en = 0
    
    ctrl_hum_reg = osrs_h & 0x07
    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg = (t_sb << 5) | (filter_coeff << 2) | spi3w_en
    
    if not write_reg(0xF2, ctrl_hum_reg): return False
    if not write_reg(0xF4, ctrl_meas_reg): return False
    if not write_reg(0xF5, config_reg): return False
    
    return True

def init_graph():
    """グラフの初期設定を行う"""
    global fig, ax1, ax2, line_temp, line_hum, line_pres
    fig, ax1 = plt.subplots(figsize=(12, 7))
    fig.canvas.manager.set_window_title('BME280 リアルタイムモニター')

    # 軸1 (左): 温度(赤)と湿度(青)
    ax1.set_xlabel("Time")
    ax1.set_ylabel("Temperature (°C) / Humidity (%)")
    line_temp, = ax1.plot([], [], 'r-o', markersize=3, label='Temperature (°C)')
    line_hum, = ax1.plot([], [], 'b-o', markersize=3, label='Humidity (%)')
    ax1.tick_params(axis='y', labelcolor='tab:red')
    ax1.grid(True)

    # 軸2 (右): 気圧(緑)
    ax2 = ax1.twinx()
    ax2.set_ylabel("Pressure (hPa)")
    line_pres, = ax2.plot([], [], 'g-o', markersize=3, label='Pressure (hPa)')
    ax2.tick_params(axis='y', labelcolor='tab:green')

    # 凡例をまとめて表示
    lines = [line_temp, line_hum, line_pres]
    ax1.legend(lines, [l.get_label() for l in lines], loc='upper left')

    fig.tight_layout()
    plt.title("BME280 Sensor Data (Real-time)")

def update_graph(frame):
    """グラフを更新する (FuncAnimationから定期的に呼ばれる)"""
    # センサーデータ読み取り
    temp, pres, hum = read_compensated_data()
    timestamp = datetime.now()

    if temp is not None and pres is not None and hum is not None:
        # 画面への表示
        print(f"[{timestamp.strftime('%H:%M:%S')}] "
              f"T:{temp:.2f}C, P:{pres:.2f}hPa, H:{hum:.2f}% ... 記録中")

        # データをリストに追加
        timestamps.append(timestamp)
        temperatures.append(temp)
        pressures.append(pres)
        humidities.append(hum)

        # CSVファイルに追記
        try:
            with open(OUTPUT_CSV_FILE, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp.isoformat(), f"{temp:.2f}", f"{pres:.2f}", f"{hum:.2f}"])
        except IOError as e:
            print(f"警告: CSVファイルへの書き込みに失敗しました: {e}")

        # グラフデータを更新
        line_temp.set_data(list(timestamps), list(temperatures))
        line_hum.set_data(list(timestamps), list(humidities))
        line_pres.set_data(list(timestamps), list(pressures))

        # 軸の範囲を自動調整
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        
        # X軸のラベルが重ならないように回転
        plt.setp(ax1.get_xticklabels(), rotation=30, ha="right")

    else:
        print(f"[{timestamp.strftime('%H:%M:%S')}] データ取得に失敗しました。")

    return line_temp, line_hum, line_pres

def main():
    """メイン処理"""
    global bus
    try:
        bus = SMBus(I2C_BUS_NUMBER)
    except Exception as e:
        print(f"エラー: I2Cバスのオープンに失敗 (バス番号 {I2C_BUS_NUMBER}): {e}")
        return

    print("BME280センサーの初期化を開始します...")
    if not setup_sensor() or not get_calib_param():
        print("センサーの初期化または補正パラメータの読み出しに失敗しました。")
        if bus: bus.close()
        return
    print("センサー初期化完了。")

    # CSVファイルの準備 (ヘッダー行の書き込み)
    try:
        with open(OUTPUT_CSV_FILE, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'temperature_c', 'pressure_hpa', 'humidity_percent'])
        print(f"データは {OUTPUT_CSV_FILE} に保存されます。")
    except IOError as e:
        print(f"エラー: CSVファイル '{OUTPUT_CSV_FILE}' の準備ができませんでした: {e}")
        if bus: bus.close()
        return

    # グラフの初期化
    init_graph()

    # アニメーションの設定 (intervalはミリ秒単位)
    ani = FuncAnimation(fig, update_graph,
                        interval=MEASUREMENT_INTERVAL_SEC * 1000,
                        blit=False,
                        cache_frame_data=False)

    try:
        print(f"\nセンサーデータの測定とグラフ表示を開始します。測定間隔: {MEASUREMENT_INTERVAL_SEC}秒")
        print("グラフウィンドウを閉じるか、コンソールで Ctrl+C を押すと中断できます。")
        plt.show()  # グラフウィンドウを表示し、アニメーションを開始
    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")
    finally:
        print("\n処理を終了します。")
        # グラフを画像として保存
        try:
            print(f"最終的なグラフを {OUTPUT_IMAGE_FILE} に保存しています...")
            fig.savefig(OUTPUT_IMAGE_FILE, dpi=150) # dpiで解像度を指定
            print("保存しました。")
        except Exception as e:
            print(f"エラー: グラフの保存に失敗しました: {e}")

        if bus:
            bus.close()
            print("I2Cバスをクローズしました。")

if __name__ == '__main__':
    main()
