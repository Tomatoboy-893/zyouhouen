/////////ライブラリのインストール/////

pip3 install smbus2 matplotlib

/////////ここから下がコード/////////
#coding: utf-8

from smbus2 import SMBus
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation # アニメーション用
from datetime import datetime, timedelta      # タイムスタンプと時間差の扱い
import collections # dequeを使うため (固定長のデータ保持に便利)

# --- 設定値 (環境に合わせて確認・変更してください) ---
I2C_BUS_NUMBER  = 1
I2C_ADDRESS = 0x76
# --- 設定値ここまで ---

# --- グラフ設定 ---
UPDATE_INTERVAL_MS = 5000  # グラフ更新間隔 (ミリ秒) = 5秒
MAX_DATA_POINTS = 20       # グラフに表示する最大データポイント数 (スライディングウィンドウ)
# --- グラフ設定ここまで ---

# グローバル変数
bus = None
digT, digP, digH = [], [], []
t_fine = 0.0

# リアルタイムプロット用データ保持 (dequeで固定長を効率的に扱う)
timestamps_list = collections.deque(maxlen=MAX_DATA_POINTS)
temperatures_list = collections.deque(maxlen=MAX_DATA_POINTS)
pressures_list = collections.deque(maxlen=MAX_DATA_POINTS)
humidities_list = collections.deque(maxlen=MAX_DATA_POINTS)

# MatplotlibのFigureとAxesオブジェクト (グローバルで保持)
fig, axs = None, None
line_temp, line_pres, line_hum = None, None, None

# --- BME280 センサー制御関数 (以前のコードから変更点は少ない) ---
# (write_reg, read_byte_data_signed, read_word_data_signed, get_calib_param,
#  compensate_T, compensate_P, compensate_H, read_raw_data,
#  read_compensated_data, setup_sensor 関数は前の回答のものをほぼそのまま使用します。
#  エラーハンドリングや補正パラメータの読み込み精度はそちらを参照してください。)

def write_reg(reg_address, data):
    if bus is None: return False
    try:
        bus.write_byte_data(I2C_ADDRESS, reg_address, data)
        return True
    except IOError: return False

def read_byte_data_signed(reg_address, signed=False):
    if bus is None: return None
    try:
        value = bus.read_byte_data(I2C_ADDRESS, reg_address)
        if signed and value > 127: value -= 256
        return value
    except IOError: return None

def read_word_data_signed(reg_address, lsb_first=True, signed=False):
    lsb = read_byte_data_signed(reg_address)
    msb = read_byte_data_signed(reg_address + 1)
    if lsb is None or msb is None: return None
    value = (msb << 8) | lsb if lsb_first else (lsb << 8) | msb
    if signed and (value & 0x8000): value -= 65536
    return value

def get_calib_param():
    global digT, digP, digH
    digT, digP, digH = [], [], []
    try:
        digT.append(read_word_data_signed(0x88))
        digT.append(read_word_data_signed(0x8A, signed=True))
        digT.append(read_word_data_signed(0x8C, signed=True))
        if None in digT: raise ValueError("Temp calib failed")

        digP.append(read_word_data_signed(0x8E))
        for i in range(1, 9):
            digP.append(read_word_data_signed(0x90 + (i-1) * 2, signed=True))
        if None in digP: raise ValueError("Pressure calib failed")

        digH.append(read_byte_data_signed(0xA1))
        digH.append(read_word_data_signed(0xE1, signed=True))
        digH.append(read_byte_data_signed(0xE3))
        e4 = read_byte_data_signed(0xE4); e5 = read_byte_data_signed(0xE5); e6 = read_byte_data_signed(0xE6)
        if e4 is None or e5 is None or e6 is None: raise ValueError("Humidity partial calib failed")
        digH.append((e4 << 4) | (e5 & 0x0F))
        digH.append(((e5 >> 4) & 0x0F) | (e6 << 4)) #  Bosch DS Rev 1.13 Fig 11
        digH.append(read_byte_data_signed(0xE7, signed=True))
        if None in digH or len(digH) < 6: raise ValueError("Humidity calib failed")
    except Exception as e:
        print(f"補正パラメータ取得エラー: {e}")
        return False
    return True

def compensate_T(adc_T):
    global t_fine
    if not digT or len(digT) < 3: return None
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = ((adc_T / 131072.0 - digT[0] / 8192.0)**2) * digT[2]
    t_fine = v1 + v2
    return t_fine / 5120.0

def compensate_P(adc_P):
    global t_fine
    if not digP or len(digP) < 9 or t_fine == 0.0: return None
    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0)**2) / 2048.0) * digP[5]
    v2 += ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    var1_calc = (digP[2] * (((v1 / 4.0)**2) / 8192.0)) / 8.0
    var2_calc = (digP[1] * v1) / 2.0
    v1_main = ((32768.0 + (var1_calc + var2_calc) / 262144.0) * digP[0]) / 32768.0
    if v1_main == 0: return None
    pressure = ((1048576.0 - adc_P) - (v2 / 4096.0)) * 3125.0
    pressure = (pressure * 2.0) / v1_main if pressure < 0x80000000 else (pressure / v1_main) * 2.0
    v1 = (digP[8] * (((pressure / 8.0)**2) / 8192.0)) / 4096.0
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    return (pressure + ((v1 + v2 + digP[6]) / 16.0)) / 100.0

def compensate_H(adc_H):
    global t_fine
    if not digH or len(digH) < 6 or t_fine == 0.0: return None
    var_h_calc = t_fine - 76800.0
    humidity = adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_h_calc)
    humidity *= (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h_calc * (1.0 + digH[2] / 67108864.0 * var_h_calc)))
    humidity *= (1.0 - digH[0] * humidity / 524288.0)
    return max(0.0, min(humidity, 100.0))

def read_raw_data():
    block = []
    for i in range(8):
        byte_val = read_byte_data_signed(0xF7 + i)
        if byte_val is None: return None, None, None
        block.append(byte_val)
    return (block[3] << 12) | (block[4] << 4) | (block[5] >> 4), \
           (block[0] << 12) | (block[1] << 4) | (block[2] >> 4), \
           (block[6] << 8)  |  block[7]

def read_compensated_data():
    temp_raw, pres_raw, hum_raw = read_raw_data()
    if temp_raw is None: return None, None, None
    temperature = compensate_T(temp_raw)
    if temperature is None: return None, None, None
    pressure = compensate_P(pres_raw)
    humidity = compensate_H(hum_raw)
    return temperature, pressure, humidity

def setup_sensor():
    osrs_t = 1; osrs_p = 1; osrs_h = 1; mode = 3; t_sb = 5; filter_coeff = 0
    if not write_reg(0xF2, osrs_h & 0x07): return False
    if not write_reg(0xF4, (osrs_t << 5) | (osrs_p << 2) | mode): return False
    if not write_reg(0xF5, (t_sb << 5) | (filter_coeff << 2) | 0): return False
    return True

# --- グラフ初期化関数 ---
def init_plot():
    """グラフの初期設定を行う (FuncAnimationのinit_func用)"""
    global line_temp, line_pres, line_hum
    
    # 温度プロット
    axs[0].set_ylabel('Temperature (°C)')
    axs[0].set_title('Temperature')
    axs[0].grid(True)
    line_temp, = axs[0].plot([], [], marker='o', linestyle='-', color='r', label='Temp')
    axs[0].legend(loc='upper left', fontsize='small')
    
    # 気圧プロット
    axs[1].set_ylabel('Pressure (hPa)')
    axs[1].set_title('Pressure')
    axs[1].grid(True)
    line_pres, = axs[1].plot([], [], marker='s', linestyle='-', color='g', label='Pressure')
    axs[1].legend(loc='upper left', fontsize='small')

    # 湿度プロット
    axs[2].set_ylabel('Humidity (%)')
    axs[2].set_title('Humidity')
    axs[2].grid(True)
    line_hum, = axs[2].plot([], [], marker='^', linestyle='-', color='b', label='Humidity')
    axs[2].legend(loc='upper left', fontsize='small')

    fig.autofmt_xdate() # X軸の日時ラベルをいい感じにフォーマット
    axs[2].set_xlabel('Time') # 一番下のサブプロットにX軸ラベル
    fig.suptitle('BME280 Real-time Sensor Data', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # suptitleとの重なり調整
    
    # Y軸の初期範囲 (適宜調整)
    axs[0].set_ylim(10, 40)  # 温度用
    axs[1].set_ylim(950, 1050) # 気圧用
    axs[2].set_ylim(0, 100)   # 湿度用

    return line_temp, line_pres, line_hum, # blit=True のためにartistを返す

# --- グラフ更新関数 ---
def update_plot(frame):
    """指定された間隔で呼び出され、グラフを更新する (FuncAnimationのfunc用)"""
    temp, pres, hum = read_compensated_data()
    current_time = datetime.now()

    if temp is not None: temperatures_list.append(temp)
    else: temperatures_list.append(None) # データ欠損の場合も長さを合わせるためNoneを追加
    
    if pres is not None: pressures_list.append(pres)
    else: pressures_list.append(None)
        
    if hum is not None: humidities_list.append(hum)
    else: humidities_list.append(None)
    
    timestamps_list.append(current_time)

    # Y軸の自動調整 (Noneをフィルタリング)
    valid_temps = [t for t in temperatures_list if t is not None]
    if valid_temps: axs[0].set_ylim(min(valid_temps) - 2, max(valid_temps) + 2)
    
    valid_pres = [p for p in pressures_list if p is not None]
    if valid_pres: axs[1].set_ylim(min(valid_pres) - 5, max(valid_pres) + 5)
    
    valid_hums = [h for h in humidities_list if h is not None]
    if valid_hums: axs[2].set_ylim(min(valid_hums) - 5, max(valid_hums) + 5)

    # データをセット
    line_temp.set_data(list(timestamps_list), list(temperatures_list))
    line_pres.set_data(list(timestamps_list), list(pressures_list))
    line_hum.set_data(list(timestamps_list), list(humidities_list))

    # X軸の範囲を更新 (スライディングウィンドウ)
    if timestamps_list:
        axs[0].set_xlim(timestamps_list[0], timestamps_list[-1] + timedelta(seconds=1)) # 少し未来まで表示

    # 途中経過をコンソールにも表示
    ts_str = current_time.strftime('%H:%M:%S')
    t_str = f"{temp:.2f}°C" if temp is not None else "N/A"
    p_str = f"{pres:.2f}hPa" if pres is not None else "N/A"
    h_str = f"{hum:.2f}%" if hum is not None else "N/A"
    print(f"[{ts_str}] T:{t_str}, P:{p_str}, H:{h_str}")

    return line_temp, line_pres, line_hum, # blit=True のためにartistを返す

# --- メイン処理 ---
def main():
    global bus, fig, axs # グローバル変数を参照
    try:
        bus = SMBus(I2C_BUS_NUMBER)
    except Exception as e:
        print(f"エラー: I2Cバス ({I2C_BUS_NUMBER}) のオープンに失敗: {e}")
        return

    print("BME280センサー初期化中...")
    if not setup_sensor():
        print("センサー設定失敗。")
        if bus: bus.close()
        return
    print("センサー設定完了。")

    if not get_calib_param():
        print("補正パラメータ取得失敗。")
        if bus: bus.close()
        return
    print("補正パラメータ取得完了。リアルタイムプロットを開始します...")

    # プロットの準備
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # アニメーションの開始
    # blit=True は効率的な描画のため。init_func と update_plot が artist のシーケンスを返す必要あり。
    ani = FuncAnimation(fig, update_plot, init_func=init_plot, 
                        interval=UPDATE_INTERVAL_MS, blit=True, cache_frame_data=False)
    
    try:
        plt.show() # グラフウィンドウを表示し、アニメーションを実行
    except Exception as e:
        print(f"プロット中にエラーが発生しました: {e}")
    finally:
        if bus:
            bus.close()
            print("I2Cバスをクローズしました。")
        print("プログラムを終了します。")

if __name__ == '__main__':
    main()
