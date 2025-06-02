#coding: utf-8

from smbus2 import SMBus
import time
import matplotlib.pyplot as plt # グラフ描画用ライブラリ
from datetime import datetime  # タイムスタンプ変換用

# --- 設定値 (環境に合わせて確認・変更してください) ---
I2C_BUS_NUMBER  = 1
I2C_ADDRESS = 0x76
# --- 設定値ここまで ---

bus = None
digT = []
digP = []
digH = []
t_fine = 0.0

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
    else: # 通常はLSB firstだが、センサーによってはMSB firstの場合もある
        value = (lsb << 8) | msb
    
    if signed:
        if value & 0x8000:
            value = value - 65536 # 16ビット符号付き整数の場合
    return value

def get_calib_param():
    global digT, digP, digH
    digT, digP, digH = [], [], []

    digT.append(read_word_data_signed(0x88))
    digT.append(read_word_data_signed(0x8A, signed=True))
    digT.append(read_word_data_signed(0x8C, signed=True))
    if None in digT: return False

    digP.append(read_word_data_signed(0x8E))
    for i in range(1, 9): # P2からP9まで (計8個)
        param = read_word_data_signed(0x90 + (i-1) * 2, signed=True)
        if param is None: return False
        digP.append(param)
    if None in digP: return False

    val_A1 = read_byte_data_signed(0xA1)
    if val_A1 is None: return False
    digH.append(val_A1) # H1

    val_E1 = read_word_data_signed(0xE1, signed=True)
    if val_E1 is None: return False
    digH.append(val_E1) # H2
    
    val_E3 = read_byte_data_signed(0xE3)
    if val_E3 is None: return False
    digH.append(val_E3) # H3

    e4 = read_byte_data_signed(0xE4)
    e5_lsb = read_byte_data_signed(0xE5)
    e6 = read_byte_data_signed(0xE6) # Boschデータシートでは0xE5の残りビットと0xE6を組み合わせる
    if e4 is None or e5_lsb is None or e6 is None: return False
    digH.append((e4 << 4) | (e5_lsb & 0x0F))       # H4
    digH.append(((e5_lsb >> 4) & 0x0F) | (e6 << 4)) # H5 - こちらの合成が一般的か確認
   

    val_E7 = read_byte_data_signed(0xE7, signed=True)
    if val_E7 is None: return False
    digH.append(val_E7) # H6
    
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
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048.0) * digP[5] # P6 (digP[5])
    v2 = v2 + ((v1 * digP[4]) * 2.0) # P5 (digP[4])
    v2 = (v2 / 4.0) + (digP[3] * 65536.0) # P4 (digP[3])
    
    # P1, P2, P3 は digP[0], digP[1], digP[2]
    var1_calc = (digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192.0)) / 8.0 # P3
    var2_calc = (digP[1] * v1) / 2.0 # P2
    v1_combined = (var1_calc + var2_calc) / 262144.0
    
    v1_main = ((32768.0 + v1_combined) * digP[0]) / 32768.0 # P1
    
    if v1_main == 0: return None
    
    pressure = ((1048576.0 - adc_P) - (v2 / 4096.0)) * 3125.0
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1_main
    else:
        pressure = (pressure / v1_main) * 2.0
    
    # P7, P8, P9 は digP[6], digP[7], digP[8]
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096.0 # P9
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0 # P8
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0) # P7
    return pressure / 100.0

def compensate_H(adc_H):
    global t_fine
    if not digH or len(digH) < 6 or None in digH or t_fine == 0.0: return None

    # H1-H6 は digH[0]-digH[5]
    var_h_calc = t_fine - 76800.0
    
    humidity = adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_h_calc) # H4, H5
    humidity = humidity * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h_calc * \
                       (1.0 + digH[2] / 67108864.0 * var_h_calc))) # H2, H6, H3
    humidity = humidity * (1.0 - digH[0] * humidity / 524288.0) # H1

    if humidity > 100.0: humidity = 100.0
    elif humidity < 0.0: humidity = 0.0
    return humidity

def read_raw_data():
    block = []
    for i in range(8):
        byte_val = read_byte_data_signed(0xF7 + i)
        if byte_val is None: return None, None, None
        block.append(byte_val)
    pres_raw = (block[0] << 12) | (block[1] << 4) | (block[2] >> 4)
    temp_raw = (block[3] << 12) | (block[4] << 4) | (block[5] >> 4)
    hum_raw  = (block[6] << 8)  |  block[7]
    return temp_raw, pres_raw, hum_raw

def read_compensated_data():
    temp_raw, pres_raw, hum_raw = read_raw_data()
    if temp_raw is None or pres_raw is None or hum_raw is None:
        return None, None, None
    temperature = compensate_T(temp_raw)
    if temperature is None:
        return None, None, None
    pressure = compensate_P(pres_raw)
    humidity = compensate_H(hum_raw)
    return temperature, pressure, humidity

def setup_sensor():
    osrs_t = 1; osrs_p = 1; osrs_h = 1
    mode   = 3; t_sb   = 5; filter_coeff = 0; spi3w_en = 0
    ctrl_hum_reg  = osrs_h & 0x07
    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
    config_reg    = (t_sb << 5) | (filter_coeff << 2) | spi3w_en
    if not write_reg(0xF2, ctrl_hum_reg): return False
    if not write_reg(0xF4, ctrl_meas_reg): return False
    if not write_reg(0xF5, config_reg): return False
    return True

def plot_data(readings):
    """収集したデータをグラフで表示する"""
    if not readings:
        print("グラフ描画用のデータがありません。")
        return

    timestamps = []
    temperatures = []
    pressures = []
    humidities = []

    for r in readings:
        # タイムスタンプをdatetimeオブジェクトに変換
        timestamps.append(datetime.fromtimestamp(r["timestamp_epoch"]))
        temperatures.append(r["temperature_celsius"])
        pressures.append(r["pressure_hpa"])
        humidities.append(r["humidity_percent"])

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True) # 3行1列のサブプロット

    # 温度グラフ
    axs[0].plot(timestamps, temperatures, marker='o', linestyle='-', color='r', label='Temperature')
    axs[0].set_ylabel('Temperature (°C)')
    axs[0].set_title('Temperature over Time')
    axs[0].grid(True)
    axs[0].legend()

    # 気圧グラフ
    axs[1].plot(timestamps, pressures, marker='s', linestyle='-', color='g', label='Pressure')
    axs[1].set_ylabel('Pressure (hPa)')
    axs[1].set_title('Pressure over Time')
    axs[1].grid(True)
    axs[1].legend()

    # 湿度グラフ
    axs[2].plot(timestamps, humidities, marker='^', linestyle='-', color='b', label='Humidity')
    axs[2].set_ylabel('Humidity (%)')
    axs[2].set_title('Humidity over Time')
    axs[2].grid(True)
    axs[2].legend()

    # X軸のフォーマット (日付時刻)
    fig.autofmt_xdate() # X軸ラベルが重ならないように自動調整
    plt.xlabel('Time')
    fig.suptitle('BME280 Sensor Data Log', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # suptitleとの重なりを調整
    plt.show()


def main():
    global bus
    try:
        bus = SMBus(I2C_BUS_NUMBER)
    except Exception as e:
        print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")
        return

    print("BME280センサーの初期化を開始します...")
    if not setup_sensor():
        print("センサーの動作モード設定に失敗しました。")
        if bus: bus.close()
        return
    print("センサーの動作モード設定完了。")

    if not get_calib_param():
        print("補正パラメータの読み出しに失敗しました。センサー接続を確認してください。")
        if bus: bus.close()
        return
    print("補正パラメータ読み出し完了。")

    all_readings = []
    measurement_duration = 60
    interval = 5
    start_time_script = time.time()
    
    print(f"\nセンサーデータの測定を開始します。測定時間: {measurement_duration}秒, 測定間隔: {interval}秒")
    print("Ctrl+Cで中断できます。")

    try:
        loop_count = 0
        while (time.time() - start_time_script) < measurement_duration:
            loop_iter_start_time = time.time()
            loop_count += 1
            
            temp, pres, hum = read_compensated_data()
            current_timestamp_epoch = time.time()
            
            # 簡易的な途中経過表示
            timestamp_str_now = time.strftime('%H:%M:%S', time.localtime(current_timestamp_epoch))
            temp_disp = f"{temp:.2f}°C" if temp is not None else "N/A"
            pres_disp = f"{pres:.2f}hPa" if pres is not None else "N/A"
            hum_disp = f"{hum:.2f}%" if hum is not None else "N/A"
            print(f"[{timestamp_str_now}] T:{temp_disp}, P:{pres_disp}, H:{hum_disp} (測定 #{loop_count})")

            all_readings.append({
                "timestamp_epoch": current_timestamp_epoch,
                "temperature_celsius": temp,
                "pressure_hpa": pres,
                "humidity_percent": hum
            })
            
            time_to_wait = interval - (time.time() - loop_iter_start_time)
            if time_to_wait > 0:
                time.sleep(time_to_wait)
        
        print("\n測定が完了しました。")

    except KeyboardInterrupt:
        print("\n測定がユーザーによって中断されました。")
    finally:
        if bus:
            bus.close()
            print("I2Cバスをクローズしました。")

    if all_readings:
        print("\n--- 最終測定結果一覧 ---")
        for i, reading in enumerate(all_readings):
            timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(reading["timestamp_epoch"]))
            temp_str = f"{reading['temperature_celsius']:.2f} ℃" if reading['temperature_celsius'] is not None else "N/A"
            pres_str = f"{reading['pressure_hpa']:.2f} hPa" if reading['pressure_hpa'] is not None else "N/A"
            hum_str  = f"{reading['humidity_percent']:.2f} %" if reading['humidity_percent'] is not None else "N/A"
            print(f"#{i+1} 時刻: {timestamp_str}, 温度: {temp_str}, 気圧: {pres_str}, 湿度: {hum_str}")
        
        # グラフ描画関数の呼び出し
        plot_data(all_readings)
    else:
        print("データは収集されませんでした。")

if __name__ == '__main__':
    main()
