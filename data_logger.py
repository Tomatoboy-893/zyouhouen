# 必要なライブラリをインポート

from smbus2 import SMBus
import time
from datetime import datetime
import csv  

I2C_BUS_NUMBER = 1
I2C_ADDRESS = 0x76
# --- 追加: 保存するCSVファイル名 ---
OUTPUT_CSV_FILE = 'bme280_log.csv'

# ... (write_reg, read_byte_data_signed, ... compensate_H までの関数はそのまま) ...
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


    try:
        with open(OUTPUT_CSV_FILE, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # ヘッダー行を定義
            writer.writerow(['timestamp', 'temperature_c', 'pressure_hpa', 'humidity_percent'])
        print(f"データは {OUTPUT_CSV_FILE} に保存されます。")
    except IOError as e:
        print(f"エラー: CSVファイル '{OUTPUT_CSV_FILE}' の準備ができませんでした: {e}")
        if bus: bus.close()
        return


    measurement_duration = 3600  # 3600秒 = 1時間 測定
    interval = 10  # 10秒間隔
    
    print(f"\nセンサーデータの測定を開始します。測定時間: {measurement_duration}秒, 測定間隔: {interval}秒")
    print("Ctrl+Cで中断できます。")
    
    start_time_script = time.time()
    try:
        while (time.time() - start_time_script) < measurement_duration:
            loop_iter_start_time = time.time()
            
            temp, pres, hum = read_compensated_data()
            
            # タイムスタンプはPCでの処理も考慮し、ISO 8601形式を推奨
            timestamp_str = datetime.now().isoformat()
            
            if temp is not None and pres is not None and hum is not None:
                # 画面への表示
                print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                      f"T:{temp:.2f}C, P:{pres:.2f}hPa, H:{hum:.2f}% ... CSVに記録しました。")

                # --- 変更: データをCSVファイルに追記 ---
                try:
                    with open(OUTPUT_CSV_FILE, 'a', newline='', encoding='utf-8') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp_str, temp, pres, hum])
                except IOError as e:
                    print(f"警告: CSVファイルへの書き込みに失敗しました: {e}")
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] データ取得に失敗しました。")
            
            # 正確なインターバルで待機
            time_to_wait = interval - (time.time() - loop_iter_start_time)
            if time_to_wait > 0:
                time.sleep(time_to_wait)
                
        print("\n予定の測定時間が完了しました。")

    except KeyboardInterrupt:
        print("\n測定がユーザーによって中断されました。")
    finally:
        if bus:
            bus.close()
            print("I2Cバスをクローズしました。")

if __name__ == '__main__':
    main()
