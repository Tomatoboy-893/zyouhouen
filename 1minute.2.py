#coding: utf-8

from smbus2 import SMBus
import time

# --- 設定値 
# Raspberry PiのI2Cバス番号 (通常は1)
I2C_BUS_NUMBER  = 1
# BME280センサーのI2Cアドレス (通常は0x76。0x77の場合もあります)
I2C_ADDRESS = 0x76
# --- 設定値ここまで ---

# グローバル変数
bus = None # I2Cバスのインスタンス (初期化は後で行う)
digT = []
digP = []
digH = []
t_fine = 0.0

# --- 低レベルI2C通信関数 (エラーハンドリング付き) ---
def write_reg(reg_address, data):
    """センサーのレジスタに1バイト書き込む (エラーハンドリング付き)"""
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
    """センサーのレジスタから1バイト読み込む (エラーハンドリング付き)"""
    if bus is None:
        print("エラー: I2Cバスが初期化されていません。")
        return None
    try:
        value = bus.read_byte_data(I2C_ADDRESS, reg_address)
        if signed: # 符号付きの場合の処理 (必要に応じて使用)
            if value > 127:
                value -= 256
        return value
    except IOError as e:
        print(f"エラー: I2C読み込み失敗 (アドレス {hex(I2C_ADDRESS)}, レジスタ {hex(reg_address)}): {e}")
        return None

def read_word_data_signed(reg_address, lsb_first=True, signed=False):
    """センサーのレジスタから2バイト(ワード)読み込む (エラーハンドリング付き)"""
    lsb = read_byte_data_signed(reg_address)
    msb = read_byte_data_signed(reg_address + 1)

    if lsb is None or msb is None:
        return None

    if lsb_first:
        value = (msb << 8) | lsb
    else:
        value = (lsb << 8) | msb
    
    if signed:
        if value & 0x8000: # 最上位ビットが1なら負数
            value = (-value ^ 0xFFFF) + 1 # 2の補数表現を考慮した符号反転
    return value


# --- BME280 センサー制御関数 ---
def get_calib_param():
    """センサーから補正パラメータを読み出し、グローバル変数に格納する。"""
    global digT, digP, digH
    digT, digP, digH = [], [], [] # 初期化

    # 温度補正パラメータ
    digT.append(read_word_data_signed(0x88)) # dig_T1 (unsigned short)
    digT.append(read_word_data_signed(0x8A, signed=True)) # dig_T2 (signed short)
    digT.append(read_word_data_signed(0x8C, signed=True)) # dig_T3 (signed short)
    if None in digT: return False # 読み込み失敗

    # 気圧補正パラメータ
    digP.append(read_word_data_signed(0x8E)) # dig_P1 (unsigned short)
    for i in range(8): # P2からP9まで
        param = read_word_data_signed(0x90 + i * 2, signed=True) # P2-P9 (signed short)
        if param is None: return False
        digP.append(param)
    if None in digP: return False

    # 湿度補正パラメータ
    val = read_byte_data_signed(0xA1) # dig_H1 (unsigned char)
    if val is None: return False
    digH.append(val)

    val = read_word_data_signed(0xE1, signed=True) # dig_H2 (signed short)
    if val is None: return False
    digH.append(val)

    val = read_byte_data_signed(0xE3) # dig_H3 (unsigned char)
    if val is None: return False
    digH.append(val)
    
    e4 = read_byte_data_signed(0xE4) # dig_H4 bits 11:4
    e5_lsb = read_byte_data_signed(0xE5) # dig_H4 bits 3:0, dig_H5 bits 7:4
    e6 = read_byte_data_signed(0xE6) # dig_H5 bits 3:0
    if e4 is None or e5_lsb is None or e6 is None: return False
    digH.append((e4 << 4) | (e5_lsb & 0x0F))
    digH.append((e6 << 4) | (e5_lsb >> 4 & 0x0F)) # データシートによって表現が異なる場合あり、要確認

    val = read_byte_data_signed(0xE7, signed=True) # dig_H6 (signed char)
    if val is None: return False
    digH.append(val)
    
    # print(f"Debug: digT={digT}, digP={digP}, digH={digH}") # デバッグ用
    return True


def compensate_T(adc_T):
    """生の温度データ(adc_T)を摂氏温度に補正する。"""
    global t_fine
    if not digT: return None # 補正パラメータがない

    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = ((adc_T / 131072.0 - digT[0] / 8192.0) ** 2) * digT[2] # **2 で二乗
    t_fine = v1 + v2
    temperature = t_fine / 5120.0
    return temperature

def compensate_P(adc_P):
    """生の気圧データ(adc_P)をhPaに補正する。"""
    global t_fine
    if not digP or t_fine == 0.0: return None # 補正パラメータがないか、t_fineが未計算

    pressure = 0.0
    v1 = (t_fine / 2.0) - 64000.0
    v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048.0) * digP[5] # digPのインデックスに注意
    v2 = v2 + ((v1 * digP[4]) * 2.0)
    v2 = (v2 / 4.0) + (digP[3] * 65536.0)
    
    # digPのインデックスが0から始まるため、データシートのP1, P2...と対応させる
    # digP[0] = P1, digP[1] = P2, ...
    calc_v1_num = (digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192.0)) / 8.0
    calc_v1_den = ((digP[1] * v1) / 2.0)
    v1_combined = (calc_v1_num + calc_v1_den) / 262144.0 # BME280データシートでは v1 = ... で再代入
    
    v1 = ((32768.0 + v1_combined) * digP[0]) / 32768.0 # digP[0] = _p[0] (dig_P1)
    
    if v1 == 0:
        return None # ゼロ除算を回避

    pressure = ((1048576.0 - adc_P) - (v2 / 4096.0)) * 3125.0
    if pressure < 0x80000000:
        pressure = (pressure * 2.0) / v1
    else:
        pressure = (pressure / v1) * 2.0 # pressureは既に符号情報を含んでいるはず
    
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096.0
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0
    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)
    return pressure / 100.0 # PaからhPaに変換

def compensate_H(adc_H):
    """生の湿度データ(adc_H)を%RHに補正する。"""
    global t_fine
    if not digH or t_fine == 0.0: return None

    var_H = t_fine - 76800.0
    if var_H == 0: # ゼロ除算の可能性を回避 (厳密には浮動小数点比較に注意)
      
        pass

   
    
    humidity = adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_H)
    humidity = humidity * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_H * \
                       (1.0 + digH[2] / 67108864.0 * var_H)))
    humidity = humidity * (1.0 - digH[0] * humidity / 524288.0)

    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0
    return humidity


def read_raw_data():
    """センサーから温度・気圧・湿度の生データを読み出す。"""
    # burst readで0xF7から0xFEまでの8バイトを読み出す
    block = []
    for i in range(8): # 0xF7 から 0xFE まで
        byte_val = read_byte_data_signed(0xF7 + i)
        if byte_val is None: return None, None, None # 読み出し失敗
        block.append(byte_val)

    pres_raw = (block[0] << 12) | (block[1] << 4) | (block[2] >> 4)
    temp_raw = (block[3] << 12) | (block[4] << 4) | (block[5] >> 4)
    hum_raw  = (block[6] << 8)  |  block[7]
    
    return temp_raw, pres_raw, hum_raw

def read_compensated_data():
    """補正済みの温湿度・気圧データを取得する。"""
    temp_raw, pres_raw, hum_raw = read_raw_data()

    if temp_raw is None or pres_raw is None or hum_raw is None:
        return None, None, None # 生データ読み取り失敗

    temperature = compensate_T(temp_raw)
    # 温度が正常に取得できた場合のみ、気圧と湿度を計算
    if temperature is None:
        return None, None, None
        
    pressure = compensate_P(pres_raw)
    humidity = compensate_H(hum_raw)
    
    return temperature, pressure, humidity


def setup_sensor():
    """センサーの動作モードを設定する。"""
    # オーバーサンプリング設定など (データシート参照)
    osrs_t = 1  # Temperature oversampling x1
    osrs_p = 1  # Pressure oversampling x1
    osrs_h = 1  # Humidity oversampling x1
    mode   = 3  # Normal mode
    t_sb   = 5  # Tstandby 1000ms
    filter_coeff = 0  # Filter off
    spi3w_en = 0  # 3-wire SPI Disable

    # レジスタ設定値の作成
    ctrl_hum_reg  = osrs_h & 0x07 # 湿度オーバーサンプリング (0xF2)
    ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode # 温度・気圧OS、モード (0xF4)
    config_reg    = (t_sb << 5) | (filter_coeff << 2) | spi3w_en # Tstandby, Filter (0xF5)

    if not write_reg(0xF2, ctrl_hum_reg): return False
    if not write_reg(0xF4, ctrl_meas_reg): return False
    if not write_reg(0xF5, config_reg): return False
    return True

# --- メイン処理 ---
def main():
    global bus
    try:
        bus = SMBus(I2C_BUS_NUMBER)
    except Exception as e:
        print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")
        print("I2Cが有効になっているか、バス番号が正しいか確認してください。")
        return

    print("BME280センサーの初期化を開始します...")
    if not setup_sensor():
        print("センサーの動作モード設定に失敗しました。")
        bus.close()
        return
    print("センサーの動作モード設定完了。")

    if not get_calib_param():
        print("補正パラメータの読み出しに失敗しました。センサー接続を確認してください。")
        bus.close()
        return
    print("補正パラメータ読み出し完了。")

    all_readings = []
    measurement_duration = 60  # 測定時間（秒）
    interval = 5               # 測定間隔（秒）
    
    start_time_script = time.time()
    
    print(f"\nセンサーデータの測定を開始します。測定時間: {measurement_duration}秒, 測定間隔: {interval}秒")
    print("Ctrl+Cで中断できます。")

    try:
        loop_count = 0
        while (time.time() - start_time_script) < measurement_duration:
            loop_iter_start_time = time.time()
            loop_count += 1
            print(f"\n--- 測定 #{loop_count} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---")

            temp, pres, hum = read_compensated_data()
            current_timestamp_epoch = time.time() # データ取得試行時刻

            if temp is not None and pres is not None and hum is not None:
                print(f"  温度: {temp:.2f} ℃")
                print(f"  気圧: {pres:.2f} hPa")
                print(f"  湿度: {hum:.2f} %")
                all_readings.append({
                    "timestamp_epoch": current_timestamp_epoch,
                    "temperature_celsius": temp,
                    "pressure_hpa": pres,
                    "humidity_percent": hum
                })
            else:
                print("  データの取得に失敗しました。")
                all_readings.append({ # エラー発生時も記録は残す（値はNone）
                    "timestamp_epoch": current_timestamp_epoch,
                    "temperature_celsius": None,
                    "pressure_hpa": None,
                    "humidity_percent": None
                })
            
            time_to_wait = interval - (time.time() - loop_iter_start_time)
            if time_to_wait > 0:
                time.sleep(time_to_wait)
        
        print("\n測定が完了しました。")

    except KeyboardInterrupt:
        print("\n測定がユーザーによって中断されました。")
    finally:
        if bus:
            bus.close() # I2Cバスをクローズ
            print("I2Cバスをクローズしました。")

    if all_readings:
        print("\n--- 最終測定結果一覧 ---")
        for i, reading in enumerate(all_readings):
            timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(reading["timestamp_epoch"]))
            temp_str = f"{reading['temperature_celsius']:.2f} ℃" if reading['temperature_celsius'] is not None else "N/A"
            pres_str = f"{reading['pressure_hpa']:.2f} hPa" if reading['pressure_hpa'] is not None else "N/A"
            hum_str  = f"{reading['humidity_percent']:.2f} %" if reading['humidity_percent'] is not None else "N/A"
            
            print(f"#{i+1} 時刻: {timestamp_str}, 温度: {temp_str}, 気圧: {pres_str}, 湿度: {hum_str}")
    else:
        print("データは収集されませんでした。")

if __name__ == '__main__':
    main()
