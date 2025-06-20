# app.py
from flask import Flask, render_template, jsonify
from smbus2 import SMBus
import time
from datetime import datetime
import collections # データ保持のためのdequeをインポート

# --- BME280センサー関連の定数と関数はそのまま ---
I2C_BUS_NUMBER = 1
I2C_ADDRESS = 0x76

bus = None
digT = []
digP = []
digH = []
t_fine = 0.0

def write_reg(reg_address, data):
    global bus
    if bus is None:
        try:
            bus = SMBus(I2C_BUS_NUMBER)
        except Exception as e:
            print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")
            return False
    try:
        bus.write_byte_data(I2C_ADDRESS, reg_address, data)
        return True
    except IOError as e:
        print(f"エラー: I2C書き込み失敗 (アドレス {hex(I2C_ADDRESS)}, レジスタ {hex(reg_address)}): {e}")
        return False

def read_byte_data_signed(reg_address, signed=False):
    global bus
    if bus is None:
        try:
            bus = SMBus(I2C_BUS_NUMBER)
        except Exception as e:
            print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")
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
    global bus
    if bus is None:
        try:
            bus = SMBus(I2C_BUS_NUMBER)
        except Exception as e:
            print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")
            return None, None, None
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

# --- Flaskアプリケーションの開始 ---
app = Flask(__name__)

# 最新のセンサーデータを保持するキュー (例: 過去60個のデータを保持)
# collections.dequeは高速な両端キューを提供し、最大数を指定すると自動的に古い要素を削除します。
sensor_data_history = collections.deque(maxlen=60) # 10秒間隔で10分間のデータ

# センサー初期化をアプリケーション起動時に一度だけ行う
@app.before_first_request
def initialize_sensor():
    global bus
    try:
        bus = SMBus(I2C_BUS_NUMBER)
        print("BME280センサーの初期化を開始します...")
        if not setup_sensor():
            print("センサーの動作モード設定に失敗しました。")
        else:
            print("センサーの動作モード設定完了。")

        if not get_calib_param():
            print("補正パラメータの読み出しに失敗しました。センサー接続を確認してください。")
        else:
            print("補正パラメータ読み出し完了。")
    except Exception as e:
        print(f"エラー: I2Cバスのオープンに失敗しました (バス番号 {I2C_BUS_NUMBER}): {e}")

    # 定期的にデータを読み取るためのスレッドを起動することも可能ですが、
    # シンプルにするため、ここではAPIリクエスト時に読み取るか、別プロセスでデータを更新する想定です。
    # ここでは、/dataエンドポイントへのアクセス時にデータを取得する簡易な方法を採用します。

# メインページ
@app.route('/')
def index():
    return render_template('index.html')

# センサーデータをJSON形式で提供するAPIエンドポイント
@app.route('/data')
def get_data():
    temp, pres, hum = read_compensated_data()
    timestamp_str = datetime.now().isoformat()

    if temp is not None and pres is not None and hum is not None:
        data = {
            'timestamp': timestamp_str,
            'temperature': round(temp, 2),
            'pressure': round(2, 2), # 精度を丸める
            'humidity': round(hum, 2)
        }
        # 最新データを履歴に追加
        sensor_data_history.append(data)
        # 全ての履歴データを返す（グラフ描画用）
        return jsonify(list(sensor_data_history))
    else:
        # データ取得失敗時は、これまでの履歴データがあればそれを返す
        if sensor_data_history:
            return jsonify(list(sensor_data_history))
        else:
            return jsonify({"error": "Failed to read sensor data"}), 500

if __name__ == '__main__':
    # Flaskアプリケーションを実行
    # host='0.0.0.0'で外部からのアクセスを許可
    # debug=Trueは開発用。本番環境ではFalseにするか、Gunicornなどを使う
    app.run(host='0.0.0.0', port=5000, debug=True)
