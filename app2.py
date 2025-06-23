# app.py
from flask import Flask, render_template, jsonify
import time
from datetime import datetime
import collections
import threading
import logging
import json

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# BME280センサー用のクラス
class BME280Sensor:
    def __init__(self):
        self.bus = None
        self.initialized = False
        self.digT = []
        self.digP = []
        self.digH = []
        self.t_fine = 0.0
        self.I2C_BUS = 1
        self.I2C_ADDR = 0x76
        self.last_error = None
        
    def init_bus(self):
        """I2Cバスを安全に初期化"""
        if self.bus is not None:
            return True
            
        try:
            from smbus2 import SMBus
            self.bus = SMBus(self.I2C_BUS)
            return True
        except ImportError:
            logger.warning("smbus2がインストールされていません。模擬モードで動作します。")
            return False
        except Exception as e:
            logger.error(f"I2Cバス初期化失敗: {e}")
            self.last_error = str(e)
            return False
    
    def write_reg(self, reg, data):
        """レジスタに書き込み"""
        if not self.init_bus():
            return False
        try:
            self.bus.write_byte_data(self.I2C_ADDR, reg, data)
            return True
        except Exception as e:
            logger.error(f"書き込み失敗 reg={hex(reg)}: {e}")
            return False
    
    def read_byte(self, reg, signed=False):
        """1バイト読み込み"""
        if not self.init_bus():
            return None
        try:
            value = self.bus.read_byte_data(self.I2C_ADDR, reg)
            if signed and value > 127:
                value -= 256
            return value
        except Exception as e:
            logger.error(f"読み込み失敗 reg={hex(reg)}: {e}")
            return None
    
    def read_word(self, reg, signed=False):
        """2バイト読み込み"""
        lsb = self.read_byte(reg)
        msb = self.read_byte(reg + 1)
        if lsb is None or msb is None:
            return None
        
        value = (msb << 8) | lsb
        if signed and value & 0x8000:
            value -= 65536
        return value
    
    def setup_sensor(self):
        """センサー設定"""
        # 湿度オーバーサンプリング x1
        if not self.write_reg(0xF2, 0x01): return False
        # 温度・気圧オーバーサンプリング x1, ノーマルモード
        if not self.write_reg(0xF4, 0x27): return False
        # スタンバイ 1000ms, フィルタOFF
        if not self.write_reg(0xF5, 0xA0): return False
        return True
    
    def read_calibration(self):
        """校正パラメータ読み込み"""
        # 温度校正
        self.digT = [
            self.read_word(0x88),          # T1
            self.read_word(0x8A, True),    # T2
            self.read_word(0x8C, True)     # T3
        ]
        if None in self.digT: return False
        
        # 気圧校正
        self.digP = [self.read_word(0x8E)]  # P1
        for i in range(1, 9):
            val = self.read_word(0x90 + (i-1)*2, True)
            if val is None: return False
            self.digP.append(val)
        
        # 湿度校正（簡略化）
        h1 = self.read_byte(0xA1)
        h2 = self.read_word(0xE1, True)
        h3 = self.read_byte(0xE3)
        if None in [h1, h2, h3]: return False
        
        self.digH = [h1, h2, h3, 0, 0, 0]  # 簡略版
        return True
    
    def read_raw_data(self):
        """生データ読み込み"""
        if not self.init_bus():
            return None, None, None
        try:
            # 8バイト一括読み込み
            data = self.bus.read_i2c_block_data(self.I2C_ADDR, 0xF7, 8)
            pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            hum_raw = (data[6] << 8) | data[7]
            return temp_raw, pres_raw, hum_raw
        except Exception as e:
            logger.error(f"生データ読み込み失敗: {e}")
            return None, None, None
    
    def compensate_temp(self, raw):
        """温度補正"""
        if not self.digT or raw is None: return None
        var1 = (raw / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        var2 = ((raw / 131072.0 - self.digT[0] / 8192.0) ** 2) * self.digT[2]
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0
    
    def compensate_pressure(self, raw):
        """気圧補正（簡略版）"""
        if not self.digP or raw is None or self.t_fine == 0: return None
        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.digP[5] / 32768.0
        var2 = var2 + var1 * self.digP[4] * 2.0
        var2 = (var2 / 4.0) + (self.digP[3] * 65536.0)
        var1 = (self.digP[2] * var1 * var1 / 524288.0 + self.digP[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.digP[0]
        if var1 == 0: return None
        pressure = 1048576.0 - raw
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
        return pressure / 100.0  # hPa
    
    def compensate_humidity(self, raw):
        """湿度補正（簡略版）"""
        if not self.digH or raw is None or self.t_fine == 0: return None
        var_H = self.t_fine - 76800.0
        if var_H == 0: return 0
        var_H = (raw - (self.digH[3] * 64.0)) * (self.digH[1] / 65536.0)
        humidity = var_H * (1.0 - self.digH[0] * var_H / 524288.0)
        return max(0.0, min(100.0, humidity))
    
    def initialize(self):
        """センサー初期化"""
        try:
            if not self.init_bus():
                return False
            
            if not self.setup_sensor():
                logger.error("センサー設定失敗")
                return False
            
            if not self.read_calibration():
                logger.error("校正パラメータ読み込み失敗")
                return False
            
            self.initialized = True
            logger.info("BME280センサー初期化完了")
            return True
            
        except Exception as e:
            logger.error(f"センサー初期化エラー: {e}")
            self.last_error = str(e)
            return False
    
    def read_data(self):
        """センサーデータ読み込み"""
        if not self.initialized:
            return None, None, None
        
        temp_raw, pres_raw, hum_raw = self.read_raw_data()
        if temp_raw is None:
            return None, None, None
        
        temperature = self.compensate_temp(temp_raw)
        pressure = self.compensate_pressure(pres_raw)
        humidity = self.compensate_humidity(hum_raw)
        
        return temperature, pressure, humidity

# グローバル変数
sensor = BME280Sensor()
data_history = collections.deque(maxlen=50)  # 最大50個のデータ
latest_data = None
data_lock = threading.Lock()
app_running = True

def data_collector():
    """バックグラウンドデータ収集"""
    global latest_data
    logger.info("データ収集開始")
    
    while app_running:
        try:
            temp, pres, hum = sensor.read_data()
            
            if temp is not None and pres is not None and hum is not None:
                data = {
                    'timestamp': datetime.now().isoformat(), # ISO 8601 形式に変更
                    'temperature': round(temp, 1),
                    'pressure': round(pres, 1),
                    'humidity': round(hum, 1)
                }
                
                with data_lock:
                    data_history.append(data)
                    latest_data = data
                
                logger.debug(f"データ更新: {temp:.1f}°C, {pres:.1f}hPa, {hum:.1f}%")
            else:
                logger.warning("センサーデータ読み込み失敗")
            
        except Exception as e:
            logger.error(f"データ収集エラー: {e}")
        
        time.sleep(5)  # 5秒間隔

# Flaskアプリ
app = Flask(__name__)

@app.route('/')
def index():
    """メインページ"""
    # templatesフォルダにあるindex.htmlをレンダリングする
    return render_template('index.html')

@app.route('/api/latest')
def api_latest():
    """最新データAPI"""
    with data_lock:
        if latest_data:
            return jsonify(latest_data)
        else:
            # smbus2がない場合のデモデータ
            return jsonify({
                'timestamp': datetime.now().isoformat(), # ISO 8601 形式に変更
                'temperature': 25.0,
                'pressure': 1013.0,
                'humidity': 50.0,
                'demo': True
            })

@app.route('/api/history')
@app.route('/data') # /data エンドポイントを追加
def api_history_and_data():
    """履歴データAPI および /data エンドポイント"""
    with data_lock:
        return jsonify(list(data_history))

@app.route('/api/status')
def api_status():
    """ステータスAPI"""
    return jsonify({
        'sensor_initialized': sensor.initialized,
        'data_count': len(data_history),
        'last_error': sensor.last_error,
        'uptime': datetime.now().isoformat() # ISO 8601 形式に変更
    })

def create_app():
    """アプリ初期化"""
    # センサー初期化
    if sensor.initialize():
        logger.info("✅ センサー初期化成功")
    else:
        logger.warning("⚠️ センサー初期化失敗 - デモモードで動作")
    
    # データ収集スレッド開始
    collector_thread = threading.Thread(target=data_collector, daemon=True)
    collector_thread.start()
    
    return app

if __name__ == '__main__':
    try:
        app = create_app()
        logger.info("🚀 Flaskアプリ開始")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        logger.info("👋 アプリ終了")
        app_running = False
    except Exception as e:
        logger.error(f"💥 起動エラー: {e}")
        app_running = False
