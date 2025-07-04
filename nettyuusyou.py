# coding: utf-8

# ---------------------------------------------------------------------------
# BME280センサー 熱中症警戒Gmailアラートシステム
#
# 機能:
# 1. 10分おきに温度と湿度を測定します。
# 2. 設定したしきい値を超えた場合、熱中症の危険を検知します。
# 3. 危険を検知した際に、Gmailで指定したアドレスにアラートメールを送信します。
# 4. メールの送りすぎを防ぐため、通知は危険状態になった最初の1回のみ送信します。
#    （一度、平常な状態に戻ると、再度通知するようになります）
#
# ---------------------------------------------------------------------------


# 必要なライブラリをインポート
import smtplib
import time
from datetime import datetime
from email.mime.text import MIMEText
from email.header import Header
from smbus2 import SMBus


# -- センサーに関する設定 --
I2C_BUS_NUMBER = 1      # ラズパイのI2Cバス番号 (通常は1)
I2C_ADDRESS = 0x76      # BME280のI2Cアドレス (0x76 または 0x77)

# -- Gmailと通知に関する設定 --
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
SENDER_EMAIL =          # 送信元にするあなたのGmailアドレス
SENDER_PASSWORD =      # Googleアカウントで取得した16桁のアプリパスワード
RECEIVER_EMAIL = # 通知を受け取りたいメールアドレス（自分宛てでOK）

# -- 熱中症アラートの条件設定 --
# 条件1: または、条件2: のいずれかを満たした場合に「危険」と判断
# 条件1: 温度がこの値を超えたら危険
TEMP_THRESHOLD_DANGER = 31.0
# 条件2: 温度がこの値を超え、かつ湿度がこの値を超えたら危険
TEMP_THRESHOLD_WARNING = 28.0
HUMI_THRESHOLD_WARNING = 75.0

# -- 監視間隔の設定 --
INTERVAL_SECONDS = 600  # 測定間隔を秒で指定 (600秒 = 10分)

# グローバル変数
bus = None
digT, digP, digH = [], [], []
t_fine = 0.0

# --- BME280センサー制御関数群 ---
# 低レベルのI2C通信や補正計算を行うための関数です。

def write_reg(reg_address, data):
    """I2Cバスに1バイト書き込む"""
    if bus is None: return False
    try:
        bus.write_byte_data(I2C_ADDRESS, reg_address, data)
        return True
    except IOError as e:
        print(f"エラー: I2C書き込み失敗: {e}")
        return False

def read_byte_data(reg_address):
    """I2Cバスから符号なし1バイト読み込む"""
    if bus is None: return None
    try:
        return bus.read_byte_data(I2C_ADDRESS, reg_address)
    except IOError as e:
        print(f"エラー: I2C読み込み失敗: {e}")
        return None

def read_word_data(reg_address, signed=False):
    """I2Cバスから2バイト読み込む"""
    lsb = read_byte_data(reg_address)
    msb = read_byte_data(reg_address + 1)
    if lsb is None or msb is None: return None
    
    value = (msb << 8) | lsb
    if signed and value & 0x8000:
        value -= 65536
    return value

def get_calib_param():
    """センサーから補正パラメータを読み出す"""
    global digT, digP, digH
    digT.append(read_word_data(0x88))
    digT.append(read_word_data(0x8A, signed=True))
    digT.append(read_word_data(0x8C, signed=True))

    digP.append(read_word_data(0x8E))
    for i in range(8):
        digP.append(read_word_data(0x90 + i * 2, signed=True))
    
    digH.append(read_byte_data(0xA1))
    digH.append(read_word_data(0xE1, signed=True))
    digH.append(read_byte_data(0xE3))
    e4 = read_byte_data(0xE4)
    e5 = read_byte_data(0xE5)
    e6 = read_byte_data(0xE6)
    digH.append((e4 << 4) | (e5 & 0x0F))
    digH.append((e6 << 4) | (e5 >> 4))
    e7 = read_byte_data(0xE7)
    if e7 > 127: e7 -= 256
    digH.append(e7)
    
    if None in digT or None in digP or None in digH:
        return False
    return True

def compensate_T(adc_T):
    """温度を生データから計算"""
    global t_fine
    v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    v2 = ((adc_T / 131072.0 - digT[0] / 8192.0)**2) * digT[2]
    t_fine = v1 + v2
    return t_fine / 5120.0

def compensate_H(adc_H):
    """湿度を生データから計算"""
    h = t_fine - 76800.0
    h = (adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * h)) * \
        (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * h * (1.0 + digH[2] / 67108864.0 * h)))
    h = h * (1.0 - digH[0] * h / 524288.0)
    return max(0.0, min(100.0, h))

def read_raw_data():
    """センサーから8バイトの生データを一括で読み込む"""
    try:
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0xF7, 8)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]
        return temp_raw, pres_raw, hum_raw
    except IOError as e:
        print(f"エラー: センサー生データ読み込み失敗: {e}")
        return None, None, None

def read_compensated_data():
    """補正計算済みの温湿度データを取得する"""
    temp_raw, _, hum_raw = read_raw_data()
    if temp_raw is None or hum_raw is None:
        return None, None
    
    temp = compensate_T(temp_raw)
    humi = compensate_H(hum_raw)
    return temp, humi

def setup_sensor():
    """センサーの動作モードを設定する"""
    if not write_reg(0xF2, 1): return False  # Humidity oversampling x1
    if not write_reg(0xF4, 0x27): return False # Temp/Press oversampling x1, Normal mode
    return True

# --- Gmail送信関数 ---
def send_alert_email(temp, humi):
    """熱中症警戒アラートのメールを送信する"""
    subject = "【熱中症アラート】危険な温湿度を検知しました！"
    body = f"""熱中症の危険性が高い環境を検知しました。
直ちにエアコンの使用や水分補給などの対策を行ってください。

---
検知時刻: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
現在の温度: {temp:.2f} ℃
現在の湿度: {humi:.2f} %
---
"""
    try:
        msg = MIMEText(body, 'plain', 'utf-8')
        msg['Subject'] = Header(subject, 'utf-8')
        msg['From'] = SENDER_EMAIL
        msg['To'] = RECEIVER_EMAIL

        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SENDER_EMAIL, SENDER_PASSWORD)
            server.send_message(msg)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] >> アラートメールの送信に成功しました。")

    except Exception as e:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] >> エラー: メール送信に失敗しました: {e}")


# --- メイン処理 ---
def main():
    """プログラムのメイン処理"""
    global bus
    try:
        bus = SMBus(I2C_BUS_NUMBER)
    except Exception as e:
        print(f"エラー: I2Cバス {I2C_BUS_NUMBER} を開けませんでした: {e}")
        return

    print("センサー初期化中...")
    if not setup_sensor() or not get_calib_param():
        print("センサーの初期化に失敗。接続を確認してください。")
        if bus: bus.close()
        return
    print("センサー初期化完了。")
    print("-" * 40)
    print(f"監視を開始します。(測定間隔: {INTERVAL_SECONDS}秒)")
    print(f"危険判断のしきい値: {TEMP_THRESHOLD_DANGER}℃ または ({TEMP_THRESHOLD_WARNING}℃ かつ {HUMI_THRESHOLD_WARNING}%)")
    print("Ctrl+Cで中断できます。")
    print("-" * 40)

    is_alert_sent = False  # メールを送信済みかどうかのフラグ

    try:
        while True:
            temp, humi = read_compensated_data()
            
            if temp is not None and humi is not None:
                timestamp = datetime.now().strftime('%H:%M:%S')
                print(f"[{timestamp}] 現在値: 温度={temp:.2f}C, 湿度={humi:.2f}%")
                
                # 熱中症の危険性を判定
                is_dangerous = (temp >= TEMP_THRESHOLD_DANGER) or \
                               (temp >= TEMP_THRESHOLD_WARNING and humi >= HUMI_THRESHOLD_WARNING)

                # 条件1: 危険な状態で、まだアラートを送っていなければ送信する
                if is_dangerous and not is_alert_sent:
                    print(f"[{timestamp}] !! 危険な状態を検知しました。アラートを送信します。")
                    send_alert_email(temp, humi)
                    is_alert_sent = True  # 送信済みフラグを立て、連続送信を防ぐ
                
                # 条件2: 危険でない状態で、以前にアラートを送ったことがある場合
                elif not is_dangerous and is_alert_sent:
                    print(f"[{timestamp}] -- 平常な状態に戻りました。監視を継続します。")
                    is_alert_sent = False # フラグをリセットし、再度通知できるようにする

            else:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] データ取得に失敗しました。5秒後に再試行します。")
                time.sleep(5) # 失敗した場合は少し待ってからリトライ
                continue

            # 設定した間隔で待機
            time.sleep(INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\nプログラムがユーザーによって中断されました。")
    finally:
        if bus:
            bus.close()
            print("I2Cバスをクローズしました。")


if __name__ == '__main__':
    main()
