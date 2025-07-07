import os
from flask import Flask, request, abort
from linebot import LineBotApi, WebhookHandler
from linebot.exceptions import InvalidSignatureError
from linebot.models import MessageEvent, TextMessage

app = Flask(__name__)

# ★★★ ここにあなたのLINE Botのチャネルアクセストークンとチャネルシークレットを設定してください ★★★
# 環境変数から読み込むのが推奨ですが、一時的なデバッグなので直接記述してもOKです。
# ただし、このコードをインターネット上に公開する際には絶対に直接記述しないでください。
CHANNEL_ACCESS_TOKEN = os.getenv('LINE_CHANNEL_ACCESS_TOKEN', 'YOUR_CHANNEL_ACCESS_TOKEN_HERE')
CHANNEL_SECRET = os.getenv('LINE_CHANNEL_SECRET', 'YOUR_CHANNEL_SECRET_HERE')

line_bot_api = LineBotApi(CHANNEL_ACCESS_TOKEN)
handler = WebhookHandler(CHANNEL_SECRET)

@app.route("/callback", methods=['POST'])
def callback():
    signature = request.headers['X-Line-Signature']
    body = request.get_data(as_text=True)
    
    print("\n--- LINE Webhook Received ---")
    print(f"Request body: {body}") # 受信したJSONデータ全体を表示（デバッグ用）

    try:
        handler.handle(body, signature)
    except InvalidSignatureError:
        print("Invalid signature. Check your Channel Secret in LINE Developers.")
        abort(400)
    except Exception as e:
        print(f"An error occurred during webhook handling: {e}")
        abort(500)
    return 'OK'

@handler.add(MessageEvent, message=TextMessage)
def handle_message(event):
    # ここであなたのユーザーIDが出力されます！
    user_id = event.source.user_id
    print("\n***************************************************")
    print("LINEユーザーIDを取得しました！")
    print(f"このIDをあなたの matome.py の LINE_USER_ID_TO_SEND に設定してください。")
    print(f"あなたのLINEユーザーID: {user_id}")
    print("***************************************************")
    
    # オプション: BotがあなたのIDを返信するようにしたい場合 (注意: これを有効にすると返信メッセージが送信されます)
    # line_bot_api.reply_message(
    #     event.reply_token,
    #     TextMessage(text=f"あなたのユーザーIDは: {user_id} です")
    # )

if __name__ == "__main__":
    # Flaskアプリを実行 (デフォルトでポート5000)
    print("Flaskアプリを起動中... http://0.0.0.0:5000")
    print("ngrok などでこのURLをインターネットに公開してください。")
    app.run(host='0.0.0.0', port=5000)
