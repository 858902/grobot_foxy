import threading
import logging
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import pandas as pd
import ast
from flask_cors import CORS
import socketio

# 로깅 설정
logging.basicConfig(level=logging.INFO)

# 전역 변수로 데이터 저장
received_texts = []

app = Flask(__name__)
CORS(app)
socketio_app = SocketIO(app, cors_allowed_origins='*')

file_path = "/home/nuc1/Downloads/Data.csv"
df = pd.read_csv(file_path)
sample_df = pd.DataFrame()
basket_df = pd.DataFrame()
destination_list = []

keyword = []
ID = []

@app.route('/')
def new_page():
    return render_template('Home.html')

@app.route('/home')
def home():
    global df
    global keyword
    global ID
    global basket_df
    global sample_df
    global joined_keyword

    # 수신된 키워드와 ID를 사용합니다.
    if not keyword or not ID:
        sample_df = df.sample(10).reset_index(drop=True)
    
    capitalized_keywords = [word.capitalize() for word in keyword]
    joined_keyword = '&'.join(capitalized_keywords)
    logging.info(f"Joined keyword: {joined_keyword}")

    return render_template('capstone.html', products=sample_df.to_dict('records'), basket_items=basket_df.to_dict('records'), keyword=joined_keyword)

@app.route('/product/<int:product_id>')
def product_page(product_id):
    global df
    global basket_df
    global joined_keyword
    
    product_info = df[df['ID'] == product_id].iloc[0].to_dict()
    recommend_df = pd.DataFrame()
    ID_list = df[df['ID'] == product_id]['recommend_list'].iloc[0]
    ID_list = ast.literal_eval(ID_list)

    for i in ID_list:
        if i != product_id:
            recommend_df = pd.concat([recommend_df, df[df['ID'] == i]], ignore_index=True)

    logging.info(f"Product info: {product_info}")
    logging.info(f"Recommendations: {recommend_df}")

    return render_template('productpage.html', product=product_info, recommends=recommend_df.to_dict('records'), keyword=joined_keyword)

@app.route('/add-to-basket', methods=['POST'])
def add_to_basket():
    global df
    global basket_df
    global destination_list
    data = request.get_json()
    product_id = data['product_id']
    product_info = df[df['ID'] == int(product_id)]
    basket_df = pd.concat([basket_df, product_info], ignore_index=True)
    print(basket_df)
    destination_list = basket_df['map_location'].unique()
    conversion_dict = {'A': 1, 'B': 1, 'C': 1, 'D': 1, 'E': 2, 'F': 2, 'G': 2, 'H': 2, 'I': 2, 'J': 2}

    # 리스트의 각 요소를 숫자로 변환하고, 그 결과를 문자열로 변환하여 리스트에 저장
    converted_list = [str(conversion_dict[item]) for item in destination_list]

    # 변환된 리스트의 요소를 띄어쓰기로 구분하여 하나의 문자열로 합치기
    destination_list = ' '.join(converted_list)
    logging.info(f"Destination list: {destination_list}")
    return jsonify({'message': '장바구니에 상품이 추가되었습니다!'})

@app.route('/path/to/delete/basket_item', methods=['POST'])
def delete_basket_item():
    data = request.get_json()
    product_id = data.get('product_id')
    try:
        product_id_int = int(product_id)
    except ValueError:
        return jsonify({'message': '유효하지 않은 상품 ID입니다.'}), 400
    global basket_df
    basket_df = basket_df[basket_df['ID'] != product_id_int]
    return jsonify({'message': '상품이 성공적으로 삭제되었습니다.'}), 200

@app.route('/get-destinations', methods=['GET'])
def get_destinations():
    global destination_list
    logging.info(f"Destination list: {destination_list}")
    return jsonify({'destinations': str(destination_list)})

@app.route('/get-location/<int:product_id>', methods=['GET'])
def get_location(product_id):
    product_info = df.loc[df['ID'] == product_id, 'map_location'].values
    if product_info.size > 0:
        return jsonify({'map_location': product_info[0]})
    else:
        return jsonify({'error': 'Product not found'}), 404

# 키워드와 ID를 수신하는 엔드포인트
@app.route('/keyword_ids', methods=['POST'])
def keyword_ids():
    global sample_df
    global keyword
    global ID
    data = request.get_json()
    keyword = data.get('keywords', [])
    ID = data.get('ids', [])
    logging.info(f"Received keywords: {keyword}")
    logging.info(f"Received ids: {ID}")

    # 수신한 키워드와 ID를 사용하여 sample_df 업데이트
    sample_df = pd.DataFrame()
    for i in ID:
        sample_df = pd.concat([sample_df, df[df['ID'] == i]], ignore_index=True)
    
    logging.info(f"Updated sample_df: {sample_df}")

    socketio_app.emit('keyword_ids', {'keywords': keyword, 'ids': ID})  # 수신한 키워드와 ID를 클라이언트로 전송
    return jsonify({'status': 'success'})

# SocketIO 이벤트 처리
@socketio_app.on('keyword_ids')
def handle_keyword_ids(data):
    keywords = data.get('keywords')
    ids = data.get('ids')
    logging.info(f"Received keywords via SocketIO: {keywords}")
    logging.info(f"Received ids via SocketIO: {ids}")
    emit('keyword_ids', {'keywords': keywords, 'ids': ids}, broadcast=True)

@socketio_app.on('connect')
def handle_connect():
    logging.info("Client connected")

@socketio_app.on('disconnect')
def handle_disconnect():
    logging.info("Client disconnected")

@socketio_app.on('input_text')
def handle_input_text(data):
    logging.info(f"Received input text: {data['text']}")

@socketio_app.on('response_text_ko')
def handle_response_text(data):
    logging.info(f"Received response text: {data['text']}")
    # 데이터를 전역 변수에 추가
    received_texts.append(data['text'])
    # 클라이언트에게 확인 응답 보내기
    emit('response_text_ko', {'text': data['text']})

@app.route('/get-latest-text')
def get_latest_text():
    if received_texts:
        # 가장 최근의 텍스트 반환
        return jsonify({'text': received_texts[-1]})
    else:
        return jsonify({'text': ''})
    

# SocketIO 클라이언트 설정
sio = socketio.Client()

@sio.event
def connect():
    logging.info("Connected to the UI server.")

@sio.event
def connect_error(data):
    logging.error(f"Failed to connect to the UI server: {data}")

@sio.event
def disconnect():
    logging.info("Disconnected from the UI server.")

@sio.event
def keyword_ids(data):
    global sample_df
    global keyword
    global ID
    logging.info(f"Received keyword and ID data: {data}")
    keyword = data.get('keywords', [])
    ID = data.get('ids', [])

    # 수신한 키워드와 ID를 사용하여 sample_df 업데이트
    sample_df = pd.DataFrame()
    for i in ID:
        sample_df = pd.concat([sample_df, df[df['ID'] == i]], ignore_index=True)
    
    logging.info(f"Updated sample_df: {sample_df}")

def run_socketio_client():
    sio.connect('http://1.224.223.74:5002')
    sio.wait()

if __name__ == '__main__':
    # Flask 서버와 SocketIO 클라이언트 모두 실행
    client_thread = threading.Thread(target=run_socketio_client)
    client_thread.start()

    socketio_app.run(app, debug=True, host='0.0.0.0', port=5000)
    client_thread.join()
