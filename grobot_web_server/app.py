from flask import Flask, render_template
from flask import Flask, request, jsonify
import pandas as pd

app = Flask(__name__)


df = pd.read_csv(r"~/Downloads/Tafas_data1.csv")

basket_df = pd.DataFrame()

@app.route('/home')  # 이전에 '/'였던 부분을 '/home'으로 변경
def home():
    global df
    global basket_df
    # 랜덤하게 10개의 상품을 선택
    sample_df = df.sample(10).reset_index(drop=True)

    return render_template('capstone.html', products=sample_df.to_dict('records'), basket_items=basket_df.to_dict('records'))

@app.route('/')  # 이제 new_page 함수가 시작 페이지가 됩니다.
def new_page():
    # 필요한 데이터 처리
    # 예시에서는 데이터 처리 없이 바로 HTML 파일을 렌더링합니다.
    return render_template('Home.html')

@app.route('/add-to-basket', methods=['POST'])
def add_to_basket():
    global df
    global basket_df
    data = request.get_json()
    product_id = data['product_id']

    # 상품 ID를 사용하여 원래 df에서 상품 정보를 찾음
    product_info = df[df['ID'] == int(product_id)]  # 가정: ID가 유니크하다

    # basket_df에 상품 정보 추가
    basket_df = basket_df.append(product_info, ignore_index=True)
    print(basket_df)

    return jsonify({'message': '장바구니에 상품이 추가되었습니다!'})

@app.route('/path/to/delete/basket_item', methods=['POST'])
def delete_basket_item():
    data = request.get_json()
    product_id = data.get('product_id')

    # product_id가 유효한지 확인
    try:
        product_id_int = int(product_id)  # 문자열을 정수로 변환 시도
    except ValueError:
        # 변환 실패 시, 클라이언트에게 오류 메시지를 반환
        return jsonify({'message': '유효하지 않은 상품 ID입니다.'}), 400

    global basket_df
    # 상품 ID에 해당하는 아이템을 basket_df에서 제거
    basket_df = basket_df[basket_df['ID'] != product_id_int]
    
    return jsonify({'message': '상품이 성공적으로 삭제되었습니다.'}), 200


if __name__ == '__main__':
    app.run(debug=True)
