from flask import Flask, render_template
from flask import Flask, request, jsonify
from flask_cors import CORS
import pandas as pd
import ast

app = Flask(__name__)
CORS(app)

file_path = "/home/nuc1/Downloads/Data.csv"
df = pd.read_csv(file_path)

basket_df = pd.DataFrame()
destination_list = []

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

@app.route('/product/<int:product_id>')
def product_page(product_id):
    global df
    global basket_df
    
    product_info = df[df['ID'] == product_id].iloc[0].to_dict()
    recommend_df = pd.DataFrame()  
    ID_list = df[df['ID'] == product_id]['recommend_list'].iloc[0]
    ID_list = ast.literal_eval(ID_list)

    for i in ID_list:
        if i != product_id:
            # pandas.concat을 사용하여 DataFrame 연결
            recommend_df = pd.concat([recommend_df, df[df['ID'] == i]], ignore_index=True)

    print(recommend_df)

    # 찾은 상품 정보를 productpage.html로 전달합니다.
    return render_template('productpage.html', product=product_info ,recommends=recommend_df.to_dict('records'))# 여기서 'productpage.html'은 templates 폴더 안에 있어야 합니다.
    

@app.route('/add-to-basket', methods=['POST'])
def add_to_basket():
    global df
    global basket_df
    global destination_list
    data = request.get_json()
    product_id = data['product_id']

    # 상품 ID를 사용하여 원래 df에서 상품 정보를 찾음
    product_info = df[df['ID'] == int(product_id)]  # 가정: ID가 유니크하다

    # basket_df에 상품 정보 추가
    basket_df = pd.concat([basket_df, product_info], ignore_index=True)
    print(basket_df)
    destination_list = basket_df['map_location'].unique()
    print(destination_list)
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

@app.route('/get-destinations', methods=['GET'])
def get_destinations():
    global destination_list
    print(str(destination_list))
    return jsonify({'destinations': str(destination_list)})

@app.route('/get-location/<int:product_id>', methods=['GET'])
def get_location(product_id):
    # 제품 ID를 기반으로 map_location 정보 검색
    product_info = df.loc[df['ID'] == product_id, 'map_location'].values
    if product_info.size > 0:
        return jsonify({'map_location': product_info[0]})
    else:
        return jsonify({'error': 'Product not found'}), 404



if __name__ == '__main__':
    app.run(debug=True)
