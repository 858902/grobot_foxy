// 모달 요소를 얻습니다.
var modal = document.getElementById("myModal");

var modalContent = document.querySelector('.modal-content');

var mapLocation ;

// 버튼 객체를 얻습니다.
var btn = document.getElementById("routeButton");
var autoBtn = document.getElementById("autoButton"); // auto 버튼 객체 추가
var calcBtn = document.getElementById('full-size-button'); // 계산 하러 가기 버튼
//var locBtn = document.getElementById('locationButton'); // 위치 안내 받기 버튼
var locationButtons = document.querySelectorAll('.locationButton');

// 모달 닫기 <span> 요소를 얻습니다. (별도로 <span>으로 닫는 버튼이 있다면)
var span = document.getElementsByClassName("close")[0];

// '예' 버튼 클릭 시 수행할 동작을 저장할 변수
var yesAction = function() {};
// '아니오' 버튼 클릭 시 수행할 동작을 저장할 변수
var noAction = function() {};

window.onload = function() {
    loadStatus();
};

function loadStatus() {
    // Local Storage에서 'currentStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'ON'을 설정합니다.
    var currentStatus = localStorage.getItem('currentStatus') || 'ON';
    
    // 가져온 값에 따라 화면에 표시될 텍스트를 설정합니다.
    document.querySelector('.part3_bottom').textContent = currentStatus;
}

var waypointPub = new ROSLIB.Topic({
    ros: ros,
    name: '/waypoint_list_raw',
    messageType: 'std_msgs/String'
});

// 사용자가 버튼 클릭 시 모달을 표시합니다.
btn.onclick = function() {
    document.getElementById("modalText").innerHTML = "최적 경로로 안내 받으시겠습니까?";
    modal.style.display = "block";
    yesAction = function() {
        console.log("최적 경로 안내 시작");
        modal.style.display = "none";
        fetch('/get-destinations')
        .then(response => response.json())
        .then(data => {
            console.log(data.destinations); // 콘솔에 destination_list 데이터 출력
            var destinationsString = data.destinations;

            var waypointlist = new ROSLIB.Message({
                data: destinationsString // JSON.stringify를 사용하지 않고 직접 변환된 문자열을 사용
            });
            // var waypointlist = new ROSLIB.Message({
            //     data: JSON.stringify(data.destinations)
            // });
            waypointPub.publish(waypointlist); // waypoint list 전달
            navStart.publish(goal_type1);  // ok 신호 

        })
        .catch(error => console.error('Error:', error));
        };
    noAction = function() {
        console.log("최적 경로 안내 취소");
        modal.style.display = "none";
    };
}

function loadStatus() {
    // Local Storage에서 'currentStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'ON'을 설정합니다.
    var currentStatus = localStorage.getItem('currentStatus') || 'ON';
    
    // 가져온 값에 따라 화면에 표시될 텍스트를 설정합니다.
    document.querySelector('.part3_bottom').textContent = currentStatus;
}

// auto 버튼 클릭 시 모달을 표시합니다.
autoBtn.onclick = function() {
    var currentStatus = document.querySelector('.part3_bottom').textContent; // 해당 클래스를 가진 요소가 있다고 가정합니다.
    if (currentStatus === "ON") {
        document.getElementById("modalText").innerHTML = "자율 주행 모드를 멈추시고, 수동 운전 모드로 전환하시겠습니까?";
        yesAction = function() {
            signalOffset.publish(msg_offset2); // offset 측정 끝
            console.log('Manual mode activated');
            modeSwitch.publish(mode_type1); // 수동 모드 on
            document.querySelector('.part3_bottom').textContent = 'OFF';
            localStorage.setItem('currentStatus', document.querySelector('.part3_bottom').textContent);
            modal.style.display = "none";
        };
    } else {
        document.getElementById("modalText").innerHTML = "수동 운전 모드를 멈추시고, 자율 주행 모드로 전환하시겠습니까?";
        yesAction = function() {
            modeSwitch.publish(mode_type2); // navigation
            console.log('Auto mode activated');
            document.querySelector('.part3_bottom').textContent = 'ON';
            localStorage.setItem('currentStatus', document.querySelector('.part3_bottom').textContent);
            modal.style.display = "none";
        };
    }
    noAction = function() {
        console.log("모드 변경 취소");
        modal.style.display = "none";
    };
    modal.style.display = "block";
}
calcBtn.onclick = function() {
    document.getElementById("modalText").innerHTML = "계산대로 이동하시겠습니까?";
    modal.style.display = "block";
    yesAction = function() {
        // 승민아 여기 비상탈출 예
        console.log("계산대로 이동");
        modal.style.display = "none";
    };
    noAction = function() {
        // 승민아 여기 비상탈출 아니오
        console.log("이동 취소");
        modal.style.display = "none";
    };
}

// '위치 안내 받기' 버튼 클릭 시 모달을 표시합니다.
locationButtons.forEach(function(button) {
    button.addEventListener('click', function() {
        changeModal()
        
        modal.style.display = "block";
        // 예를 들어, 상단과 하단의 여백을 60px, 좌우의 여백을 30px로 설정하고 싶은 경우
        


        yesAction = function() {
            console.log("상품 위치로 이동, 상품 ID: " + button.getAttribute('data-product-id'));
            modal.style.display = "none";
            resetModalSize(); // 모달 크기를 초기화
        };
        noAction = function() {
            console.log("이동 취소");
            modal.style.display = "none";
            resetModalSize(); // 모달 크기를 초기화
        };
    });
});
// '예' 버튼 클릭 시 설정된 동작을 실행합니다.
document.querySelector('.yes-button').addEventListener('click', function() {
    yesAction();
});

// '아니오' 버튼 클릭 시 설정된 동작을 실행합니다.
document.querySelector('.no-button').addEventListener('click', function() {
    noAction();
});

// 사용자가 모달 외부 클릭 시, 모달을 닫습니다.
window.onclick = function(event) {
    if (event.target == modal) {
        modal.style.display = "none";
    
    }
}

// 각 버튼에 대해 이벤트 리스너를 추가
locationButtons.forEach(function(button) {
    button.addEventListener('click', function() {
    
        var productId = this.getAttribute('data-product-id');
        console.log(productId); // 현재 클릭된 버튼의 product ID를 콘솔에 출력
    });
});

    


function changeModal() {
    
    modalContent.innerHTML = ""; // 모달 컨텐트의 내용을 먼저 비웁니다.
    modalContent.style.width = "60%"; // 모달의 너비를 280px로 설정합니다.
    modalContent.style.height = "60%"; // 모달의 높이를 210px로 설정합니다.
    modalContent.style.display = "flex"; // flex 디스플레이 설정
    modalContent.style.flexDirection = "row"; // 컨텐츠를 가로로 배열
    modalContent.style.justifyContent = "center"; // 컨텐츠를 왼쪽으로 정렬
    modalContent.style.alignItems = "center"; // 컨텐츠를 센터 정렬
    modalContent.style.overflow = "hidden";

    // 이미지를 위한 div를 생성하고 스타일을 적용합니다.
    var modalImage = document.createElement("div");
    modalImage.style.flex = "7"; // 이미지 영역의 비율을 조정합니다.
    modalImage.style.width = "100%"; // 너비를 210px로 설정
    modalImage.style.height = "95%"; 
    modalImage.style.backgroundImage = `url('static/images/map_image/map_${mapLocation}.png')`;
    modalImage.style.backgroundRepeat = "no-repeat";
    modalImage.style.backgroundSize = "contain"; // 이미지가 div 안에 완전히 들어가도록 조정합니다.
    modalImage.style.marginRight = "20px"; // 오른쪽 마진 추가
    modalImage.style.padding = "20px" ;
    modalImage.style.borderRadius = "10px";

    // 텍스트를 위한 div를 생성하고 스타일을 적용합니다.
    var newmodalText = document.createElement("div");
    newmodalText.style.flex = "3.5"; // 텍스트 영역을 나머지 공간으로 채웁니다.
    newmodalText.style.height = "100%"; // 높이를 부모와 동일하게 설정합니다.
    newmodalText.innerHTML = mapLocation+' 구역 으로 <br> 이동하시겠습니까?'; // 텍스트 내용을 설정
    newmodalText.style.display = "flex";
    newmodalText.style.flexDirection = "column";
    newmodalText.style.justifyContent = "center";
    newmodalText.style.alignItems = "center"; // 텍스트를 왼쪽 정렬합니다.
    newmodalText.style.padding = "10px"; // 내부 패딩 설정

    // '예', '아니오' 버튼을 위한 div를 생성하고 스타일을 적용합니다.
    var buttonsDiv = document.createElement("div");
    buttonsDiv.className = "modal-buttons"; // 버튼들을 위한 div에 클래스 이름 설정

    // '예' 버튼 생성
    var yesButton = document.createElement("button");
    yesButton.className = "yes-button"; // 클래스 이름 설정
    yesButton.textContent = "예";
    // 승민 : 여기 하나 위치일 때 토픽 쏘면 돼 데이터이름은 mapLocation
    yesButton.onclick = function() {
        console.log("예 버튼 클릭됨");
        modal.style.display = "none";
        location.reload();
    };

    // '아니오' 버튼 생성
    var noButton = document.createElement("button");
    noButton.className = "no-button"; // 클래스 이름 설정
    noButton.textContent = "아니오";
    // '아니오' 버튼 클릭 이벤트 여기에 추가
    noButton.onclick = function() {
        console.log("아니오 버튼 클릭됨");
        modal.style.display = "none";
        location.reload();
    };

    // 모달 바깥 클릭 시 모달이 닫히지 않도록 처리
    modal.addEventListener('click', function(event) {
        // 클릭된 요소가 모달 컨텐츠 내부인지 확인
        if (!modalContent.contains(event.target)) {
            // 모달 컨텐츠 바깥 클릭 시 이벤트를 무시
            event.stopPropagation();
        } else {
            // 모달 컨텐츠 내부 클릭 시 기본 동작 수행
            // 예: 모달 닫기, 새로고침 등
        }
    }, true); // 캡처 단계에서 이벤트 처리하기 위해 true로 설정


    // 버튼들을 buttonsDiv에 추가
    buttonsDiv.appendChild(yesButton);
    buttonsDiv.appendChild(noButton);

    // modalContent에 이미지, 텍스트 div, 버튼들을 추가합니다.
    modalContent.appendChild(modalImage);
    modalContent.appendChild(newmodalText);
    newmodalText.appendChild(buttonsDiv); // 버튼들을 텍스트 div에 추가하지만, 이 부분은 디자인에 따라 조정할 수 있습니다.
}

document.addEventListener('DOMContentLoaded', function() {
    var locationButtons = document.querySelectorAll('.locationButton');
    locationButtons.forEach(function(btn) {
        btn.addEventListener('click', function() {
            var productId = this.getAttribute('data-product-id');
            fetch('/get-location/' + productId)
                .then(response => response.json())
                .then(data => {
                    mapLocation = data.map_location; // 변수에 값을 저장
                    console.log(mapLocation);
                    changeModal(); // mapLocation 값을 성공적으로 받아온 후 changeModal 함수를 호출
                })
                .catch(error => console.error('Error:', error));
        });
    });
});

navGoal.subscribe(function(message) {
    console.log('수신된 메시지: ', message);
    // 메시지가 특정 조건을 만족할 때 동작 수행
    if (message.data === 'success') {
        console.log('목적지 도착'); 
        document.getElementById("modalText").innerHTML = "상품 위치에 도착하였습니다.<br>구매를 완료 하셨으면 <br>예 버튼을 눌러주세요 ";
        modal.style.display = "block";

        // '예' 버튼 클릭 시 수행할 동작 변경
        yesAction = function() {
            console.log('다음 경유지 이동');
            navStart.publish(goal_type1);  
            modal.style.display = "none";
        };

        noAction = function() {
            console.log("경로 안내 중지");
            modal.style.display = "none";
        };
    }
});

