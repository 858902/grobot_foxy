// 모달 요소를 얻습니다.
var modal = document.getElementById("myModal");

var modalContent = document.querySelector('.modal-content');

var mapLocation ;

// 버튼 객체를 얻습니다.

var autoBtn = document.getElementById("autoButton"); // auto 버튼 객체 추가
var locBtn = document.getElementById("locationButton");

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
// 사용자가 버튼 클릭 시 모달을 표시합니다.


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


// '위치 안내 받기' 버튼 클릭 시 모달을 표시합니다.
locBtn.onclick = function() {
    document.getElementById("modalText").innerHTML = "상품 위치로 이동하시겠습니까?";
    modal.style.display = "block";
    yesAction = function() {
        console.log("상품으로 이동");
        modal.style.display = "none";
    };
    noAction = function() {
        console.log("위치 안내 취소");
        modal.style.display = "none";
    };
}
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


