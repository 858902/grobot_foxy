// 모달 요소를 얻습니다.
var modal = document.getElementById("myModal");

// 새 모달 요소를 얻습니다.
var newModal = document.getElementById("newModal");

// 버튼 객체를 얻습니다.
var cartBtn = document.getElementById("cartButton");
var autoBtn = document.getElementById("autoButton");

// 모달 닫기 <span> 요소를 얻습니다.
var span = document.getElementsByClassName("close")[0];
var newModalSpan = newModal.getElementsByClassName("close")[0];

// '예' 버튼 클릭 시 수행할 동작을 저장할 변수
var yesAction = function() {};

// '예' 버튼에 대한 이벤트 리스너를 한 번만 추가합니다.
document.querySelector('.yes-button').addEventListener('click', function() {
    yesAction(); // 현재 설정된 동작을 실행합니다.
});

window.onload = function() {
    loadStatus();
};

var cartStatus = 'Unlinked';

// 로컬 스토리지에서 'cartStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'Unlinked'를 설정합니다.
cartStatus = localStorage.getItem('cartStatus') || 'Unlinked';

// 화면에 표시될 텍스트를 설정합니다.
document.querySelector('.part3_bottom').textContent = cartStatus;

// 카트 상태를 변경하는 함수 예시
function updateCartStatus(newStatus) {
    // 새로운 상태를 변수에 저장합니다.
    cartStatus = newStatus;
    
    // 새로운 상태를 로컬 스토리지에 저장합니다.
    localStorage.setItem('cartStatus', cartStatus);
}

function loadCartStatus() {
    // 로컬 스토리지에서 'cartStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'Unlinked'를 설정합니다.
    cartStatus = localStorage.getItem('cartStatus') || 'Unlinked';
    
    // 화면에 표시될 텍스트를 설정합니다.
    document.querySelector('.part3_bottom').textContent = cartStatus;
}

// 페이지가 로드될 때 상태를 설정합니다.
document.addEventListener('DOMContentLoaded', loadCartStatus);


// cart 버튼 클릭 시 모달을 표시합니다.
cartBtn.onclick = function() {
    console.log('Cart button clicked');
    
    // cartStatus에 따라 모달 메시지를 설정합니다.
    if (cartStatus === 'Unlinked') {
        document.getElementById("modalText").innerHTML = "로봇과 카트를 결합하시겠습니까?";
    } else {
        document.getElementById("modalText").innerHTML = "로봇과 카트를 해제하시겠습니까?";
    }
    
    modal.style.display = "block";
    
    // '예' 버튼 클릭 시 수행할 동작 변경
    yesAction = function() {
        console.log('Cart combination started');
        
        if (cartStatus === 'Unlinked') {
            // 결합 동작 수행
            updateCartStatus('Linked');
            console.log('Cart and robot linked');
            // signalOffset.publish(msg_offset1);
            newModal.style.display = "block";
            
        } else {
            // 해제 동작 수행
            updateCartStatus('Unlinked');
            console.log('Cart and robot unlinked');
            modal.style.display = "none";
        }
        
    };
};


// auto 버튼 클릭 시 모달을 표시합니다.
autoBtn.onclick = function() {
    var currentStatus = document.querySelector('.part3_bottom').textContent;
    if (currentStatus === "ON") {
        document.getElementById("modalText").innerHTML = "자율 주행 모드를 멈추시고, 수동 운전 모드로 전환하시겠습니까?";
        signalOffset.publish(msg_offset1); // offset 측정 시작 
    } else {
        document.getElementById("modalText").innerHTML = "수동 운전 모드를 멈추시고, 자율 주행 모드로 전환하시겠습니까?";
    }

    modal.style.display = "block";
    // '예' 버튼 클릭 시 수행할 동작 변경
    yesAction = function() {
        if (currentStatus === "ON") 
            {
            signalOffset.publish(msg_offset2); // offset 측정 끝
            console.log('Manual mode activated');
            modeSwitch.publish(mode_type1); // 수동 모드 on
            document.querySelector('.part3_bottom').textContent = 'OFF';
            localStorage.setItem('currentStatus', document.querySelector('.part3_bottom').textContent);
            } 
        else 
            {
            console.log('Auto mode activated');
            modeSwitch.publish(mode_type2); // navigation
            document.querySelector('.part3_bottom').textContent = 'ON';
            localStorage.setItem('currentStatus', document.querySelector('.part3_bottom').textContent);
        }
        modal.style.display = "none";
    };
}


// 모달 닫기 버튼 이벤트 리스너
span.onclick = function() {
    modal.style.display = "none";
}
newModalSpan.onclick = function() {
    newModal.style.display = "none";
}

// 모달 외부 클릭 시 닫힘 처리
window.onclick = function(event) {
    if (event.target == modal) {
        modal.style.display = "none";
    } else if (event.target == newModal) {
        newModal.style.display = "none";
    }
};

// '아니오' 버튼 클릭 이벤트 리스너
document.querySelector('.no-button').addEventListener('click', function() {
    console.log('No button clicked');
    modal.style.display = "none";
});

function loadStatus() {
    // Local Storage에서 'currentStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'ON'을 설정합니다.
    var currentStatus = localStorage.getItem('currentStatus') || 'OFF';
    
    // 가져온 값에 따라 화면에 표시될 텍스트를 설정합니다.
    document.querySelector('.part3_bottom').textContent = currentStatus;
}



// 완료 버튼 객체를 얻습니다.
var completeButton = document.getElementById("nextButton");

// 완료 버튼 클릭 이벤트 리스너
completeButton.addEventListener('click', function() {
    console.log('Complete button clicked');
    // var mode_on = new ROSLIB.Message({
    //     data: 'mode_on'
    // });

    // if (modeCheck) {
    //     modeCheck.publish(mode_on);
    // } else {
    //     console.error('모드쳌 없음 안보임');
    // }
    // if (micCheck2) {
    //     micCheck2.subscribe(function(message) {
    //         console.log(`Receive message: ${message.data}`);
    //     });
    // } else {
    //     console.error('마이크쳌 없음 안보임');
    // }
    // modal.style.display = "none";
    // newModal.style.display = "none";

    modeCheck.publish(mode_on);
    // signalOffset.publish(msg_offset2); // offset 측정 끝
    micCheck2.subscribe(function(message) {
        console.log(`Receive message: ${message}`);
      });
    modal.style.display = "none";
    newModal.style.display = "none";
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