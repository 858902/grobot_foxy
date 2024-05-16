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

window.onload = function() {
    loadStatus();
};

// '예' 버튼에 대한 이벤트 리스너를 한 번만 추가합니다.
document.querySelector('.yes-button').addEventListener('click', function() {
    yesAction(); // 현재 설정된 동작을 실행합니다.
});

// cart 버튼 클릭 시 모달을 표시합니다.
cartBtn.onclick = function() {
    console.log('Cart button clicked');
    document.getElementById("modalText").innerHTML = "로봇과 카트를 결합하시겠습니까?";
    modal.style.display = "block";
    // '예' 버튼 클릭 시 수행할 동작 변경
    yesAction = function() {
        console.log('Cart combination started');
        //modal.style.display = "none";
        newModal.style.display = "block"; // 새 모달 표시
    };
}

// auto 버튼 클릭 시 모달을 표시합니다.
autoBtn.onclick = function() {
    var currentStatus = document.querySelector('.part3_bottom').textContent;
    if (currentStatus === "ON") {
        document.getElementById("modalText").innerHTML = "자율 주행 모드를 멈추시고, 수동 운전 모드로 전환하시겠습니까?";
    } else {
        document.getElementById("modalText").innerHTML = "수동 운전 모드를 멈추시고, 자율 주행 모드로 전환하시겠습니까?";
    }
    modal.style.display = "block";
    // '예' 버튼 클릭 시 수행할 동작 변경
    yesAction = function() {
        var currentStatus = document.querySelector('.part3_bottom').textContent;
        if (currentStatus === "ON") {
            console.log('Manual mode activated');
            document.querySelector('.part3_bottom').textContent = 'OFF';
        } else {
            console.log('Auto mode activated');
            document.querySelector('.part3_bottom').textContent = 'ON';
        }
        // 현재 상태를 Local Storage에 저장합니다.
        localStorage.setItem('currentStatus', document.querySelector('.part3_bottom').textContent);
        
        // 모달을 숨깁니다.
        modal.style.display = "none";
    };
}

function loadStatus() {
    // Local Storage에서 'currentStatus' 값을 가져옵니다. 값이 없다면 기본값으로 'ON'을 설정합니다.
    var currentStatus = localStorage.getItem('currentStatus') || 'ON';
    
    // 가져온 값에 따라 화면에 표시될 텍스트를 설정합니다.
    document.querySelector('.part3_bottom').textContent = currentStatus;
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

// 완료 버튼 객체를 얻습니다.
var completeButton = document.getElementById("nextButton");

// 완료 버튼 클릭 이벤트 리스너
completeButton.addEventListener('click', function() {
    console.log('Complete button clicked');
    //modeCheck.publish(mode_on);
    //micCheck2.subscribe(function(message) {
        //console.log(`Receive message: ${message}`);
     // });
    modal.style.display = "none";
    newModal.style.display = "none";
});
