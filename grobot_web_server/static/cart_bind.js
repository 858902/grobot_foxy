var modal = document.getElementById("myModal");

// 새 모달 요소를 얻습니다.
var newModal = document.getElementById("newModal");

// 버튼 객체를 얻습니다.
var btn = document.getElementById("cartButton");

// 모달 닫기 <span> 요소를 얻습니다. (별도로 <span>으로 닫는 버튼이 있다면)
var span = document.getElementsByClassName("close")[0];



// 사용자가 버튼 클릭 시 모달을 표시합니다.
btn.onclick = function() {
    modal.style.display = "block";
}

// '예' 버튼 클릭 이벤트 리스너
document.querySelector('.yes-button').addEventListener('click', function() {
    console.log('예 버튼 클릭');
    // 기존 모달을 숨깁니다.
    /*modal.style.display = "none";*/
    // 새 모달을 즉시 표시합니다.
    newModal.style.display = "block";
});

// 새 모달의 닫기 버튼을 위한 코드
var newModalClose = newModal.getElementsByClassName("close")[0];
newModalClose.onclick = function() {
    newModal.style.display = "none";
};

// 모달 외부를 클릭했을 때 모달이 닫히도록 하는 코드
// 여기서 window.onclick 이벤트 리스너가 중복되어 있으므로 하나로 합쳐서 처리합니다.
window.onclick = function(event) {
    if (event.target == modal) {
        modal.style.display = "none";
    } else if (event.target == newModal) {
        newModal.style.display = "none";
    }
};

// '아니오' 버튼 클릭 이벤트 리스너
document.querySelector('.no-button').addEventListener('click', function() {
    console.log('아니오 버튼 클릭');
    modal.style.display = "none";
    // 여기에 "아니오" 버튼 클릭 시 수행할 추가 로직을 넣을 수 있습니다.
});

// 완료 버튼 객체를 얻습니다.
var completeButton = document.getElementById("nextButton");

// 완료 버튼 클릭 이벤트 리스너
completeButton.addEventListener('click', function() {
    // 모든 모달을 숨깁니다.
    
    console.log("Publishing message:", mode_on);
    modeCheck.publish(mode_on);
    micCheck2.subscribe(function(message) {
        console.log(`Receive message: ${message}`);
      });
    modal.style.display = "none";
    newModal.style.display = "none";
    // 여기에 "완료" 버튼 클릭 시 수행할 추가 로직을 넣을 수 있습니다.

});
