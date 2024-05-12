// 모달 요소를 얻습니다.
var modal = document.getElementById("myModal");

// 버튼 객체를 얻습니다.
var btn = document.getElementById("routeButton");

// 모달 닫기 <span> 요소를 얻습니다. (별도로 <span>으로 닫는 버튼이 있다면)
var span = document.getElementsByClassName("close")[0];

// 사용자가 버튼 클릭 시 모달을 표시합니다.
btn.onclick = function() {
    modal.style.display = "block";
}

// 사용자가 <span> (x) 클릭 시, 모달을 닫습니다.


// 사용자가 모달 외부 클릭 시, 모달을 닫습니다.
window.onclick = function(event) {
    if (event.target == modal) {
        modal.style.display = "none";
    }
}

document.querySelector('.yes-button').addEventListener('click', function() {
    console.log('예 버튼 클릭');
    // 여기에 "예" 버튼 클릭 시 수행할 로직 추가
});

document.querySelector('.no-button').addEventListener('click', function() {
    console.log('아니오 버튼 클릭');
    modal.style.display = "none";
    // 여기에 "아니오" 버튼 클릭 시 수행할 로직 추가
});
