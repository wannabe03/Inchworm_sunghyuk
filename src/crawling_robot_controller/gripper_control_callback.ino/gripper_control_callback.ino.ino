#include <Servo.h>

#define DEFAULT_POS 50  // 시리얼 통신이 없을 때 기본 위치
#define OPEN_POS 70     // 그립이 열릴 때 위치
#define GRIP_POS 0     // 그립이 닫힐 때 위치
#define SWEEP_DELAY 10  // 서보 모터 이동 속도 조절 (ms)

Servo base_grip;
Servo ee_grip;

int base_command = 0;
int ee_command = 0;
int base_target = DEFAULT_POS;
int ee_target = DEFAULT_POS;

// 서보 모터를 부드럽게 이동시키는 함수
void moveServoSmoothly(Servo &servo, int &current_pos, int target_pos) {
    while (current_pos != target_pos) {
        if (current_pos < target_pos) {
            current_pos++;  // 목표 위치까지 증가
        } else if (current_pos > target_pos) {
            current_pos--;  // 목표 위치까지 감소
        }
        servo.write(current_pos);  // 서보 모터 이동
        delay(SWEEP_DELAY);        // 천천히 이동하도록 딜레이 추가
    }
}

void setup() {
    Serial.begin(115200);  // 시리얼 통신 시작
    base_grip.attach(9);
    ee_grip.attach(10);

    // 초기 위치를 디폴트 값으로 설정
    base_grip.write(DEFAULT_POS);
    ee_grip.write(DEFAULT_POS);
}

void loop() {
    static int prev_base_command = 0;
    static int prev_ee_command = 0;

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // 줄바꿈 문자('\n')까지 데이터 읽기
        int commaIndex = input.indexOf(',');

        if (commaIndex > 0) {
            base_command = input.substring(0, commaIndex).toInt();
            ee_command = input.substring(commaIndex + 1).toInt();
        }

        // base_command 값이 0이면 GRIP_POS, 아니면 OPEN_POS
        base_target = (base_command == 0) ? GRIP_POS : OPEN_POS;
        ee_target = (ee_command == 0) ? GRIP_POS : OPEN_POS;
    } else {
        // 시리얼이 멈추면 기본 위치로 복귀
        base_target = DEFAULT_POS;
        ee_target = DEFAULT_POS;
    }

    // 순서 적용: 먼저 닫는 동작 수행 후 여는 동작 수행
    if (prev_base_command == 1 && base_command == 0) {
        // Base가 먼저 닫혀야 하는 경우
        moveServoSmoothly(base_grip, prev_base_command, base_target);
    }
    if (prev_ee_command == 1 && ee_command == 0) {
        // EE가 먼저 닫혀야 하는 경우
        moveServoSmoothly(ee_grip, prev_ee_command, ee_target);
    }

    if (prev_base_command == 0 && base_command == 1) {
        // Base가 열리는 동작 (이전 그립을 다 끝내고 수행)
        moveServoSmoothly(base_grip, prev_base_command, base_target);
    }
    if (prev_ee_command == 0 && ee_command == 1) {
        // EE가 열리는 동작 (이전 그립을 다 끝내고 수행)
        moveServoSmoothly(ee_grip, prev_ee_command, ee_target);
    }

    prev_base_command = base_command;
    prev_ee_command = ee_command;
}
