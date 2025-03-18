#include <iostream>
#include <cmath>

using namespace std;

int main() {
    double x, y;
    const double L = 1.0;  // 링크의 길이, 필요에 따라 조정하세요.

    cout << "x와 y 값을 입력하세요: ";
    cin >> x >> y;
    
    // theta2 계산: 주어진 식에서 acos의 결과는 0~pi 범위이므로 부호에 주의하세요.
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    // theta1 계산: atan2와 asin 함수를 사용
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    // theta3 계산: theta1과 theta2의 합의 음수
    double theta3 = -(theta1 + theta2);

    cout << "theta1: " << theta1 << endl;
    cout << "theta2: " << theta2 << endl;
    cout << "theta3: " << theta3 << endl;
    
    return 0;
}
