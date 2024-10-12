import os
import time
import cv2
import pigpio as gpio
import threading


def angleToDutyCycle(angle):
    return 2.5 + (angle / 180) * 10


def initServo(piS, servo_pin):
    piS.set_PWM_frequency(servo_pin, 50)
    piS.set_PWM_range(servo_pin, 100)
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(45))
    time.sleep(1)
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(135))
    time.sleep(1)
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(85))

    print("舵机初始化完成")


def initMotor(piM, pwm_pin):    
    piM.set_mode(pwm_pin, gpio.OUTPUT)

    piM.set_PWM_frequency(pwm_pin, 200)
    piM.set_PWM_range(pwm_pin, 40000)
    piM.set_PWM_dutycycle(pwm_pin, 10000)
    time.sleep(2)

    print("电机初始化完成")


def pidControl(piS, angle, servo_pin):
    global kp
    global kd 
    global last_error
    global error

    kp = 0.33
    kd = 0.11

    last_error = 0
    error = 0

    angle_outmax = 25
    angle_outmin = -45

    error = angle - 300
    error_angle = kp * error + kd * (error - last_error)

    if error_angle > angle_outmax:
        error_angle = angle_outmax
    elif error_angle < angle_outmin:
        error_angle = angle_outmin

    angle = 85 - error_angle
    print(angle, error, error_angle)
    last_error = error
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(angle))
    time.sleep(1)
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(angle))


def videoProcessing(piS, servo_pin):
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("摄像头未打开")
            break

        frame = cv2.resize(frame, (600, 400))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 1))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)
        canny = cv2.Canny(blackhat, 150, 255)
        t = 0
        left = [-1 for i in range(0, 100)]
        right = [-1 for i in range(0, 100)]
        mid = [-1 for i in range(0, 100)]
        mid_sum = 0
        left[0] = 0
        right[0] = 0
        cv2.line(gray, (0, 370), (600, 370), (0, 0, 255), 2)
        cv2.line(gray, (0, 270), (600, 270), (0, 0, 255), 2)

        for i in range(370, 270, -1):
            flag_left = 0
            flag_right = 0
            for z in range(0, 600):
                if canny[i, z] == 255:
                    left[t] = z
                    flag_left = 1
                    cv2.circle(gray, (z, i), 1, (255, 0, 0), 2)
                    for j in range(z, 600):
                        if canny[i][j] == 255 and j - z > 100:
                            flag_right = 1
                            right[t] = j
                            cv2.circle(gray, (j, i), 1, (255, 0, 0), 2)
                            break
                    break
                if flag_left == 0:
                    left[t] = 0
                    cv2.circle(gray, (left[t], i), 1, (255, 0, 0), 2)
                    right[t] = 599
                    cv2.circle(gray, (right[t], i), 1, (255, 0, 0), 2)

                if flag_left == 1 and flag_right == 0:
                    right[t] = 599

            if t > 0:
                if flag_right == 0:
                    if left[t - 1] - left[t] > 0:
                        n = left[t - 1]
                        left[t - 1] = 599 - right[t - 1]
                        right[t - 1] = n
                        cv2.circle(gray, (left[t - 1], i), 1, (255, 0, 0), 2)
                    else:
                        cv2.circle(gray, (right[t - 1], i), 1, (255, 0, 0), 2)
                    
                mid[t - 1] = (left[t - 1] + right[t - 1]) / 2
                mid_sum += mid[t - 1]

            t += 1
        mid_final = mid_sum / 100
        cv2.imshow("gray", gray)
        if cv2.waitKey(1) & 0xFF == 27:
            break
        print(mid_final)
        pidControl(piS, mid_final, servo_pin)

    cap.release()
    cv2.destroyAllWindows()


def forward(piM, pwm_pin):
    while True:
        piM.set_PWM_dutycycle(pwm_pin, 10000)
        time.sleep(3)


if __name__ == "__main__":
    os.system("sudo killall pigpiod")
    os.system("sudo pigpiod")
    os.system("sudo systemctl stop network-rc.service")
    time.sleep(2)
    servo_pin = 12
    pwm_pin = 13
    pi = gpio.pi()
    initServo(pi, servo_pin)
    initMotor(pi, pwm_pin)
    time.sleep(1)
    thread_main = threading.Thread(target=videoProcessing, args=(pi, servo_pin))
    thread_main.start()
    time.sleep(1)
    try:
        forward(pi, pwm_pin)
    except KeyboardInterrupt:
        print("程序结束")
        os.system("sudo killall pigpiod")
