import os
import time
import cv2
import numpy as np
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
    # print(angle, error, error_angle)
    last_error = error
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(angle))
    time.sleep(1)
    piS.set_PWM_dutycycle(servo_pin, angleToDutyCycle(angle))


def color_thresh(image, s_thresh, l_thresh, b_thresh, v_thresh):
    luv= cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    lab=cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
    # 提取hsv中的s通道，lab中的b通道，luv中的l通道，hsv中的v通道
    s_channel = hsv[:,:,1]
    b_channel=lab[:,:,2]
    l_channel = luv[:,:,0]
    v_channel= hsv[:,:,2]
    # 提取S通道中符合阈值的像素点
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel > s_thresh[0]) & (s_channel <= s_thresh[1])] = 255
    # cv2.imshow("s_binary", s_binary)
    # 提取b通道中符合阈值的像素点
    b_binary = np.zeros_like(b_channel)
    b_binary[(b_channel > b_thresh[0]) & (b_channel <= b_thresh[1])] = 255
    # cv2.imshow("b_binary", b_binary)
    # 提取l通道中符合阈值的像素点
    l_binary = np.zeros_like(l_channel)
    l_binary[(l_channel > l_thresh[0]) & (l_channel <= l_thresh[1])] = 255
    # cv2.imshow("l_binary", l_binary)
    # 提取v通道中符合阈值的像素点
    v_binary = np.zeros_like(v_channel)
    v_binary[(v_channel > v_thresh[0]) & (v_channel <= v_thresh[1])] = 255
    # cv2.imshow("v_binary", v_binary)
    
    combined = np.zeros_like(s_channel)
    # 提取同时满足3个及以上通道阈值的像素点
    combined[((s_binary == 255) & (b_binary == 255) & (l_binary == 255)) | \
             ((s_binary == 255) & (b_binary == 255) & (v_binary == 255)) | \
             ((s_binary == 255) & (l_binary == 255) & (v_binary == 255)) | \
             ((b_binary == 255) & (l_binary == 255) & (v_binary == 255))] = 255
    
    return combined


def videoProcessing(piS, servo_pin):
    # cv2.namedWindow("trackbar", cv2.WINDOW_NORMAL)
    # cv2.createTrackbar("s_min", "trackbar", 0, 255, lambda x: None)
    # cv2.createTrackbar("s_max", "trackbar", 60, 255, lambda x: None)
    # cv2.createTrackbar("l_min", "trackbar", 145, 255, lambda x: None)
    # cv2.createTrackbar("l_max", "trackbar", 255, 255, lambda x: None)
    # cv2.createTrackbar("b_min", "trackbar", 90, 255, lambda x: None)
    # cv2.createTrackbar("b_max", "trackbar", 160, 255, lambda x: None)
    # cv2.createTrackbar("v_min", "trackbar", 150, 255, lambda x: None)
    # cv2.createTrackbar("v_max", "trackbar", 255, 255, lambda x: None)
    cap = cv2.VideoCapture(0)
    mask = np.zeros((400, 600), dtype=np.uint8)
    mask = cv2.fillPoly(mask, [np.array([[50, 150], [550, 150], [600, 350], [0, 350]])], 255)
    if not cap.isOpened():
        print("视频打开失败")
        return
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (600, 400))
        # 提取符合阈值的像素点
        # s_min = cv2.getTrackbarPos("s_min", "trackbar")
        # s_max = cv2.getTrackbarPos("s_max", "trackbar")
        # l_min = cv2.getTrackbarPos("l_min", "trackbar")
        # l_max = cv2.getTrackbarPos("l_max", "trackbar")
        # b_min = cv2.getTrackbarPos("b_min", "trackbar")
        # b_max = cv2.getTrackbarPos("b_max", "trackbar")
        # v_min = cv2.getTrackbarPos("v_min", "trackbar")
        # v_max = cv2.getTrackbarPos("v_max", "trackbar")
        s_min, s_max, l_min, l_max, b_min, b_max, v_min, v_max = 139, 255, 118, 181, 142, 255, 196, 255
        combined = color_thresh(frame, s_thresh=(s_min, s_max), l_thresh=(l_min, l_max), b_thresh=(b_min, b_max), v_thresh=(v_min, v_max))
        binary = cv2.bitwise_and(combined, mask)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_x = []
        for contour in contours:
            if cv2.contourArea(contour) < 100:
                continue
            rect = cv2.minAreaRect(contour)
            angle = rect[2]
            width = rect[1][0]
            height = rect[1][1]

            if width < height:
                angle += 90
            print(angle)

            if angle > 150 or angle < 30:
                continue
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            # cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

            center_x.append(rect[0][0])
        center = 300
        if len(center_x) > 0:
            center_x.sort()
            if len(center_x) >= 2:
                if center_x[0] < 300 and center_x[-1] > 300:
                    center = (center_x[0] + center_x[-1]) / 2
                elif center_x[0] < 300 and center_x[-1] < 300:
                    center = center_x[0] / 2 + 300
                elif center_x[0] > 300 and center_x[-1] > 300:
                    center = center_x[0] / 2
            else:
                if center_x[0] < 300:
                    center = center_x[0] / 2 + 300
                else:
                    center = center_x[0] / 2
            # cv2.circle(frame, (int(center), 200), 5, (0, 255, 0), -1)
        # cv2.imshow("frame", frame)
        print(center)
        pidControl(piS, center, servo_pin)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def forward(piM, pwm_pin):
    time.sleep(5)
    i = 10000
    while True:
        piM.set_PWM_dutycycle(pwm_pin, i)
        i += 1000
        if i >= 12000:
            i = 10000
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
    # time.sleep(1)
    # try:
    #     forward(pi, pwm_pin)
    # except KeyboardInterrupt:
    #     print("程序结束")
    #     os.system("sudo killall pigpiod")
