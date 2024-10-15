import cv2
import time
import pigpio as pio
import os
import threading#多线程

open_io="sudo pigpiod"
 
close_io="sudo killall pigpiod"
os.system(close_io)
os.system(open_io)
os.system("sudo systemctl stop network-rc.service")
time.sleep(2)

def angleToDutyCycle(angle):
   return 2.5 + (angle / 180.0)* 10

servo_pin = 12                # 舵机信号线接树莓派GPIO17
pi1= pio.pi()
pi1.set_PWM_frequency(servo_pin,50)  # frequency 50Hz
pi1.set_PWM_range(servo_pin,100)    # set range 1000
pi1.set_PWM_dutycycle(servo_pin,angleToDutyCycle(85))  # set an initial position 92
print("舵机初始化完成！")
time.sleep(2)                 # wait for 2 seconds

'''
for dy in range(60,150,10):#230
		pi1.set_PWM_dutycycle(servo_pin,angleToDutyCycle(dy))
		print (dy)
		time.sleep(3)

for dy in range(140,50,-10):#230
		pi1.set_PWM_dutycycle(servo_pin,angleToDutyCycle(dy))
		print (dy)
		time.sleep(3)
'''
#---------------------PID系数----------------------------------
kp = 0.33  
kd =0.11  

last_error=0
error=0 
#限幅
angle_outmax=25
angle_outmin=-45  

#---------------------------------------------------------
#GPIO初始化
pwm_pin = 13 #定义pwm输出引脚：电调口 13
pi = pio.pi()#实例化pigpio对象
pi.set_mode(pwm_pin,pio.OUTPUT) #设置PWM引脚为输出状态

#pwm初始化
pi.set_PWM_frequency(pwm_pin,200)#设定14号引脚产生的pwm波形的频率为50Hz

#在默认采样率5下只能取
#8000  4000  2000 1600 1000  800  500  400  320
#250   200   160  100   80   50   40   20   10

pi.set_PWM_range(pwm_pin,40000) #限制占空比的范围（一个pwm周期分成多少份），总范围25-40000。

#解锁
pi.set_PWM_dutycycle(pwm_pin,10000) #指定pwm波形脉宽
time.sleep(2)

# for dy in range(10000,14000,1000):#230
# 	pi.set_PWM_dutycycle(pwm_pin,dy)
# 	print (f"*************{dy}")
# 	time.sleep(2)

# for dy in range(9610,9500,-20):#230
#       pi.set_PWM_dutycycle(pwm_pin,dy)
# 			print (dy)
# 			time.sleep(3)

#----------------------------PD调节舵机----图像处理---------------------------------------------------------------------------------------------------------    
cap= cv2.VideoCapture(0)
def main():
  while(cap.isOpened()):
    ret, img = cap.read()
    k = cv2.waitKey(1)
    img_=cv2.resize(img,(600,400))
    img1 = cv2.cvtColor(img_, cv2.COLOR_BGR2GRAY)
  
    ret,binary = cv2.threshold(img1,200,255,0)#阈值处理
    kerne = cv2.getStructuringElement(cv2.MORPH_RECT,(35,1)) #横线检测
    blackhatImg = cv2.morphologyEx(img1,cv2.MORPH_BLACKHAT,kerne) #黑帽
    image_1 = cv2.Canny(blackhatImg, 150,255)
    #cv2.imshow("3",image_1)
    t = 0
    left = [-1 for i in range(0,100)]
    right = [-1 for i in range(0,100)]
    mid = [-1 for i in range(0,100)]
    mid_sum = 0
    left[0]=0
    right[0]=0
    cv2.line(img1,(0,370),(600,370),(0,0,255),2)
    cv2.line(img1,(0,270),(600,270),(0,0,255),2)
    
    for i in range(370,270,-1):
        flag_left=0
        flag_right=0  
        for z in range(0,600):
            if(image_1[i][z]==255):
                flag_left=1
                #cv2.circle(img1, (z,i),1,(255,0,0),1)
                left[t] = z #假设为左边线
                for j in range(z,600):
                    if((image_1[i][j]==255)and(j-z>100)):
                        flag_right=1
                        right[t] = j
                        #cv2.circle(img1, (j,i),1, (255,0,0),1)
                        break
                break

            if(flag_left==0):
                left[t] = 0
                #cv2.circle(img1, (left[t],i),1, (255,0,0),1)
                right[t] = 599
                #cv2.circle(img1, (right[t],i),1, (255,0,0),1)
            
            if(flag_left==1 and flag_right==0):
                right[t] = 599  
       
        if(t>0):
            if(flag_right==0):
                if(left[t-1]-left[t]>0):
                    n = left[t-1]
                    left[t-1]=599-right[t-1]
                    right[t-1]=n
                    #cv2.circle(img1, (left[t-1],i),1, (255,0,0),1)
                #else:
                    #cv2.circle(img1, (right[t-1],i),1, (255,0,0),1)


            mid[t-1] = (left[t-1]+right[t-1])/2
            mid_sum = mid[t-1] + mid_sum
        t = t + 1
    mid_final = mid_sum/100

    global kp
    global kd
    global last_error
    global error
	
    error=mid_final-300 
    error_angle = kp*error + kd*(error-last_error)
	
    if(error_angle>angle_outmax):
      error_angle=angle_outmax
    if(error_angle<angle_outmin):
      error_angle=angle_outmin
	
    angle=85-error_angle 
    print(mid_final,        error,         error_angle,        angle)#打印赛道中线，赛道中线与图像偏差，角度偏差，角度        
    last_error = error
    pi1.set_PWM_dutycycle(servo_pin,angleToDutyCycle(angle))
    time.sleep(1)
    pi1.set_PWM_dutycycle(servo_pin,angleToDutyCycle(angle)) # 清空当前占空比，使舵机停止抖动

thread_main= threading.Thread(target=main)
thread_main.start()

def dian():
	while 1:
		pi.set_PWM_dutycycle(pwm_pin,11000)
		time.sleep(2)

try:   
   dian()
   pass
except KeyboardInterrupt:
   print ("over!")
   cv2.destroyAllWindows()
   cap.release()
   os.system(close_io)