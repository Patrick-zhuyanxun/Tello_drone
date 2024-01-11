'''
    專案名稱:Tello 無人機偵測Apriltag飛行
    Contributor:劉茂德、朱彥勳
    製作日期:2024年1月
    reference:
        1.Tello API:https://djitellopy.readthedocs.io/en/latest/tello/
        2.Apriltag: https://blog.csdn.net/zhuoqingjoking97298/article/details/122229650
'''

from simple_pid import PID
import pupil_apriltags as apriltag
import cv2
import numpy as np
from djitellopy.tello import Tello
import threading
import time
import matplotlib.pyplot as plt
import time


def estimate_distance(focal_length, tag_size_in_pixels, known_tag_size):
    # 使用相似三角形原理估算距離
    distance = (known_tag_size * focal_length) / tag_size_in_pixels
    return distance

final_distance = 0  #y軸與目標距離相差距離
final_axis = [0,0]  #x,z軸定位點
stop_flag = False   #停止訊號
plot_flag = False   #開始畫圖訊號
pidx = PID(1, 0, 0.1, setpoint=0,sample_time=0.05,output_limits=(-1,1)) #建立x方向的pid控制器
pidy = PID(1, 0, 0.1, setpoint=0,sample_time=0.05,output_limits=(-1,1)) #建立y方向的pid控制器
pidz = PID(1, 0, 0.1, setpoint=0,sample_time=0.05,output_limits=(-1,1)) #建立z方向的pid控制器
start_time = time.time()    #紀錄時間，畫圖用
GOAL_DIS = 1.0  #y軸目標距離，單位公尺

mydrone = Tello()               
mydrone.connect()               #無人機初始化
print(mydrone.get_battery())    #獲取無人機電池電量

lr,fb,ud,yv = 0,0,0,0           #無人機控制訊號參數，分別為左右、前後、上下、順逆時針
dif_arr_x = []                  #畫圖用矩陣建立
dif_arr_y = []
dif_arr_z = []
dis_arr_x = []
dis_arr_y = []
dis_arr_z = []
time_arr_1 = []
time_arr_2 = []


def get_distance():         #獲取無人機與目標點距離

    global final_distance,final_axis,stop_flag,GOAL_DIS     #參數初始化
    global lr,fb,ud,yv
    global dis_arr_x, dis_arr_y, dis_arr_z
    global plot_flag, time_arr_1
    focal_length_value = 800        #相機成像與焦點距離，由相似三角形算法推得    
    known_tag_size = 0.0385         #Apriltag Size 單位公尺

    mydrone.takeoff()       #無人機起飛
    mydrone.streamon()      #開啟相機
    cap = mydrone.get_frame_read()  #讀取相機畫面
    mydrone.FPS_30      #設置相機幀率

    at_detector = apriltag.Detector(families='tag36h11 tag25h9')  #for windows 偵測Apriltag 36h11與25h9家族
    i=0             #Apriltag 消失計數器
    count_time = 10     #Apriltag 消失幾幀設定值
    count = count_time
    while(1):

        frame = cap.frame
        (h, w) = frame.shape[:2]
        # 檢測鍵盤資訊
        k=cv2.waitKey(1)
        if k==27:
            break
        elif k & 0xFF == ord('q'):      #按下q跳出迴圈
            break
        # 檢測apriltag
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #圖片轉灰階，減少運算亮
        tags = at_detector.detect(gray)
        #print(tags)
        if tags == []:                  #計算未抓到Apriltag畫面次數，超過設定次數判定消失，讓無人機待在原地不動
            if count>=0:
                count-=1
            else:
                lr,fb,ud,yv=0,0,0,0
                final_distance = GOAL_DIS
                final_axis = [0,0]
        for tag in tags:                #將所有偵測到的Apriltag四個點及中心法向量標示在影像上
            count = count_time
            # print(type(tag.center))
            cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            cv2.line(frame, tuple(tag.center.astype(int)), (w//2, h//2), (0, 0, 255), 2)
            distance = []
            for corner in tag.corners:
                tag_size_in_pixels = ((corner[0]-tag.center[0])**2+(corner[1]-tag.center[1])**2)**(1/2)
                dis = estimate_distance(focal_length_value, tag_size_in_pixels, known_tag_size)
                distance.append(dis)
            
            final_distance = np.mean(distance)      #計算之Y軸距離
            final_axis = tag.center-[w/2, h/2]      #計算之X,Z軸像素差值(應該要以三角形公式推算距離與像素比例較為準確，但尚未更改程式碼)
            if(plot_flag):                          #距離紀錄
                dis_arr_x.append(final_axis[0]/400)
                dis_arr_y.append(final_distance)
                dis_arr_z.append(final_axis[1]/100)
                time_arr_1.append(time.time()-start_time)
                
            if final_distance !=0:         #初次偵測到Apriltag時，啟動控制函數，避免一開始抓不到偵測點，導致無人機暴衝
                event.set()
            #time.sleep(1/30)
            #if final_distance != 0:
                #lock.release()
            #print(final_result)
            #print(diff)
        # 显示检测结果
        cv2.imshow('capture', frame)
    stop_flag = True    #啟動停止旗標
    mydrone.streamoff() #關閉無人機鏡頭
    #cap.release()
    # stop_flag = True
    cv2.destroyAllWindows()
    
def get_control():          #控制函式
    global lr,fb,ud,yv
    global final_axis,final_distance,q_flag,pidx,pidy,GOAL_DIS
    global dif_arr_x, dif_arr_y, dif_arr_z
    global plot_flag,time_arr_2
    max_speed = [30,50,30]
    event.wait()            # 等待Apriltag被偵測
    event.clear()           # 觸發後將事件回歸原本狀態
    while(1):
        #lr,fb,ud,yv = 0,0,0,0
        difx = final_axis[0]/400            #X方向像素正規化(避免數值過大，超出pid輸出範圍)
        dify = final_distance-GOAL_DIS      #y軸距離差值計算
        difz = final_axis[1]/100            #z方向像素正規化(避免數值過大，超出pid輸出範圍)
        dif = [difx,dify,difz]
        plot_flag = True        #開始記錄控制資訊
        dif_arr_x.append(difx)
        dif_arr_y.append(dify)
        dif_arr_z.append(difz)
        time_arr_2.append(time.time()-start_time)

        time.sleep(0.05)
        control  = [pidx(difx),pidy(dify),pidz(difz)]
        print('distance:',dif,'control',control)
        if (control[1]>=0):         #因無人機往後飛行給定同樣速度飛行較慢，加上一增益
            control[1] = control[1]*1.5
        lr =  -(int(control[0]*max_speed[0]))       #設定無人機控制訊號
        fb =  -(int(control[1]*max_speed[1]))
        ud =  (int(control[2]*max_speed[2]))

        if stop_flag:       #使用者按下Q，程式中止
            print('end')
            mydrone.end()
            time.sleep(3)
            break

event = threading.Event()   # 註冊Apriltag讀取事件
get_dis_f = threading.Thread(target=get_distance)  # 建立計算距離的執行緒
get_control_f = threading.Thread(target=get_control)  # 建立控制函數的執行緒


get_dis_f.start()  # 啟用執行緒
get_control_f.start()   #啟用執行緒
while (not stop_flag):
    mydrone.send_rc_control(lr,fb,ud,yv)    #送出控制訊號

#繪圖
plt.subplot(211)
plt.plot(time_arr_1,dis_arr_x)
plt.plot(time_arr_1,dis_arr_y)
plt.plot(time_arr_1,dis_arr_z)
plt.ylabel('distance')
plt.legend(["dis_x", "dis_y", "dis_z"])

plt.subplot(212)
plt.xlabel('time')
plt.ylabel('error')
plt.plot(time_arr_2, dif_arr_x)
plt.plot(time_arr_2, dif_arr_y)
plt.plot(time_arr_2, dif_arr_z)
plt.legend(["dif_x", "dif_y", "dif_z"])
plt.show()
