#!/usr/bin/python3

import threading
import cv2
import numpy as np
import code3.axe as tools
import time
from imutils.video import VideoStream
# from imutils.video import FPS
import code3.flycontroller as flyCtrl

# 配置信息
from code3.config import (
    PROTOTXT_PATH,
    MODEL_PATH,
    DEFAULT_CONFIDENCE,
    CLASSES,
    COLORS,
)


frameList = []
useRemote = True
allStop = 0
moveStat = 0
url = "http://192.168.1.1:80/snapshot.cgi?user=admin&pwd="


class myThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        print("[INFO] 开启线程： " + self.name)
        taskStart(self.name)


def taskStart(threadName):
    global allStop, moveStat, vs
    if threadName == "Thread-1":
        print("[INFO] 等待无人稳定")
        time.sleep(3)
        if useRemote:
            while allStop != 1:
                cap = cv2.VideoCapture(url)
                if cap.isOpened():
                    ret, frame = cap.read()
                else:
                    print("[ERROR] 远程摄像头未链接 ")
                    allStop = 1
                    frame = None
                    exit(1)
                frameList.append(frame)

    if threadName == "Thread-2":

        if not useRemote:
            vs = VideoStream().start()

        # 加载模型
        print("[初始化] 加载模型...")
        net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATH, MODEL_PATH)

        found = False  # 初始化 未发现目标
        count = 1
        lastPersonBox = None

        try:
            while not found:
                # frame = cv2.imread("/home/admin-x/workspace/PyCharm/opencv/output/00.jpg", 1)
                if useRemote:
                    while True:
                        if len(frameList) != 0:
                            break
                    xframe = frameList.pop()
                else:
                    xframe = vs.read()
                # 一致化 frame大小，否则得到的目标物坐标系不一致
                xframe = tools.resize(xframe, height=500)
                (h, w) = xframe.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(xframe, (300, 300)), 0.007843, (300, 300), 127.5)
                print("[计算] 目标检测:", count)
                count += 1
                net.setInput(blob)
                detections = net.forward()

                for i in np.arange(0, detections.shape[2]):
                    confidence = detections[0, 0, i, 2]
                    if confidence > 0.95:
                        idx = int(detections[0, 0, i, 1])
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")
                        # lastPersonBox用来修正跟踪对象的矩形框大小
                        lastPersonBox = (startX, startY, endX, endY)
                        found = True
                        if CLASSES[idx] == 'person':
                            print('[调试] 第一帧目标物坐标:', lastPersonBox)
                            break  # 找到了就可以结束循环了.我们只需要坐标

            # 1：找到多个检测对象中的 目标物，通过与lastPersonBox矩形的(相交/相并)匹配度来确认目标物
            # 2：中心点的偏移

            # 保存目标物距离的队列，用来判断目标物 是前进还是后退
            move_direction = []
            MOVE_NUM = 5  # 判断移动方向的次数
            STEP_LEN = 2  # 靠近还是远离的误差值
            for i in range(0, MOVE_NUM):
                move_direction.append({
                    'length': i,
                    'time': round(time.time(), 1),
                })

            while True:
                start_time = time.time()
                timer = cv2.getTickCount()

                if useRemote:
                    while allStop != 1:
                        if len(frameList) != 0:
                            break
                    xframe = frameList.pop()
                else:
                    xframe = vs.read()

                xframe = tools.resize(xframe, height=500)
                (h, w) = xframe.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(xframe, (300, 300)), 0.007843, (300, 300), 127.5)

                # 将blob传给神经网络 获得检测和预测
                net.setInput(blob)

                # 这一步检测是最费时的
                #####
                detections = net.forward()
                #####

                # 初始化
                maxCoincidentRate = -1.0  # 最大相交率
                minDriftageLength = 0xFF  # 中心点的偏移距离

                maxCoincidentBox = lastPersonBox
                minDriftageBox = lastPersonBox

                # 处理检测到的物体
                for i in np.arange(0, detections.shape[2]):
                    confidence = detections[0, 0, i, 2]

                    if confidence > DEFAULT_CONFIDENCE:
                        index = int(detections[0, 0, i, 1])
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")

                        # 如果检测结果是person则与lastPersonBox比较，位移差别较小则是目标。
                        # 用相交面积与相并面积的比值最大来确定目标。
                        if CLASSES[index] == 'person':
                            coincident_rate = tools.intersectedRate(lastPersonBox,
                                                                    (startX, startY, endX, endY))  # 计算当前person和目标物的相交比例
                            driftage_len = tools.calcCenterLength(lastPersonBox,
                                                                  (startX, startY, endX, endY))  # 当前person和目标物的中心点距离
                            if coincident_rate > maxCoincidentRate:
                                maxCoincidentBox = (startX, startY, endX, endY)
                                maxCoincidentRate = coincident_rate
                            if driftage_len < minDriftageLength:
                                minDriftageBox = (startX, startY, endX, endY)
                                minDriftageLength = driftage_len

                        label = "{}: {:.2f}%".format(CLASSES[index],
                                                     confidence * 100)
                        cv2.rectangle(xframe, (startX, startY), (endX, endY),
                                      COLORS[index], 4)
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(xframe, label, (startX, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[index], 2)  # 打标签

                # 如果最大比值的那个框大于设定值，则确定是目标物
                if maxCoincidentRate > 0.5 and tools.shouldUpdate(lastPersonBox, maxCoincidentBox, minDriftageBox):
                    # 红色是当前确定的目标物颜色
                    # 给目标物做出标记 提示
                    label = "  target"
                    label_pos = maxCoincidentBox[1] + 15
                    cv2.putText(xframe, label, (maxCoincidentBox[0], label_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 0, 255),
                                2)  # 设置目标物的标签
                    cv2.rectangle(xframe, (maxCoincidentBox[0], maxCoincidentBox[1]),
                                  (maxCoincidentBox[2], maxCoincidentBox[3]),
                                  (0, 255, 0), 2, 2)
                    lastPersonBox = maxCoincidentBox
                    # print(maxCoincidentBox)
                    # 距离计算：
                    mid_x = (maxCoincidentBox[0] + maxCoincidentBox[2]) / 2
                    mid_y = (maxCoincidentBox[1] + maxCoincidentBox[3]) / 2
                    apx_distance = maxCoincidentBox[2] - maxCoincidentBox[0]

                    move_direction.pop(0)
                    move_direction.append({
                        'length': apx_distance,
                        'time': round(time.time(), 1)
                    })  # 把当前目标物的距离入队列
                    # print(move_direction)

                    cv2.putText(xframe, '{}'.format(apx_distance), (int(mid_x), int(mid_y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    # 靠近还是远离判断
                    near_cnt = 0  # 靠近计数器
                    far_cnt = 0  # 远离计数器

                    for i in range(1, MOVE_NUM):
                        difference = move_direction[i]['length'] - move_direction[i - 1]['length']
                        if difference > STEP_LEN:
                            near_cnt += 1
                        elif difference < -STEP_LEN:
                            far_cnt += 1

                    if apx_distance >= 300:
                        cv2.putText(xframe, 'STOP', (int(mid_x), int(mid_y + 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        moveStat = 1

                    elif near_cnt >= MOVE_NUM - 2:
                        cv2.putText(xframe, 'SLOW DOWN!!!', (int(mid_x), int(mid_y + 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                    elif far_cnt >= MOVE_NUM - 2:
                        cv2.putText(xframe, 'HURRY UP!!!', (int(mid_x), int(mid_y + 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(xframe, 'FOLLOWING!!!', (int(mid_x), int(mid_y + 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        moveStat = 2
                        print("FOLLOWING!!!")

                    if mid_x <= 200:
                        print("TURN LEFT!!!")
                        moveStat = 3
                    # elif mid_x > int(2 * w / 3):
                    elif mid_x >= 300:
                        print("TURN RIGHT!!!")
                        moveStat = 4
                cv2.imshow("Tracking", xframe)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                print('[耗时] {:.3f}s'.format(time.time() - start_time), "FPS:", int(fps))
                if int(fps) < 2:
                    moveStat = 0
                if cv2.waitKey(1) & 0xff == ord('q'):
                    allStop = 1
                    break
        except KeyboardInterrupt:
            print('异常停止')
            allStop = 1
            pass
    if threadName == "Thread-3":
        if useRemote:
            flyCtrl.takeOff()
        if not flyCtrl.yser:
            print("[ERROR] 端口初始化失败")
            allStop = 1
        while allStop == 0:
            if moveStat == 1:
                time.sleep(0.2)
            else:
                flyCtrl.xdoSend(moveStat)


threads = []

# 创建新线程
thread1 = myThread(1, "Thread-1")  # 开启远程传图,将图片装入List
thread2 = myThread(2, "Thread-2")  # 目标检测KCF跟踪算法读取List
thread3 = myThread(3, "Thread-3")  # 控制飞行状态

# 开启新线程
thread1.start()
thread2.start()
thread3.start()

# 添加线程到线程列表
threads.append(thread1)
threads.append(thread2)
threads.append(thread3)

# 等待所有线程完成
for t in threads:
    t.join()

if flyCtrl.yser:
    flyCtrl.yser.close()
    print("[INFO] 关闭端口")

print("[INFO] 退出主线程")
