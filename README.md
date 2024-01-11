# Tello無人機追蹤Apriltag飛行

## 目的

使用Tello相機追蹤Apriltag，控制無人機進行上下左右前後飛行。

## Apriltag定位

1. 透過相似三角形計算Ariltag與無人機的相對距離
2. 利用Apriltag中心座標與攝影機畫面中心座標差值得到無人機相對座標

## 控制方法

- 使用PID控制，輸入距離誤差值，得到速度。

## 簡報影片連結

- [https://www.canva.com/design/DAF5eV5yMMc/zMw7WZC1kPoWArvp2U73zw/view?utm_content=DAF5eV5yMMc&utm_campaign=designshare&utm_medium=link&utm_source=editor](https://www.canva.com/design/DAF5eV5yMMc/zMw7WZC1kPoWArvp2U73zw/view?utm_content=DAF5eV5yMMc&utm_campaign=designshare&utm_medium=link&utm_source=editor)

![Untitled](Tello%E7%84%A1%E4%BA%BA%E6%A9%9F%E8%BF%BD%E8%B9%A4Apriltag%E9%A3%9B%E8%A1%8C%20d7fc82893d6b4160b12f9bbd62440e95/Untitled.png)

## 控制結果

![Control System Final Presentation.png](Tello%E7%84%A1%E4%BA%BA%E6%A9%9F%E8%BF%BD%E8%B9%A4Apriltag%E9%A3%9B%E8%A1%8C%20d7fc82893d6b4160b12f9bbd62440e95/Control_System_Final_Presentation.png)

## 結論

- 訊號延遲大，反應時間稍慢，可以再調整PID參數。
- 有些穩態誤差，因沒有加入積分項參數，但主因為無人機速度需大於10才會有明顯移動
- 可以讀取Apriltag的法向量，並加入Z軸旋轉(pitch)控制

## 參考資料

1. [https://djitellopy.readthedocs.io/en/latest/tello/](https://djitellopy.readthedocs.io/en/latest/tello/)
2. [https://blog.csdn.net/zhuoqingjoking97298/article/details/122229650](https://blog.csdn.net/zhuoqingjoking97298/article/details/122229650)
