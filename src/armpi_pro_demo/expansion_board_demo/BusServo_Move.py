import time
import Board

print('''
**********************************************************
*********功能:幻尔科技树莓派扩展板，串口舵机运动例程*************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

while True:
	# 参数：参数1：舵机id; 参数2：位置; 参数3：运行时间
	# 舵机的转动范围0-240度，对应的脉宽为0-1000,即参数2的范围为0-1000

	Board.setBusServoPulse(6, 800, 1000) # 6号舵机转到800位置，用时1000ms
	time.sleep(0.5) # 延时0.5s

	Board.setBusServoPulse(6, 200, 1000) # 6号舵机转到200位置，用时1000ms
	time.sleep(0.5) # 延时0.5s
    
    
