# BT-dongles-for-Espresso
BT dongles over ESP32 for Espresso : thermometer and pressure 

Those two pieces of code will receive information from a sensor (temperature or pressure) and broadcast it on BT
It is compatible with Android apps like BeanConqueror from Lars Saalbach and Pressensor CF Coffee Flow.

I connect them on my old RaspberryB board that do not have BT feature (too old) and send those values over two ESP32 that are connected to my smartphone. 
See python code on Pirok2 > multithreadBTDongle.py

Code was generated thanks to Chatgpt and iterated a lot to be completely working and resilient
Hope it helps someone...
