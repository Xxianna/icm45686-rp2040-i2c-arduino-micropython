

使用arduino和micropython做的简单的 rp2040 和 icm45686 的通信以及上位机数据收集程序，使用i2c接口

使用方法（arduino）

|imu|rp2040|
|---|---|
|SDA |GP2|
|SCL|GP3|
|AD0|GND|

micropy的sda和scl好像使用GP0和GP1，如果使用i2c 0的话

