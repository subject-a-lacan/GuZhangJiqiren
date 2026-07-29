FIREWATER格式：
本协议是CSV风格的字符串流，直观简洁，编程像printf简单。但由于字符串解析消耗更多的运算资源（无论在上位机还是下位机），建议仅在通道数量不多、发送频率不高的时候使用。

采样数据解析#
数据格式#
"<any>:ch0,ch1,ch2,...,chN\n"
any和冒号可以为空，但换行(\n)不可省略；
any不可以为"image"，这个前缀用于解析图片数据；
此处\n为换行，并非指字符斜杠+字符n；
\n也可以为\n\r，或\r\n。
发送4个曲线的数据长这个样子
"channels: 1.386578,0.977929,-0.628913,-0.942729\n"

或者不加any和冒号
"1.386578,0.977929,-0.628913,-0.942729\n"


justfloat格式：
重点
在51单片机中，浮点为大端，使用JustFloat需要调换一下字节序；
字节接收区请勾选十六进制，以十六进制方式打印字符，否则只能打印乱码。
协议特点#
本协议是小端浮点数组形式的字节流协议，纯十六进制浮点传输，节省带宽。此协议非常适合用在通道数量多、发送频率高的时候。

采样数据解析#
数据格式#
#define CH_COUNT <N>
struct Frame {
    float ch_data[CH_COUNT];
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
};
ch_data为小端浮点数组，里面放着需要发送的CH_COUNT个通道。
tail为帧尾。
发送4个曲线的数据长这个样子

bf 10 59 3f b1 02 95 3e 57 a6 16 be 7b 4d 7f bf 00 00 80 7f
Arduino示例代码#
Copy
void setup() {
 Serial.begin(115200);
}

float t = 0;
void loop() {
    t += 0.1;
    // 发送数据
    float ch[4];  
    ch[0] = sin(t);
    ch[1] = sin(2*t);
    ch[2] = sin(3*t);
    ch[3] = sin(4*t);
    Serial.write((char *)ch, sizeof(float) * 4); 
    // 发送帧尾
    char[4] tail = {0x00, 0x00, 0x80, 0x7f};
    Serial.write(tail, 4);
    delay(100);
}