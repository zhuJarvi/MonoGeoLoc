#include <iostream>
#include <vector>
#include <string>
#include "unistd.h"
#include <cstring>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

using namespace itas109;
using namespace std;

// 自定义串口监听类（处理接收事件）
class SerialListener : public CSerialPortListener {
public:
    void onReadEvent(const char* portName, unsigned int readLen) override {
        if (readLen <= 0) return;

        vector<char> buf(readLen + 1, 0);
        int recv = m_sp->readData(buf.data(), readLen);
        if (recv > 0) {
            cout << "\n[" << portName << "] 收到数据(" << recv << "字节): " 
                 << buf.data() << endl;
        }
    }

    void setSerialPort(CSerialPort* sp) { m_sp = sp; }

private:
    CSerialPort* m_sp = nullptr;
};

int main() {
    CSerialPort sp;
    SerialListener listener;
	listener.setSerialPort(&sp);

    // 4. 打开串口（9600波特率，8N1，无流控）
    sp.init(
        "/dev/ttyUSB0",
        BaudRate115200,          // 波特率
        ParityNone,     // 校验：无
        DataBits8,      // 数据位：8
        StopOne,      // 停止位：1
        FlowNone,       // 流控：无
        4096
    );
	sp.setReadIntervalTimeout(0);
	sp.open();
    if (!sp.isOpen()) {
        cerr << "打开串口失败: " << sp.getLastErrorMsg() << endl;
        return -1;
    }

	sp.connectReadEvent(&listener);

    char hex[5];
    hex[0] = 0x31;
    hex[1] = 0x32;
    hex[2] = 0x33;
    hex[3] = 0x34;
    hex[4] = 0x35;
    sp.writeData(hex, sizeof(hex));

    // 写入字符串数据
    sp.writeData("Dapenson", 8);

    // 循环等待
    for (;;)
    {
                std::cout << "发送数据: " ;

        // 延时1S
        sleep(1);
        // 写入字符串数据
        const char *data_send = "Dapenson\n";
    sp.writeData("Dapenson", 8);
    }

    return 0;
}