#include "serial.h"

int SerialPort::fd;
unsigned char SerialPort::g_write_buf[WRITE_BUFF_LENGTH];
unsigned char SerialPort::g_CRC_buf[CRC_BUFF_LENGTH];
unsigned char SerialPort::g_rec_buf[REC_BUFF_LENGTH];

int16_t SerialPort::_yaw_reduction = 0x0000;
int16_t SerialPort::_pitch_reduction = 0x0000;
int16_t SerialPort::_depth_reduction = 0x0000;


SerialPort::SerialPort()
{
    cout << "The Serial set ......" << endl;
    const char *DeviceName[4] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

    /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
     * Terminals'll default to be set as Control Terminals
     */
    struct termios newstate;
    /*打开串口*/
    bzero(&newstate, sizeof(newstate)); //清零
    for (size_t i = 0; i < (sizeof(DeviceName) / sizeof(char *)); ++i)
    {
        fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY | O_ASYNC);
        if (fd == -1)
        {
            printf("Can't Open Serial Port %s\n", DeviceName[i]);
            continue;
        }
        else
        {
            printf("Open Serial Port %s Successful\n", DeviceName[i]);
            break;
        }
    }
    cfsetospeed(&newstate, B115200);
    cfsetispeed(&newstate, B115200);

    //本地连线, 取消控制功能 | 开始接收
    newstate.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    newstate.c_cflag &= ~CSIZE;
    //设置停止位1
    newstate.c_cflag &= ~CSTOPB;
    //设置数据位8位
    newstate.c_cflag |= CS8;
    //设置无奇偶校验位，N
    newstate.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    newstate.c_cc[VTIME] = 0;
    newstate.c_cc[VMIN] = 0;

    /*清空当前串口*/
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
}

/**
 * @brief Destroy the Serial Port:: Serial Port object
 */
SerialPort::~SerialPort(void)
{
    if (!close(fd))
        printf("Close Serial Port Successful\n");
}

/**
*  @brief: 串口数据读取函数
*  @return: string  返回收到的字符串
*  @note:   逐字节读取并存到字符串
*           等待0.01s后结束读取,将所得字符串返回
*  @authors: Rcxxx
*            Hzkkk
*/
void SerialPort::receiveData(int arr[REC_BUFF_LENGTH])
{
    memset(g_rec_buf, '0', REC_BUFF_LENGTH); //清空缓存
    char rec_buf_temp[REC_BUFF_LENGTH * 2];
    read(fd, rec_buf_temp, sizeof(rec_buf_temp));

    for (int i = 0; i < (int)sizeof(rec_buf_temp); ++i)
    {
        if (rec_buf_temp[i] == 'S' && rec_buf_temp[i + sizeof(g_rec_buf) - 1] == 'E')
        {
            for (int j = 0; j < ((int)sizeof(g_rec_buf)); ++j)
            {
                g_rec_buf[j] = rec_buf_temp[i + j];
            }
            break;
        }
    }

    for (size_t i = 0; i < sizeof(g_rec_buf); ++i)
    {
        arr[i] = (g_rec_buf[i] - '0');
    }
    tcflush(fd, TCIFLUSH);
}

/**
 *@brief: RM串口发送格式化函数
 *
 * @param: yaw 云台偏航
 * @param: pitch 云台俯仰
 * @param: _yaw yaw正负
 * @param: _pitch pitch正负
 * @param: data_type 是否正确识别的标志
 * @param: data_type 是否正确识别的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 */
void SerialPort::serialWrite(int temp)
{
    g_write_buf[0] =  'S';
    g_write_buf[1] =  temp;
    g_write_buf[2] =  'E';

    write(fd, g_write_buf, sizeof(g_write_buf));
    cout << "g_write_buf=  " << g_write_buf[0]
        << "  " << static_cast<int>(g_write_buf[1])
        << "  " << (g_write_buf[2])<<endl;
    memset(g_write_buf,'\0',sizeof(g_write_buf));

    _yaw_reduction = 0x0000;
    _pitch_reduction = 0x0000;
    _depth_reduction = 0x0000;

    usleep(1);
}
