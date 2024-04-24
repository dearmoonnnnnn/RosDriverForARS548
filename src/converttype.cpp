#include "converttype.h"

ConvertType::ConvertType()
{

}

unsigned short ConvertType::byteToUshort(char *x)
{
    unsigned short r;
    r = ((unsigned short)((*x)&0x00ff)<<8) | *(x+1);
    return r;
}

unsigned int ConvertType::byteToUint(char *x)
{
    unsigned int r;
    r = (((unsigned int)((*x)&0x000000ff))<<24) |(((unsigned int)((*(x+1))&0x000000ff))<<16)|(((unsigned int)((*(x+2))&0x000000ff))<<8)| (unsigned int)((*(x+3))&0x000000ff);
    return r;
}

unsigned long long ConvertType::byteToUlong(char *x)
{
    unsigned long long r;
    r = ((unsigned long long)((*x)&0x00000000000000ff)<<56) |((unsigned long long)((*(x+1))&0x00000000000000ff)<<48)|((unsigned long long)((*(x+2))&0x00000000000000ff)<<40)| ((unsigned long long)((*(x+3))&0x00000000000000ff)<<32)|\
            ((unsigned long long)((*(x+4))&0x00000000000000ff)<<24)|((unsigned long long)((*(x+5))&0x00000000000000ff)<<16)|((unsigned long long)((*(x+6))&0x00000000000000ff)<<8)|((unsigned long long)((*(x+7))&0x00000000000000ff));
    return r;
}

signed short ConvertType::byteToSshort(char *x)
{
    signed short r;
    r = (signed short)(((unsigned short)((*x)&0x00ff)<<8) | *(x+1));
    return r;
}

signed int ConvertType::byteToSint(char *x)
{
    signed int r;
    r = (signed int)(((unsigned int)((*x)&0x000000ff)<<24) |((unsigned int)((*(x+1))&0x000000ff)<<16)|((unsigned int)((*(x+2))&0x000000ff)<<8)| (unsigned int)((*(x+3))&0x000000ff));
    return r;
}

signed long long ConvertType::byteToSlong(char *x)
{
    signed long long r;
    r = (signed long long)(((unsigned long long)((*x)&0x00000000000000ff)<<56) |((unsigned long long)((*(x+1))&0x00000000000000ff)<<48)|((unsigned long long)((*(x+2))&0x00000000000000ff)<<40)| ((unsigned long long)((*(x+3))&0x00000000000000ff)<<32)|\
            ((unsigned long long)((*(x+4))&0x00000000000000ff)<<24)|((unsigned long long)((*(x+5))&0x00000000000000ff)<<16)|((unsigned long long)((*(x+6))&0x00000000000000ff)<<8)|((unsigned long long)((*(x+7))&0x00000000000000ff)));
    return r;
}

float ConvertType::byteToFloat(char *x)
{
    Obj_f.t.b[0]=*(x+3);
    Obj_f.t.b[1]=*(x+2);
    Obj_f.t.b[2]=*(x+1);
    Obj_f.t.b[3]=*(x);

    return Obj_f.c;
}

double ConvertType::byteToDouble(char *x)
{
    Obj_d.t.b[0]=*(x+7);
    Obj_d.t.b[1]=*(x+6);
    Obj_d.t.b[2]=*(x+5);
    Obj_d.t.b[3]=*(x+4);
    Obj_d.t.b[4]=*(x+3);
    Obj_d.t.b[5]=*(x+2);
    Obj_d.t.b[6]=*(x+1);
    Obj_d.t.b[7]=*(x);
    return Obj_d.c;
}

void ConvertType::floatToByte(float in, char *x)
{
    Obj_f.c = in;

    x[0]=Obj_f.t.b[3];
    x[1]=Obj_f.t.b[2];
    x[2]=Obj_f.t.b[1];
    x[3]=Obj_f.t.b[0];

}

void ConvertType::uIntToByte(unsigned int in, char *x)
{
    Obj_INT.c = in;

    x[0]=Obj_INT.t.b[3];
    x[1]=Obj_INT.t.b[2];
    x[2]=Obj_INT.t.b[1];
    x[3]=Obj_INT.t.b[0];
}

void ConvertType::uShortToByte(unsigned short in, char *x)
{
    x[0]=((in>>8)&0xff);
    x[1]=(in&0xff);
}

// 将两位16进制的字符串转换为一个字符
char ConvertType::hexToChar(const std::string& hex) {
    unsigned int intValue;             // 声明一个unsigned int变量，用于存储转换后的整数值
    std::stringstream ss;              // 声明一个字符串流
    ss << std::hex << hex;             // 将16进制标志和输入的hex字符串放入字符串流
    ss >> intValue;                    // 从字符串流中读取整数
    return static_cast<char>(intValue); // 将整数转换为字符并返回
}

// 将一个包含冒号分隔的16进制值的字符串转换为一个字符数组
char* ConvertType::hexStrToCharArray(const std::string& inputStr) {
    std::string input = inputStr;  // 创建一个输入字符串的副本，以便我们可以修改它
    std::vector<char> dataVector;

    size_t pos = 0;
    std::string token;
    while ((pos = input.find(":")) != std::string::npos) {
        token = input.substr(0, pos);
        dataVector.push_back(hexToChar(token));
        input.erase(0, pos + 1);
    }
    dataVector.push_back(hexToChar(input));   // 处理字符串中剩下的最后一个16进制值

    char* data = new char[dataVector.size()];       // 需要在适当的地方使用delte[]释放返回的字符数组内存
    std::copy(dataVector.begin(), dataVector.end(), data);

    return data;  // 返回字符数组
}



ConvertType::~ConvertType()
{

}
