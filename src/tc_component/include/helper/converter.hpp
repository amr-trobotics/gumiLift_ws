#ifndef CONVERTER_HPP
#define CONVERTER_HPP

class Converter
{
    public:
        static int bufferToIntLittleEndian(char* buffer)
        {
            int num = (unsigned char)buffer[0]
                    | (unsigned char)buffer[1] << 8
                    | (unsigned char)buffer[2] << 16
                    | (unsigned char)buffer[3] << 24;

            return num;
        }
        static int bufferToIntBigEndian(char* buffer)
        {
            int num = (unsigned char)buffer[3]
                    | (unsigned char)buffer[2] << 8
                    | (unsigned char)buffer[1] << 16
                    | (unsigned char)buffer[0] << 24;

            return num;
        }

        static int twoBytesToIntLittleEndian(char first, char second)
        {
            int num = (unsigned char)first | (unsigned char)second << 8;
            return num;
        }

        static void intTo2BytesLittleEndian(int value, std::vector<char>& bytes)
        {
            bytes.resize(2);
            bytes[0] = static_cast<char>(value & 0xFF);        
            bytes[1] = static_cast<char>((value >> 8) & 0xFF);
        }

        static char* intTo2BytesBigEndian(int value)
        {
            char* bytes = new char[2] {};
            bytes[0] = static_cast<char>((value >> 8) & 0xFF); 
            bytes[1] = static_cast<char>(value & 0xFF);        

            return bytes;
        }
        
        static void intTo4BytesLittleEndian(int value, std::vector<char>& bytes)
        {
            bytes.resize(4);
            bytes[0] = static_cast<char>(value & 0xFF);        // least significant byte
            bytes[1] = static_cast<char>((value >> 8) & 0xFF); // second least significant byte
            bytes[2] = static_cast<char>((value >> 16) & 0xFF);// second most significant byte
            bytes[3] = static_cast<char>((value >> 24) & 0xFF);// most significant byte
        }

        static char* intTo4Bytes(int n)
        {
            char* bytes = new char[4] {};

            bytes[0] = (n >> 24) & 0xFF;
            bytes[1] = (n >> 16) & 0xFF;
            bytes[2] = (n >> 8) & 0xFF;
            bytes[3] = n & 0xFF;

            return bytes;
        }

        static long long bufferToLongLong(char* data)
        {
             return *reinterpret_cast<const long long*>(data);
        }
};

#endif