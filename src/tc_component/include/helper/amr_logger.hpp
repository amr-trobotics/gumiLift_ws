#ifndef AMR_LOGGER_HPP
#define AMR_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <boost/format.hpp>
#include <boost/preprocessor.hpp>
#include <time.h>


#include "thread_safety_queue.hpp"

#pragma region DEFINE_ENUM_TO_STRING

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \ 
    case elem : return BOOST_PP_STRINGIZE(elem); 
 
#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \ 
    enum name {                                                               \ 
        BOOST_PP_SEQ_ENUM(enumerators)                                        \ 
    };                                                                        \ 
                                                                              \ 
    inline char* ToString(name v)                                       \ 
    {                                                                         \ 
        switch (v)                                                            \ 
        {                                                                     \ 
            BOOST_PP_SEQ_FOR_EACH(                                            \ 
                X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \ 
                name,                                                         \ 
                enumerators                                                   \ 
            )                                                                 \ 
            default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \ 
        }                                                                     \ 
    }

#pragma endregion


namespace fs = std::filesystem;

/*
typedef enum
{
    LOG_ERR = 1,
    LOG_WARNING = 2,
    LOG_INFO = 3
}LogLevel;*/

DEFINE_ENUM_WITH_STRING_CONVERSIONS(LogType, (LOG_ACS)(LOG_TDRIVER)(LOG_IO)(LOG_SCENARIO)(LOG_GUI)(LOG_HPIO)(LOG_ALARM)(LOG_PING)(LOG_ACS_DEBUG))
DEFINE_ENUM_WITH_STRING_CONVERSIONS(LogLevel, (LOG_ERR)(LOG_WARNING)(LOG_INFO)) 


class AmrLogger
{
    private:
        AmrLogger() = default; 
        AmrLogger& operator=(const AmrLogger& ref);

        std::string home_path_ = std::getenv("HOME");
        std::string log_path_ = home_path_ + "/tcon/log/";

        //return struct tm that has current time data
        inline struct tm getTimeStructure()
        {
            auto now = std::chrono::system_clock::now();
            std::time_t time_now = std::chrono::system_clock::to_time_t(now);            
            return *localtime(&time_now);
        }

        ///Return date -> format : YYYYMMDD
        std::string getDate()
        {
            auto time_structure = getTimeStructure();            
            
            return (boost::format("%04d%02d%02d") 
                        % (time_structure.tm_year+1900) 
                            % (time_structure.tm_mon+1) 
                                % time_structure.tm_mday).str();
        }

        //Return time -> format : hh:mm:ss
        std::string getTimeStamp()
        {
            auto time_structure = getTimeStructure();            
            
            return (boost::format("%02d:%02d:%02d") 
                        % time_structure.tm_hour
                            % time_structure.tm_min 
                                % time_structure.tm_sec).str();
        }
        
        //Create log directory if it doesn't exist
        void checkDirectory(std::string dir_path)
        {         
            fs::path p(dir_path);

            if(fs::exists(p) == false)
            {
                std::cout << "create directory" << std::endl;
                fs::create_directories(p);
            }
        }


    public:        
        //return AmrLogger instance
        static AmrLogger& getInstance()
        {
            static AmrLogger instance;
            
            return instance;
        }

        //enqueue to log_queue_ 
        void logWrite(LogType type, LogLevel level, std::string msg)
        {
            std::string dir_path = log_path_ + getDate();
            checkDirectory(dir_path);

            std::string file_path = (boost::format("%s/[%s]%s.txt") % dir_path % getDate() % ToString(type)).str();

            std::ofstream write_file(file_path, std::ios_base::app);
            std::string log = (boost::format("[%s][%s] ") % getTimeStamp() % ToString(level)).str();

            std::cout << "[" << ToString(level) << "] " << msg << std::endl;
            
            log = log + " " + msg;
            if(write_file.is_open())
            {
                write_file << log << "\n";
            }

            write_file.close();


        }
};



#endif