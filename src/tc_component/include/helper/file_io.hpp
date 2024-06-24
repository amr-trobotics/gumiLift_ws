#ifndef FILE_IO_HPP
#define FILE_IO_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>


#include "nlohmann/json.hpp"

using nlohmann::json;

class File_IO
{
    private:
        std::string home_path_ = std::getenv("HOME");
        std::string setting_path_ = home_path_ + "/tcon/setting/";
        std::string map_path_ = home_path_ + "/tcon/map/";

        std::vector<std::string> split(std::string msg, char delimiter)
        {
            std::stringstream ss(msg);
            std::vector<std::string> split_string;
            std::string data;
            while(getline(ss,data,delimiter))
            {
                split_string.push_back(data);
            }

            return split_string;
        }
        
        std::string remove_carriage_return(const std::string &input) 
        {
            std::string result = input;
            result.erase(std::remove(result.begin(), result.end(), '\r'), result.end());
            return result;
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
        File_IO() {};
        ~File_IO() {};

        void writeConifgFile(json config)
        {
            std::cout << "write config start" << std::endl;
            std::string config_path = setting_path_ + "config.info"; 
            checkDirectory(setting_path_);
            std::ofstream outFile(config_path); 

            if (outFile.is_open())
            {                 
                for(const auto& item : config.items())
                {
                    std::string key_string = item.key();
                    std::string value_string;
                    if(key_string == "start_node_1F" || key_string == "wait_node_1F" || key_string == "start_node_2F" ||
                       key_string == "wait_node_2F") //int
                    {
                        auto config_value = item.value().get<int>();
                        value_string = std::to_string(config_value);
                    }
                    else
                    {
                        continue;
                    }

                    std::string config = (boost::format("%s,%s") % key_string % value_string ).str() + '\r';

                    outFile << config; // write to the file using the stream operator
                }
            } 
            else 
            {
                std::cout << "Error opening file." << std::endl;
            }

            outFile.close(); // close the file
        }

        std::string readConfigFile()
        {
            std::cout << "start read" << std::endl;
            std::string config_path = setting_path_ + "config.info";

            std::ifstream read_file(config_path);

            json j;

            if(read_file.is_open())
            { 
                int count = 0;
                std::string line;
                while(getline(read_file, line, '\r'))
                {                    
                    auto data = split(line,',');                    
                    j[data[0]] = remove_carriage_return(data[1]);                    
                }
            }

            read_file.close();

            return j.dump();
        }

        std::string readMapScaleFile()
        {
            std::string map_path = setting_path_ + "map_scale.info";

            std::ifstream read_file(map_path);

            json scale;

            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    auto data = split(line,',');
                    
                    scale["scale_x"] = std::stoi(remove_carriage_return(data[0]));
                    scale["scale_y"] = std::stoi(remove_carriage_return(data[1]));
                }
            }

            read_file.close();

            return scale.dump();
        }

        int readMapVersionFile()
        {
            std::string file_path = setting_path_ + "map_version.info";

            std::ifstream read_file(file_path);
            int version = 0;

            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    version = std::stoi(line);                    
                }
            }

            read_file.close();

            return version;
        

        }
        
        std::string readNodeFile()
        {
            json nodes;

            std::string map_path = map_path_ + "node.info";


            std::ifstream read_file(map_path);
            int i = 0;
            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    auto data = split(line,',');
                    
                    json node;
                    node["x"] = std::stoi(remove_carriage_return(data[1]));
                    node["y"] = std::stoi(remove_carriage_return(data[2]));
                    
                    nodes[remove_carriage_return(data[0])] = node;
                    std::string check = std::to_string(i);
                    //std::cout << "read node file " + check << std::endl;
                    i++;
                    //j.push_back(node);
                }
            }

            read_file.close();

            return nodes.dump();
        }

        std::string readMapFile()
        {
            std::string map_path = setting_path_ + "map.info";

            std::ifstream read_file(map_path);

            json j;

            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    auto data = split(line,',');
                    j[data[0]] = std::stoi(data[1]);
                }
            }

            read_file.close();

            return j.dump();
        }
       
        std::string readNetworkFile()
        {
            std::string network_path = setting_path_ + "network.info"; 
            
            std::ifstream read_file(network_path);

            json j;

            if(read_file.is_open())
            {
                std::string line;
                while(getline(read_file, line))
                {
                }
            }

            read_file.close();   
            return "done";
        }

        void writeCurrentPosFile(int node, int x, int y, int angle)
        {
            std::string pos_path = setting_path_ + "last_position.info"; 
            checkDirectory(setting_path_);
            std::ofstream outFile(pos_path); 

            if (outFile.is_open())
            { 
                std::string pos = (boost::format("%d,%d,%d,%d") % node % x % y % angle).str();

                outFile << pos; // write to the file using the stream operator
                
                std::cout << "File overwritten successfully." << std::endl;
            } 
            else 
            {
                std::cout << "Error opening file." << std::endl;
            }

            outFile.close(); // close the file
        }

        std::vector<int> readCurrentPosFile()
        {
            std::string pos_path = setting_path_ + "last_position.info"; 
            std::vector<int> cur_position;

            std::ifstream read_file(pos_path);

            

            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    auto data = split(line,',');
                    cur_position.push_back(std::stoi(data[0])); // node
                    cur_position.push_back(std::stoi(data[1])); // x
                    cur_position.push_back(std::stoi(data[2])); // y
                    cur_position.push_back(std::stoi(data[3])); //angle
                }
            }

            read_file.close();

            return cur_position;
        }

        std::map<int,int> getAlarmCode()
        {
            std::string alarm_path = setting_path_ + "alarm_code.info"; 
            std::map<int,int> alarm_code;

            std::ifstream read_file(alarm_path);

            if(read_file.is_open())
            { 
                std::string line;
                while(getline(read_file, line))
                {
                    auto code = std::stoi(line);
                    alarm_code[code] = 0;
                }
            }

            read_file.close();

            return alarm_code;            
        }
        
        std::string getMapInfo()
        {
            json maps;
            int count = 0;

            std::ifstream read_file(map_path_ + "map.info");                                

            if(read_file.is_open())
            {
                std::string line;
                while(getline(read_file, line))
                {
                    json map_files;
                    count++;
                    auto data = split(line,',');
                    map_files["map_name"] = data[0];
                    map_files["map_version"] = data[1];

                    maps[data[0]] = map_files;
                }
            }
            
            read_file.close();

            maps["map_count"] = count;
            
            return maps.dump();
        }                    
                

        
};

#endif