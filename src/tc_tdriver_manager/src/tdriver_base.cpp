#include "tdriver_base.hpp"

TDriverBaseClient::TDriverBaseClient()
{
    std::cout << "TDriverBaseClient()" << std::endl;
    quit_ = false;
    receive_queue_ = std::make_shared<ThreadSafeQueue<char*>>();
    conn_thread_ =  std::thread(&TDriverBaseClient::connect, this); 
    data_receive_thread_ = std::thread(&TDriverBaseClient::getClientMessage, this);
    std::cout << "~TDriverBaseClient()" << std::endl;
}

TDriverBaseClient::~TDriverBaseClient()
{
    quit_ = true;
}


// Functions
void TDriverBaseClient::init(std::shared_ptr<ThreadSafeQueue<std::string>> queue, uint16_t api_list)
{   
    parent_queue_ = queue;
    api_lists = api_list;
    data_send_thread_ = std::thread(&TDriverBaseClient::setClientMessage, this); 
}

void TDriverBaseClient::init(std::shared_ptr<ThreadSafeQueue<std::string>> queue, std::vector<uint16_t> api_list)
{
    parent_queue_ = queue;
    int size = api_list.size();
    for(int i=0; i<size; i++)
    {
        api_list_.push_back(api_list[i]);
    }
    data_send_thread_ = std::thread(&TDriverBaseClient::setClientMessage, this);     
}

void TDriverBaseClient::connect()
{
    while(!quit_)
    {
        boost::asio::io_context io_context;
        //std::string ip = "10.0.5.91";
        std::string ip = "192.168.192.5";
        std::string port = "19204"; // navigation
        client_ = new AsyncTcpClient(io_context, ip, port, receive_queue_);
        
        client_->connect();  
        std::cout << "try to connect" << std::endl; 
    
        io_context.run();
        std::cout << "connect end" << std::endl; 
    }
}

void TDriverBaseClient::getClientMessage()
{
    while(!quit_)
    {
        if(receive_queue_->count() > 0)
        {
            
            parsingClientMessage(receive_queue_->front());
            receive_queue_->pop();
        }
        std::this_thread::sleep_for(30ms);
    }
}

void TDriverBaseClient::setClientMessage()
{

    std::this_thread::sleep_for(5s);
    while(!quit_)
    {
        int size = api_list_.size();
        if(size > 0)
        {
            for(int i=0; i<size; i++)
            {
                auto header = makeHeader(api_list_[i],0);
                
                int data_size = sizeof(header);    
                unsigned char* snd_data = new unsigned char[data_size];

                
                memcpy(snd_data, &header, sizeof(header));

                client_->write(snd_data, data_size);   

                std::this_thread::sleep_for(100ms);           
            }
        }
        else
        {
            auto header = makeHeader(api_lists,0);
            
            int data_size = sizeof(header);    
            unsigned char* snd_data = new unsigned char[data_size];

            
            memcpy(snd_data, &header, sizeof(header));

            client_->write(snd_data, data_size);
            
            std::this_thread::sleep_for(300ms);
        }
        

    }
}

void TDriverBaseClient::parsingClientMessage(char *data)
{
    unsigned char string_length[4];
    
    for(int i=4; i<8; i++)
    {
        int j = i-4;
        string_length[j] = data[i];
    }
    
    int length = 0;
    length = string_length[3];
    length |= string_length[2] << 8;
    length |= string_length[1] << 16;
    length |= string_length[0] << 24;
    //std::cout << "length : " << length << std::endl;

    if(length > 10000)
        return;
    
    unsigned char return_code[2] = {};
    return_code[0] = data[8];
    return_code[1] = data[9];

    int code = 0;
    code = return_code[1];
    code |= return_code[0] << 8;
    //std::cout << "code : " << code << std::endl;
    
    std::string return_data(&data[16], length);
    //std::cout << "return data : " << return_data << std::endl;

    try
    {    
        json j = json::parse(return_data);
        std::string str_code = std::to_string(code);
        j["DATA_TYPE"] = str_code;
        
        //logWrite(LOG_INFO, j.dump());
        parent_queue_->push(j.dump());        
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

}

TDriverProtocolHeader TDriverBaseClient::makeHeader(uint16_t api_type, int data_length)
{
    auto header = TDriverProtocolHeader(); 
    header.sync_ = 0x5A;
    header.version_ = 0x01;
    header.number_ = htons(0x01);
    header.length_ = htonl(data_length);
    header.type_ = htons(api_type);

    return header;
}

void TDriverBaseClient::logWrite(LogLevel level, std::string msg)
{
    auto logger = AmrLogger::getInstance();
    logger.logWrite(LOG_TDRIVER,level, msg);
}

void TDriverBaseClient::stop()
{
    quit_ = true;

    client_->disconnect();

    if(conn_thread_.joinable())
    {
        conn_thread_.join();
    }

    if(data_send_thread_.joinable())
    {
        data_send_thread_.join();
    }

    if(data_receive_thread_.joinable())
    {
        data_receive_thread_.join();
    }

}