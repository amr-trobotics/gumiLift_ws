#include <iostream>
#include "data_type/acs_data_type.hpp"

class Validation
{
    public:

        bool validateMoveCommand(std::string job_id, std::vector<std::string> pathlist)
        {  
            bool is_different = false;

            std::cout << "job_id" << job_id << std::endl; 
            
            if(via_node_.size() != pathlist.size() || job_id_ != job_id)
            {
                is_different = true;
            }
            else if(via_node_.size() == pathlist.size())
            {
                int size = pathlist.size();
                for(int i=0; i<size; i++)
                {
                    if(via_node_[i] != pathlist[i])
                    {
                        is_different = true;
                    }
                }

                if(is_different == true)
                {
                    return true;
                }
            }

            job_id_ = job_id;
            via_node_ = pathlist;
            return is_different;
        }

        void clearValidation()
        {
            job_id_ = "";
            via_node_.clear();
        }


        int action_;
        std::string job_id_;
        std::vector<std::string> via_node_;
};