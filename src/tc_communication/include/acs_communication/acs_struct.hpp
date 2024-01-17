#ifndef ACS_STRUCT_HPP
#define ACS_STRUCT_HPP

#include <iostream>
#include <vector>

#pragma pack(1)
struct AmrReport
{
    public:
        char stx_;
        char ver_;
        char msg_type_; 
        char seq_; 
        int data_size_; 			
};

#pragma pack(1)
struct AmrCarrier
{
    public:
        char slot_id_;
        char mat_id_len_[2];
        std::string mat_id_;
        char mat_weight_;
        double mat_direction_;
};

#pragma pack(1)
struct AmrInitReport
{
    public:
        AmrReport header_;
        double xpos_;
        double ypos_;
        char map_name_len_[2];
        std::string map_name_;
        double angle_;
        double battery_;
        char current_node_len_[2];
        std::string current_node_;
        char lift_state_;
        char num_of_carrier_;
        std::vector<AmrCarrier> carrier_list_;
};


#pragma pack(1)
struct AmrStatusReport
{
    public:
        //AmrReport report_;
        double xpos_;
        double ypos_;
        char map_name_len_[2];
        std::string map_name_;
        double angle_;
        double velocity_;
        double battery_;
        char current_node_len_[2];
        std::string current_node_;
        char next_node_len_[2];
        std::string next_node_;
};


#endif