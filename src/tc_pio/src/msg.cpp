/// @brief all of msg/srv subscription callback and publisher functions will be programmed here.
#include "pio.hpp"

void PIO::dockingOrderReceivedCallback(tc_msgs::msg::AmrTask::SharedPtr msg)
{
    docking_process_.order = msg->order;
    docking_process_.feedback = msg->feedback;
    docking_process_.carrier = msg->carrier;
    docking_process_.result = msg->result;
    docking_process_.error_code = msg->error_code;

    logSys(TrShareEnum::log_seq, "[docking_order_received_callback] order: %d, feedback: %d, carrier: %d, result: %d, error_code: %d", 
                        docking_process_.order, docking_process_.feedback, docking_process_.carrier, docking_process_.result, docking_process_.error_code);

    publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, 0, 0); // feeback = order, return to main if recieved
}


void PIO::publishDockingReturn(int _order, int _feedback, int _carrier, int _result, int _error_code)
{
    auto msg = std::make_unique<tc_msgs::msg::AmrTask>();
    msg->order      = _order;
    msg->feedback   = _feedback;
    msg->carrier    = _carrier;
    msg->result     = _result;
    msg->error_code = _error_code;
    

    logSys(TrShareEnum::log_seq,  "[publish_docking_return] order: %d, feedback: %d, carrier: %d, result: %d, error_code: %d ", _order, _feedback, _carrier, _result, _error_code);
    docking_return_->publish(std::move(msg));
}

//IO Topic Monitoring
void PIO::ioSubscriptionCallback(tc_msgs::msg::IoModMsg::SharedPtr msg)
{	
    int nCreateIoNo = 0;
    try
        {   
            //1 block of 1 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword01 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 

                if((msg->wdword09 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;


                nCreateIoNo += 1; 
            }
            //2 block of 1 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword02 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 
        
                if((msg->wdword10 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1;
            }

            //3 block of 1 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword03 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 
            
            
                if((msg->wdword11 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;
                
                nCreateIoNo += 1;
            }
            //4 block of 1 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword04 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 
        
                if((msg->wdword12 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1;
            }
            //1 block of 3 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword05 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 

                if((msg->wdword13 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1;
            }
            //1 block of 4 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword06 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 
        
                if((msg->wdword14 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1;
            }
            //1 block of 5 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword07 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 

                if((msg->wdword15 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1; 
            } 
            //////////////////////////////////////////
            //1 block of 6 Module
            for(int nI = 0; nI < 8 ; nI++)
            {
                if((msg->wdword08 >> nI & 1) == 1) io_state_.kData[nCreateIoNo].state = 1;
                else  io_state_.kData[nCreateIoNo].state = 0; 

                if((msg->wdword16 >> nI & 1) == 1) ezio_output_->at(nCreateIoNo) = 1;
                else  ezio_output_->at(nCreateIoNo) = 0;

                nCreateIoNo += 1; 
            } 

            //add log
            array_pin_in = "[";
            array_pin_out = "[";
            for (int nI = 0; nI < 8 ; nI++)
            {
                if ((msg->wdword07  >> nI & 1) == 1) array_pin_in += "1 ";
                else array_pin_in += "0 ";

                if ((msg->wdword15  >> nI & 1) == 1) array_pin_out += "1 ";
                else array_pin_out += "0 ";
            }
            array_pin_in += "]";
            array_pin_out += "]";
        }
        catch(const std::exception& e)
        {
                std::cerr << e.what() << '\n';
        }  
		
}

// PIO cs_0, cs_1, cs_2 combination from acs 
void PIO::csValue(int carrier_type)
{
    if (carrier_type == CS_OFF)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {0, 0, 0}, 3);
    }
    else if (carrier_type == N_BASKET)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {0, 0, 0}, 3);
    }
    else if (carrier_type == C_BASKET)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {1, 0, 0}, 3);
    }
    else if (carrier_type == N_BASKET_2EA)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {0, 0, 1}, 3);
    }
    else if (carrier_type == C_BASKET_2EA)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {1, 0, 1}, 3);
    }
    else if (carrier_type == BOX)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {0, 1, 0}, 3);
    }
    else if (carrier_type == MAGZINE)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {1, 1, 0}, 3);
    }
    else if (carrier_type == START_CHARGE)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {0, 1, 1}, 3);
    }
    else if (carrier_type == END_CHARGE)
    {
        updateIoArray({OUT_PIO_IN02_CS0, OUT_PIO_IN03_CS1, OUT_PIO_IN04_CS2}, {1, 1, 1}, 3);
    }
    else
    {
        logSys(TrShareEnum::log_seq, "[csValue] Invalid number: %d", carrier_type);
    }
}

void PIO::subReturnPioIDCallback(tc_msgs::msg::Piohybrid::SharedPtr msg)
{
    pio_id_ = msg->id;
    pio_id_ret = msg->ret;
    RCLCPP_INFO(get_logger(), "subscription PIO ID: %d, return: %d ", pio_id_, pio_id_ret);
}

void PIO::publishingPioID(int nID)
{
    auto outMessage = std::make_unique<tc_msgs::msg::Piohybrid>();
    outMessage->id = nID;
    outMessage->ret = 0;
    pub_pio_id_->publish(std::move(outMessage));

    RCLCPP_INFO(get_logger(), "publishing PIO ID: %d", nID);
}

void PIO::liftCallback(tc_msgs::msg::Lift::SharedPtr msg)
{
    lift_request_ = msg->request;
    lift_ret_ = msg->ret;
    RCLCPP_INFO(get_logger(), "subscription Lift request: %d, return: %d ", lift_request_, lift_ret_);
}

void PIO::liftPublishing(int req)
{
    auto outMessage = std::make_unique<tc_msgs::msg::Lift>();
    outMessage->request = req;
    outMessage->ret = 0;
    lift_pub_->publish(std::move(outMessage));

    RCLCPP_INFO(get_logger(), "publishing lift request: %d", req);
}