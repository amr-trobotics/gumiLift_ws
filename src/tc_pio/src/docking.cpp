/// @brief docking process is a sequence of PIO communication btw AMR_PIO and Device_PIO
#include "pio.hpp"

int PIO::pioDockingCommProcess(int task, int carrier_type, int speed)
{
    int seqnm = pio_docking_comm_process_seq_;
    int ret_result;
    int pio_id;
    
    switch (seqnm)
    {
    case 0: 
            updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
            timeout_ =  getTickCount(); 
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] [Start] seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 100;
        break; 

        case 100:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                pio_id = taskToPioId(task); // convert task to pio id
                publishingPioID(pio_id); // set PIO ID
                timeout_ =  getTickCount(); 
                seqnm = 110;
            }   
        break;

        case 110:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                if (pio_id_ret == RETURN_SUCCESS) // feedback msg from hybrid PIO ID set
                {
                    updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
                    timeout_ =  getTickCount(); 
                    logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] PIO ID = %d, set successfully", pio_id_);
                    logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                    pio_id_ = 0;
                    pio_id_ret = 0;
                    seqnm = 120;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::PIO_AMR_GO_OFF;
                    timeout_ =  getTickCount(); 
                    logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioDockingCommProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                    seqnm = CASE_ERROR;  
                } 
            }
               
        break;

        case 120:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                // PIO_GO AMR and Equipment must match ID
                if (io_state_.kData[IN_PIO_GO].state == 1) 
                {
                    // csValue(START_CHARGE);
                    updateIoModule(OUT_PIO_IN01_VALID,1);
                    timeout_ =  getTickCount(); 
                    logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
                    logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Valid On, seqnm: %d", seqnm);
                    logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                    seqnm = 200;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::VALID_ON_FAIL;
                    logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioDockingCommProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                    seqnm = CASE_ERROR;   
                }
            }   
        break;

    case 200:
        if (io_state_.kData[IN_PIO_OUT02_UREQ].state == 1) // equipment side is Unloading(Docking1 or Docking3) or Airshower or Elevator
        {
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Receive U_REQ 1, seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task Loading: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 300;
        }
        else if (io_state_.kData[IN_PIO_OUT01_LREQ].state == 1) // equipment side is Loading
        {
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Receive L_REQ 1, seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task Unloading: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 390;
        }
        else if (getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
        {
            error_code_ = Error_Code::U_REQ_L_REQ_FAIL;
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_err, "[pioDockingCommProcess] error_code = %d, ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", error_code_, ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = CASE_ERROR;
        }
        else if(ezio_output_->at(OUT_PIO_IN01_VALID) != 1)
        {            
            updateIoModule(OUT_PIO_IN01_VALID,1);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Valid On again cause of last valid on fail. seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        }
        break;

    case 300: // check again for cart availability?
        if (task == Task::Docking1 || task == Task::Docking4)// aquring
        {
            if (IN_TRAY_FRONT_DETECTION == 1 || IN_TRAY_REAR_DETECTION == 1)
            {
                error_code_ = Error_Code::CARRIER_AVAIL_BUT_REQ_AQUIRE;
                logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] CARRIER_AVAIL_BUT_REQ_AQUIRE, seqnm: %d", seqnm);
                logSys(TrShareEnum::log_err, "[pioDockingCommProcess] error_code = %d, CARRIER_AVAIL_BUT_REQ_AQUIRE, seqnm: %d", error_code_, seqnm);
                logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                seqnm = CASE_ERROR;
            }
            else
            {
                logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
                logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                seqnm = 390;
            }
        }
        else
        {
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 390;
        }
        break;

    case 390:
        updateIoModule(OUT_PIO_IN05_TR_REQ, 1);
        timeout_ = getTickCount();
        logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] TR_REQ On, seqnm: %d", seqnm);
        logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
        logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        seqnm = 400;
        break;

    case 400:
        if (io_state_.kData[IN_PIO_OUT04_READY].state == 1)
        {
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Receive READY 1, seqnm: %d", seqnm);
            // add ACS verify
            updateIoModule(OUT_PIO_IN06_BUSY, 1);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Busy On, seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            for (int i=0; i<2; i++)
            {
                publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_BUSY_ON, NO_ERROR);
                logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] pub count: %d, task: %d, seqnm: %d", i, task, seqnm);
            }
            // now amr need to move
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR IS NOW MOVING >>>");
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            docking_process_.result = 0;
            seqnm = 410;
        }
        else if (getTimeOut((long)timeout_, AMR_MOVING_TIME) == true)
        {
            timeout_ = getTickCount();
            error_code_ = Error_Code::CANNOT_RECEIVE_READY;
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] READY is OFF, ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_err, "[pioDockingCommProcess] READY is OFF, error_code = %d, ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", error_code_, ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = CASE_ERROR;
        }
        else if(ezio_output_->at(OUT_PIO_IN05_TR_REQ) != 1)
        {            
            updateIoModule(OUT_PIO_IN05_TR_REQ,1);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] TR_REQ On again cause of last TR_REQ on fail. seqnm: %d", seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        }
        break;

    case 410:
        //now robot moving confirm from scenario
        if (docking_process_.result == RETURN_BUSY_ON) 
        {
            docking_process_.result = 0;
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR MOVING  >>>");
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 420;
        }
    break;
    
    case 420:
        // check amr moving completed?
        if (docking_process_.result == RETURN_SUCCESS)
        {
            docking_process_.result = 0;
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR MOVING COMPLETED >>>");
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 500;
        }
        // robot moving error, need to add button to reset here.
        if (docking_process_.result == RETURN_ERR)
        {
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] AMR MOVING ERROR >>>");
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);    
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        }
    break;

    case 500:
            updateIoModule(OUT_PIO_IN06_BUSY, 0);
            updateIoModule(OUT_PIO_IN07_COMPT, 1);
            updateIoModule(OUT_PIO_IN05_TR_REQ, 0);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Busy 0, Complete 1, TR_REQ 0 , seqnm: %d", seqnm);
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = 600;
    break;

    case 600:
        if (io_state_.kData[IN_PIO_OUT04_READY].state == 0)
        {
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Receive Ready 0, seqnm: %d", seqnm);
            updateIoModule(OUT_PIO_IN07_COMPT, 0);
            updateIoModule(OUT_PIO_IN01_VALID, 0);
            updateIoModule(OUT_PIO_IN02_CS0, 0);
            updateIoModule(OUT_PIO_IN03_CS1, 0);
            updateIoModule(OUT_PIO_IN04_CS2, 0);
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] Complete 0, Valid 0, CS 0, seqnm: %d", seqnm);
            
            timeout_ = getTickCount();
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] task: %d, seqnm: %d", task, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = CASE_SUCCESS;
        }
        else if (getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
        {
            timeout_ = getTickCount();
            error_code_ = Error_Code::TIMEOUT_PIO_DOCKING_COMM;
            logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_err, "[pioDockingCommProcess] error_code = %d, ERR_PIO_COMMUNICATION_TIMEOUT: %d(s), seqnm: %d", error_code_, ERR_PIO_COMMUNICATION_TIMEOUT / 1000, seqnm);
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
            seqnm = CASE_ERROR;
        }

        break;

    case CASE_ERROR:
        ret_result = RETURN_ERR;
        logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] [ERROR] seqnm: %d", seqnm);
        logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        // // reset PIO
        // updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
        // publishingPioID(PIO_RESET); // set PIO ID
        // updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
        // logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] [Reset] seqnm: %d",seqnm);
        seqnm = CASE_END;
        break;

    case CASE_SUCCESS:
        ret_result = RETURN_SUCCESS;
        logSys(TrShareEnum::log_seq, "[pioDockingCommProcess] [success] seqnm: %d", seqnm);  
        logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        // // reset PIO
        // updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
        // publishingPioID(PIO_RESET); // set PIO ID
        // updateIoModule(OUT_PIO_SELECT,0); // turn off PIO SELECT
        // logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] [Reset] seqnm: %d",seqnm);          
        seqnm = CASE_END;
        break;

    case CASE_END:
        
        break;
    }
    pio_docking_comm_process_seq_ = seqnm;
    return ret_result;
}



int PIO::taskToPioId(int task)
{
    if (task == 1)
    {
        return PIO_ID_DOCKING_1;
    }
    else if (task == 2)
    {
        return PIO_ID_DOCKING_2;
    }
    else if (task == 3)
    {
        return PIO_ID_DOCKING_3;
    }
    else if (task == 4)
    {
        return PIO_ID_DOCKING_4;
    }
    else if (task == 5)
    {
        return PIO_ID_AIRSHOWER_1_1;
    }
    else if (task == 6)
    {
        return PIO_ID_AIRSHOWER_1_2;
    }
    else if (task == 7)
    {
        return PIO_ID_AIRSHOWER_2_1;
    }
    else if (task == 8)
    {
        return PIO_ID_AIRSHOWER_2_2;
    }
    else if (task == 9)
    {
        return PIO_ID_CHARGER;
    }
    else if (task == 10)
    {
        return PIO_ID_CHARGER;
    }
    else if (task == 11)
    {
        return PIO_ID_ELV_1;
    }
    else if (task == 12)
    {
        return PIO_ID_ELV_2;
    }
    else if (task == 13)
    {
        return PIO_RESET;
    }
    else
    {
        logSys(TrShareEnum::log_err, "[taskToPioId] [ERROR] invalid task [1-12]: %d ", task);
        return 0;
    }
}