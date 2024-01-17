/// @brief charging process is a sequence of PIO communication btw AMR_PIO and Device_PIO
#include "pio.hpp"

/// @brief Do charging PIO communication
/// @param task 
/// @return 
int PIO::pioChargingCommProcess(int task)
{
    int seqnm = pio_charging_comm_process_seq_;
    int ret_result;

    switch(seqnm)
    {   case 0: 
            updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
            timeout_ =  getTickCount(); 
            logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] [Start] seqnm: %d", seqnm);
            seqnm = 100;
        break;

        case 100:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                publishingPioID(PIO_ID_CHARGER); // set PIO ID
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
                    logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] PIO ID = %d, set successfully", pio_id_);
                    pio_id_ = 0;
                    pio_id_ret = 0;
                    seqnm = 120;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::PIO_AMR_GO_OFF;
                    timeout_ =  getTickCount(); 
                    logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioChargingCommProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    return RETURN_ERR;   
                }   
            }
             
        break;

        case 120:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                // PIO_GO AMR and Equipment must match ID
                if (io_state_.kData[IN_PIO_GO].state == 1) 
                {
                    csValue(START_CHARGE);
                    updateIoModule(OUT_PIO_IN01_VALID,1);
                    timeout_ =  getTickCount(); 
                    logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] task Charing: %d, seqnm: %d", task, seqnm);
                    seqnm = 200;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::PIO_AMR_GO_OFF;
                    logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioChargingCommProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    return RETURN_ERR;   
                } 
            } 
               
        break;

        case 200:
            if (io_state_.kData[IN_PIO_OUT06].state == 1)
            {
                timeout_ =  getTickCount(); 
                logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] task Charing: %d, seqnm: %d", task, seqnm);
                // updateIoModule(CHARGER_MC_ON,1); // LIFT AGV doesn't have MC
                seqnm = 300;
            }
            else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
            {
                timeout_ =  getTickCount();
                error_code_ = Error_Code::TIMEOUT_CHARGING_PROCESS;
                logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] ERR_PIO_COMMUNICATION_TIMEOUT(s): %d, seqnm: %d", ERR_PIO_COMMUNICATION_TIMEOUT/1000, seqnm);
                logSys(TrShareEnum::log_err, "[pioChargingCommProcess] error_code = %d, ERR_PIO_COMMUNICATION_TIMEOUT(s): %d, seqnm: %d", error_code_, ERR_PIO_COMMUNICATION_TIMEOUT/1000, seqnm);
                seqnm = CASE_ERROR;
            }
        break;

        case 300:
            csValue(CS_OFF);
            updateIoModule(OUT_PIO_IN01_VALID,0);
            publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
            timeout_ =  getTickCount(); 
            logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] task: %d, seqnm: %d", task, seqnm);
            seqnm = CASE_SUCCESS;
        break;

        case CASE_ERROR:
            ret_result = RETURN_ERR;
            logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] [ERROR] seqnm: %d", seqnm);
            seqnm = CASE_END;
        break;

        case CASE_SUCCESS:
            ret_result = RETURN_SUCCESS;
            logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] [success] seqnm: %d",seqnm);
            seqnm = CASE_END;
        break;
        
        case CASE_END:
            
        break;

    }

    pio_charging_comm_process_seq_ = seqnm;
    return ret_result;
}

int PIO::pioChargingOffProcess(int task)
{
    int seqnm = pio_charging_off_process_seq_;
    int ret_result;
    switch(seqnm)
    {   
        case 0: 
            updateIoModule(OUT_PIO_SELECT,1); // Turn on PIO SELECT
            timeout_ =  getTickCount(); 
            logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] [Start] seqnm: %d", seqnm);
            seqnm = 100;
        break;

        case 100:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                publishingPioID(PIO_ID_CHARGER); // set PIO ID
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
                    logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] PIO ID = %d, set successfully", pio_id_);
                    pio_id_ = 0;
                    pio_id_ret = 0;
                    seqnm = 120;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::PIO_AMR_GO_OFF;
                    logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioChargingOffProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    return RETURN_ERR;   
                } 
            }           
        break;

        case 120:
            if(getTimeOut((long)timeout_, PIO_SELECT_ON_TIME) == true)
            {
                // PIO_GO AMR and Equipment must match ID
                if (io_state_.kData[IN_PIO_GO].state == 1) 
                {
                    csValue(END_CHARGE);
                    updateIoModule(OUT_PIO_IN01_VALID,1);
                    timeout_ =  getTickCount();
                    logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] seqnm: %d", seqnm);
                    seqnm = 200;
                }
                else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
                {
                    error_code_ = Error_Code::PIO_AMR_GO_OFF;
                    logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", io_state_.kData[IN_PIO_GO].state);
                    logSys(TrShareEnum::log_err, "[pioChargingOffProcess] error_code = %d, AMR and Equipment PIOs are not alligned or Incorrect reception of optical signal, IN_PIO_GO = %d", error_code_, io_state_.kData[IN_PIO_GO].state);
                    return RETURN_ERR;   
                } 
            }
               
        break;

        case 200:
            if (io_state_.kData[IN_PIO_OUT07].state == 1 )
            {
                timeout_ =  getTickCount(); 
                logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] task Loading: %d, seqnm: %d", task, seqnm);
                seqnm = 300;
            }
            else if(getTimeOut((long)timeout_, ERR_PIO_COMMUNICATION_TIMEOUT) == true)
            {
                timeout_ =  getTickCount(); 
                error_code_ = Error_Code::TIMEOUT_CHARGING_OFF_PROCESS;
                logSys(TrShareEnum::log_err, "[pioChargingOffProcess] ERR_PIO_COMMUNICATION_TIMEOUT(s): %d, seqnm: %d", ERR_PIO_COMMUNICATION_TIMEOUT/1000, seqnm);
                logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] error_code = %d, ERR_PIO_COMMUNICATION_TIMEOUT(s): %d, seqnm: %d", error_code_, ERR_PIO_COMMUNICATION_TIMEOUT/1000, seqnm);
                seqnm = CASE_ERROR;
            }
        break;

        case 300:
            csValue(CS_OFF);
            updateIoModule(OUT_PIO_IN01_VALID,0);
            // updateIoModule(CHARGER_MC_ON,0);
            publishDockingReturn(docking_process_.order, docking_process_.order, docking_process_.carrier, RETURN_SUCCESS, NO_ERROR);
            timeout_ =  getTickCount(); 
            logSys(TrShareEnum::log_seq, "[pioChargingCommProcess] task: %d, seqnm: %d", task, seqnm);
            seqnm = CASE_SUCCESS;
        break;

        case CASE_ERROR:
            ret_result = RETURN_ERR;
            logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] [ERROR] seqnm: %d", seqnm);
            seqnm = CASE_END;
        break;

        case CASE_SUCCESS:
            ret_result = RETURN_SUCCESS;
            logSys(TrShareEnum::log_seq, "[pioChargingOffProcess] [success] seqnm: %d",seqnm);
            seqnm = CASE_END;
        break;
        
        case CASE_END:
            
        break;
    }

    pio_charging_off_process_seq_ = seqnm;
    return ret_result;
}

