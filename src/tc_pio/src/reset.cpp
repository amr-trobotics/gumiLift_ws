#include "pio.hpp"

void PIO::ioReset()
{
    for (int i = 0; i<64; i++)
    {
        updateIoModule(i, 0);
    }
    logSys(TrShareEnum::log_seq, "[ioReset] Done");
    return;
}


void PIO::pioReset()
{
    updateIoModule(OUT_PIO_IN02_CS0,0); 
    updateIoModule(OUT_PIO_IN03_CS1,0); 
    updateIoModule(OUT_PIO_IN04_CS2,0); 
    updateIoModule(OUT_PIO_IN01_VALID,0);
    updateIoModule(OUT_PIO_IN05_TR_REQ,0);
    updateIoModule(OUT_PIO_IN06_BUSY,0);
    updateIoModule(OUT_PIO_IN07_COMPT,0);
    updateIoModule(OUT_PIO_IN08_CONT,0);
    
    logSys(TrShareEnum::log_seq, "[pioReset] Done");
}

void PIO::globalVariablesReset()
{
    memset(&docking_process_, 0, sizeof(DockingProcess));
    pio_id_ = 0;
    pio_id_ret = 0;

    pio_docking_comm_process_seq_ = 0;
    pio_charging_comm_process_seq_ = 0;
    pio_charging_off_process_seq_ = 0;

    logSys(TrShareEnum::log_seq, "[globalVariablesReset] Done");
}
