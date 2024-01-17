################################################################################
# Messages ACS → AMR order
# order: 1 -> Docking station 1, PIO ID = 10 // aquire  1F
# order: 2 -> Docking station 2, PIO ID = 11 // deposit 1F
# order: 3 -> Docking station 3, PIO ID = 12 // deposit 2F
# order: 4 -> Docking station 4, PIO ID = 13 // aquire 2F
# order: 5 -> Airshower 1-1, PIO ID = 40 , 1F in between 
# order: 6 -> Airshower 1-2, PIO ID = 41 , 
# order: 7 -> Airshower 2-1, PIO ID = 42 , 2F in between
# order: 8 -> Airshower 2-2, PIO ID = 43 , 2F inside
# order: 9 -> Charge, PIO ID = 60
# order: 10 -> Charge OFF, PIO ID = 60
# order: 11 -> Elevator 1F, PIO ID = 80 2F
# order: 12 -> Elevator 2F, PIO ID = 81 1F
# order: 13 -> PIO Reset, PIO ID = 15 // reset PIO ID
# order: 14 -> PIO stop, clear PIO, keep PIO ID

#carrier: carrier type
#carrier : 1 - Loading
#carrier : 2 - Unloading

 
# ---
# feedback: receive order and feedback exact order
# result: return status: 1->RETURN_SUCCESS, 2->RETURN_ERR
# error_code: if error then send error_code


# 1st -> send to order, carrier type
# 2nd -> check feedback == order // Runing Task
# 3rd -> result init value == 0 / 1 SUCCESS 2 ERROR 3 RUNNING or BUSY
# 4th -> if ERROR check error_code

################################################################################

int32 order
int32 carrier 
int32 feedback
int32 result
int32 error_code

