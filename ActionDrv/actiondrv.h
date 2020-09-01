#include "mbed.h"

class actionDrv
{
public:    
    actionDrv(int _id, CAN* _can1);
    void SetOperationalMode();
    void Enable();
    void Configvelocity(int acc, int dec);
    void SetVelocity(int vel);
    void SetVelocity_mod(int vel);  //added
    void send(char* msg);
    void send_mod(char* msg);  //added
    void SetHeartBeat(int interval); //added
    void stop();
    private:
    Mutex actionDrv_mutex;
    int id, SDOid;
    CAN* can1;
   
    
};
