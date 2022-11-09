#ifndef _PID_STILL_H
#define _PID_STILL_H

#include "src/AutoPID/AutoPID.h"
#include "sensors.h"

enum _state{WARM_UP=0,INITIAL_REFLUX,TAKING_FORES,REFLUX_2HEADS,TAKING_HEADS};

class pid_still {
    public:
        pid_still(){};
        ~pid_still(){};
        void AddTempSensors();
        void start();
        _state getState(){return state;}
    private:
        //functions
        
        bool auto_power;
        float curr_temp;
        AutoPID* pid;
        unsigned long process_time; //
        _state state;
        
};

#endif
