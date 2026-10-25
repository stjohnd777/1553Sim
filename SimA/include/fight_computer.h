//
// Created by Daniel St. John St. John on 10/24/26.
//

#pragma once

class FightComputer {
    enum SENSOR {
        IMU,CAMS
    };
    enum COMMAND {

    };
    enum HZ {
        _1HZ,_10HZ,_100HZ
    };
public:
    FightComputer();

    void cycle(HZ,SENSOR);
    void cycle(HZ,COMMAND);

};