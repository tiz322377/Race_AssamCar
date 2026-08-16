//
// Created by Administrator on 24-10-12.
//

#ifndef LV_Reserve_MECANUMU_HPP
#define LV_Reserve_MECANUMU_HPP

namespace Platform::Chassis {

    enum MecanumType {
        StepBus,
        CANBus,
        Steps,
        SerialBus,
        RS485Bus
    };

    template<MecanumType _type>
    class MecanumChassis;



}

#endif //LV_Reserve_MECANUMU_HPP
