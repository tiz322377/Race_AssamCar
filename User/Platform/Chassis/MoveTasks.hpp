//
// Created by Administrator on 24-9-7.
//

#ifndef LOGISTICSVEHICLE_MOVETASKS_HPP
#define LOGISTICSVEHICLE_MOVETASKS_HPP

#include "ChassisU.hpp"

namespace Platform::Chassis {

    struct ChassisTask {
        MoveDirection Direction = Forward;
        float Distance = 0.0f;
    };

    template<unsigned int _max>
    class MoveTasks {
        ChassisTask tasks[_max];
        unsigned int used = 0;
        unsigned int current = 0;
        using moveFunc = void(*)(MoveDirection,float);
        moveFunc move = nullptr;
        ChassisTask last{};
    public:
        explicit MoveTasks(moveFunc _move) : move(_move) {};
        MoveTasks() = default;
        void AddTasks(ChassisTask _task) {
            if (used < _max) {
                tasks[used] = _task;
                used++;
            }
        };
        void RunTask() {
            if (used > current) {
                last = tasks[current];
                move(tasks[current].Direction,tasks[current].Distance);
                current++;
            }
        }
        void Clear() {
            used = 0;
            current = 0;
        }
        void DeleteFirst() {
            if (used > current) {
                current++;
            }
        }
        void DeleteLast() {
            if (used > 0) {
                used--;
            }
        }
        bool IsEmpty () {
            return used == current;
        }
        void SetMoveFunc(moveFunc _move) {
            move = _move;
        }
        ChassisTask GetLast() {
            return last;
        }
    };

} // Platform

#endif //LOGISTICSVEHICLE_MOVETASKS_HPP
