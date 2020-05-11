/**
 * Project: Simple MotionV2 based drive controller for Carla
 *
 * @file </src/utility/CTimer.h>
 *
 * @author Prajankya Sonar - <prajankya@gmail.com>
 *
 * MIT License
 * Copyright (c) 2020 Prajankya Sonar
 */

#ifndef __CTIMER_H__
#define __CTIMER_H__

#include <chrono>
#include <ostream>
#include <type_traits>

class CTimer {
    typedef typename std::conditional<
        std::chrono::high_resolution_clock::is_steady,
        std::chrono::high_resolution_clock,
        std::chrono::steady_clock>::type TClockType;

    typedef std::chrono::milliseconds TMilliseconds;

   public:
    CTimer() { Reset(); }

    /****************************************/
    /****************************************/

    void Start() {
        m_StartTime = TClockType::now();
        m_EndTime = m_StartTime;
        m_bRunning = true;
    }

    /****************************************/
    /****************************************/

    void Stop() {
        m_EndTime = TClockType::now();
        m_bRunning = false;
    }

    /****************************************/
    /****************************************/

    void Reset() {
        m_StartTime = TClockType::now();
        m_EndTime = m_StartTime;
        m_bRunning = false;
    }

    /****************************************/
    /****************************************/

    TMilliseconds Elapsed() const {
        TClockType::time_point endTime;

        if (m_bRunning) {
            endTime = TClockType::now();
        } else {
            endTime = m_EndTime;
        }

        return std::chrono::duration_cast<TMilliseconds>(endTime - m_StartTime);
    }

    /****************************************/
    /****************************************/

    bool IsRunning() { return m_bRunning; }

    /****************************************/
    /****************************************/

    template <typename T, typename Traits>
    friend std::basic_ostream<T, Traits>& operator<<(
        std::basic_ostream<T, Traits>& out, const CTimer& timer) {
        return out << timer.Elapsed().count();
    }

   private:
    TClockType::time_point m_StartTime;
    TClockType::time_point m_EndTime;
    bool m_bRunning;
};

#endif