// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#ifndef INCLUDE_STATE_MACHINE_HPP_
#define INCLUDE_STATE_MACHINE_HPP_

#include <map>
#include <vector>
#include <utility>
#include <Arduino.h>

//#define DEBUG
#ifdef DEBUG
#define func_print Serial.println(__func__)
#else
#define func_print
#endif

template <uint8_t PIN> void _interrupt_state();

class StateMachine;
typedef void* (StateMachine::*state_machine_state)();

extern std::map<uint8_t, std::pair<StateMachine*, state_machine_state>> interrupt_states;
extern std::map<StateMachine*, std::vector<uint8_t>> instance_interrupts;

#define cast_state(x) reinterpret_cast<state_machine_state>(&StateMachine::x)
#define set_state(x) this->current_state_function = cast_state(x)

class StateMachine {
public:
    void update();

    void update_until_done();

    bool is_done();

    template <uint8_t PIN> void run_state_on_interrupt(state_machine_state state, int mode)
    {
        // set map element without class initialization or copying
        if (interrupt_states.find(PIN) != interrupt_states.end()) {
            interrupt_states.erase(PIN);
        }
        interrupt_states.insert(std::make_pair(PIN, std::make_pair(this, state)));
        instance_interrupts[this].push_back(PIN);
        // interrupt_states[PIN] = state;
        attachInterrupt(digitalPinToInterrupt(PIN), _interrupt_state<PIN>, mode);
    }

    ~StateMachine();

    void wait(uint32_t ms, state_machine_state next);

    void wait_state();

    void noop_state();

protected:
    state_machine_state current_state_function = nullptr;

    uint32_t wait_time = 0;
    uint32_t check_time = 0;
    state_machine_state state_after_wait = cast_state(noop_state);
};

template <uint8_t PIN> void _interrupt_state()
{
    auto pin_state = interrupt_states[PIN];
    ((pin_state.first)->*pin_state.second)();
}

#endif // INCLUDE_STATE_MACHINE_HPP_
