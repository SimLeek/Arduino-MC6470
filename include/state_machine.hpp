#ifndef __STATE_MACHINE_HPP
#define __STATE_MACHINE_HPP

#include <Arduino.h>
#include <map>

//#define DEBUG
#ifdef DEBUG
#define func_print Serial.println(__func__)
#else
#define func_print
#endif

template <uint8_t PIN>
void _interrupt_state();

class StateMachine;
typedef void *(StateMachine::*state_machine_state)();
std::map<uint8_t, std::pair<StateMachine, state_machine_state>> interrupt_states;

#define cast_state(x) reinterpret_cast<state_machine_state>(&x)
#define set_state(x) this->current_state_function = cast_state(x)
class StateMachine
{
public:
    state_machine_state current_state_function = nullptr;

    void run_state()
    {
        func_print;
        (this->*(this->current_state_function))();
    }

    template <uint8_t PIN>
    void run_state_on_interrupt(state_machine_state state, int mode)
    {
        func_print;
        // set map element without class initialization or copying
        if (interrupt_states.find(PIN) != interrupt_states.end())
        {
            interrupt_states.erase(PIN);
        }
        interrupt_states.insert(std::make_pair(PIN, std::make_pair(this, state)));
        // interrupt_states[PIN] = state;
        attachInterrupt(digitalPinToInterrupt(PIN), _interrupt_state<PIN>, mode);
    }

    void wait(unsigned long ms, state_machine_state next)
    {
        func_print;
        wait_time = ms;
        state_after_wait = next;
        check_time = millis();
        this->current_state_function = reinterpret_cast<state_machine_state>(&StateMachine::wait_state);
    }

    void wait_state()
    {
        func_print;
        if (millis() - check_time > wait_time)
        {
            this->current_state_function = this->state_after_wait;
        }
    }

    unsigned long wait_time = 0;
    unsigned long check_time = 0;
    state_machine_state state_after_wait;
};

template <uint8_t PIN>
void _interrupt_state()
{
    auto pin_state = interrupt_states[PIN];
    ((pin_state.first).*pin_state.second)();
}

#endif