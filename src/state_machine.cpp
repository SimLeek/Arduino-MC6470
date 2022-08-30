// Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

#include "state_machine.hpp"

#include <Arduino.h>
#include <utility>

std::map<uint8_t, std::pair<StateMachine*, state_machine_state>> interrupt_states;
std::map<StateMachine*, std::vector<uint8_t>> instance_interrupts;

void StateMachine::update() { (this->*(this->current_state_function))(); }

void StateMachine::update_until_done()
{
    while (!is_done()) {
        (this->*(this->current_state_function))();
    }
}

bool StateMachine::is_done()
{
    return this->current_state_function == cast_state(noop_state) || this->current_state_function == nullptr;
}

StateMachine::~StateMachine()
{
    // remove dangling pointers
    if (instance_interrupts.find(this) != instance_interrupts.end()) {
        for (auto it = instance_interrupts[this].begin(); it != instance_interrupts[this].end(); it++) {
            interrupt_states.erase(*it);
        }
        instance_interrupts.erase(this);
    }
}

void StateMachine::wait(uint32_t ms, state_machine_state next)
{
    wait_time = ms;
    state_after_wait = next;
    check_time = millis();
    this->current_state_function = reinterpret_cast<state_machine_state>(&StateMachine::wait_state);
}

void StateMachine::wait_state()
{
    if (millis() - check_time > wait_time) {
        this->current_state_function = this->state_after_wait;
    }
}

void StateMachine::noop_state() { }
