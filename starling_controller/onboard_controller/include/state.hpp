#ifndef STATE_HPP
#define STATE_HPP

using namespace std;

class State{
    public:
        enum Value: uint8_t {
            INIT,
            TAKEOFF,
            GOTOSTART,
            LAND,
            STOP,
            EXECUTE,
            MAKESAFE,
            TERMINATE,
        };

    State() = default;
    constexpr State(Value state) : value(state) { }

    // Allow switch and comparisons.
    constexpr operator Value() const { return value; }

    // Prevent usage: if(fruit)
    explicit operator bool() = delete;

    constexpr bool operator==(State a) const { return value == a.value; }
    constexpr bool operator!=(State a) const { return value != a.value; }

    constexpr bool operator==(State::Value a) const { return value == a; }
    constexpr bool operator!=(State::Value a) const { return value != a; }


    string to_string() const {
        switch(value){
            case INIT:
                return "STATE::INIT";
            case TAKEOFF:
                return "STATE::TAKEOFF";
            case GOTOSTART:
                return "STATE::GOTOSTART";
            case LAND:
                return "STATE::LAND";
            case STOP:
                return "STATE::STOP";
            case EXECUTE:
                return "STATE::EXECUTE";
            case TERMINATE:
                return "STATE::TERMINATE";
            case MAKESAFE:
                return "STATE::MAKESAFE";
            default:
                return "INVALID_STATE";
        };
    }

    private:
        Value value;
};



#endif