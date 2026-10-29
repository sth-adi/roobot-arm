#pragma once

#include "../core/RobotTypes.h"

bool ReadControllerValue(const String& state, const char* name, float& value)
{
    const String key = "\"" + String(name) + "\"";
    const int keyStart = state.indexOf(key);
    if (keyStart < 0) {
        return false;
    }

    const int valueStart = state.indexOf(':', keyStart + key.length());
    if (valueStart < 0) {
        return false;
    }

    char* valueEnd = nullptr;
    value = strtof(state.c_str() + valueStart + 1, &valueEnd);
    return valueEnd != state.c_str() + valueStart + 1;
}

bool ReadControllerButton(const String& state, const char* name, int& value)
{
    const String key = "\"" + String(name) + "\"";
    const int keyStart = state.indexOf(key);
    if (keyStart < 0) {
        return false;
    }

    const int valueStart = state.indexOf(':', keyStart + key.length());
    if (valueStart < 0) {
        return false;
    }

    String buttonValue = state.substring(valueStart + 1);
    buttonValue.trim();
    if (buttonValue.startsWith("true")) {
        value = 1;
        return true;
    }

    if (buttonValue.startsWith("false")) {
        value = 0;
        return true;
    }

    return false;
}

bool DecodeStickInput(const String& state, StickInput& input)
{
    return ReadControllerValue(state, "LSX", input.leftX)
        && ReadControllerValue(state, "LSY", input.leftY)
        && ReadControllerValue(state, "RSX", input.rightX)
        && ReadControllerValue(state, "RSY", input.rightY)
        && ReadControllerButton(state, "A", input.aButton)
        && ReadControllerButton(state, "B", input.bButton)
        && ReadControllerButton(state, "X", input.xButton)
        && ReadControllerButton(state, "Y", input.yButton);
}
