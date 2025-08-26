#pragma once
#include <Windows.h>

class FrameTimer
{
public:
    void Init(int requestedFPS);
    int GetFramesToUpdate();
    float GetDeltaTime();

private:
    LARGE_INTEGER timerFreq;
    LARGE_INTEGER timeNow, timePrev;
    int requestedFPS;
    float intervalsPerFrame;
    long long deltaTime;

};
