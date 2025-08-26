#include "FrameTimer.h"

void FrameTimer::Init(int requestedFPS)
{
    QueryPerformanceFrequency(&timerFreq);
    QueryPerformanceCounter(&timeNow);
    QueryPerformanceCounter(&timePrev);

    this->requestedFPS = requestedFPS;
    intervalsPerFrame = timerFreq.QuadPart / requestedFPS;
}

int FrameTimer::GetFramesToUpdate()
{
    QueryPerformanceCounter(&timeNow);
    deltaTime = timeNow.QuadPart - timePrev.QuadPart;

    int framesToUpdate = static_cast<int>(deltaTime / intervalsPerFrame);
    
    if (framesToUpdate > 0)
    {
        QueryPerformanceCounter(&timePrev);
    }

    return framesToUpdate;
}

float FrameTimer::GetDeltaTime()
{
    return static_cast<float>(deltaTime) / timerFreq.QuadPart;
}

