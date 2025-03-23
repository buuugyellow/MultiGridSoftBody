#pragma once
class Solver {
public:
    virtual void Init() = 0;
    virtual void Step() = 0;
};