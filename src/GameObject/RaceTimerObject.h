#pragma once

#include "GameObject/GameObject.h"

class CarObject;
class TrackObject;

struct Race {
    int completed = 0;
    bool timing = false;
    float lap = 0;
    float best = -1;
    float prevS = -1;
};

// 周回数とラップタイム。車がコントロールラインを越えたかを固定ステップごとに調べる。
class RaceTimerObject : public GameObject {
public:
    explicit RaceTimerObject(Scene& scene) : GameObject(scene) { order = 20; }
    bool Init() override;
    void PostPhysicsUpdate(float dt) override;

    void Reset() { race_ = Race{}; }
    const Race& GetRace() const { return race_; }

private:
    const CarObject* car_ = nullptr;
    const TrackObject* track_ = nullptr;
    Race race_;
};
