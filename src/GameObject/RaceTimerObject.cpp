#include "GameObject/RaceTimerObject.h"

#include "GameObject/CarObject.h"
#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/SoundManager.h"

namespace {

void tickRace(Race& race, float s, float length, float dt) {
    if (race.prevS >= 0) {
        float gap = (length - race.prevS) + s;
        if (race.prevS > length * 0.5f && s < length * 0.5f && gap < 160.0f) {
            if (race.timing) {
                if (race.best < 0 || race.lap < race.best) {
                    race.best = race.lap;
                }
                race.completed += 1;
            }
            race.timing = true;
            race.lap = 0;
        }
    }
    race.prevS = s;
    race.lap += dt;
}

}  // namespace

bool RaceTimerObject::Init() {
    car_ = scene_.Find<CarObject>();
    track_ = scene_.Find<TrackObject>();
    return car_ && track_;
}

void RaceTimerObject::PostPhysicsUpdate(float dt) {
    const int completed = race_.completed;
    const float best = race_.best;
    tickRace(race_, car_->GetCar().ground.s, track_->GetTrack().length, dt);
    // 1 周したら短い音を鳴らす。2 周目からは、ベストラップを縮めたときに別の音にする。
    if (race_.completed > completed) {
        bool newBest = completed > 0 && race_.best < best;
        scene_.Ctx().sound.PlaySe(newBest ? System::SoundEffect::BestLap : System::SoundEffect::Lap);
    }
}
