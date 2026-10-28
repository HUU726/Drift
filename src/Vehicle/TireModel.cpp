#include "Vehicle/TireModel.h"

#include <cmath>

TireForce ITireModel::Evaluate(float load, float slipRatio, float slipAngle, float mu) const {
    TireForce out;
    if (load <= 0) {
        return out;
    }
    // ピークの滑りを 1 とした、前後と横の滑りの合計。
    float sx = slipRatio / params.peakSlipRatio;
    float sy = slipAngle / params.peakSlipAngle;
    float s = std::sqrt(sx * sx + sy * sy);
    if (s < 1e-6f) {
        return out;
    }
    float peak = PeakForce(load, mu);
    float force;
    if (s <= 1.0f) {
        force = peak * s;
    } else {
        // ピークを越えると、滑りきったときの力へなめらかに下がる。
        float slide = params.slideRatio;
        force = peak * (slide + (1.0f - slide) * std::exp(-(s - 1.0f) * 2.0f));
        out.sliding = true;
    }
    out.fx = force * sx / s;
    out.fy = -force * sy / s;  // 右へ滑るほど左向きの力
    return out;
}
