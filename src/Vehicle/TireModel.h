#pragma once

// タイヤ 1 本が路面から受ける力。
// 前後の滑り（スリップ率）と横の滑り（スリップ角）に比例して力が立ち上がり、ピークを越えると下がる。
// 前後と横の合計は摩擦円（μ × 荷重）で制限する。車輪の座標で、fx は前向き、fy は右向きが正。
struct TireForce {
    float fx = 0;
    float fy = 0;
    bool sliding = false;  // ピークを越えて滑っている
};

class ITireModel {
public:
    virtual ~ITireModel() = default;

    // load: 荷重（N）、slipRatio: (ωr − 前向きの速度) / 速度、slipAngle: 横の滑りの角度（rad、右へ滑るのが正）、
    // mu: 路面の摩擦係数
    TireForce Evaluate(float load, float slipRatio, float slipAngle, float mu) const;

    // 力の上限（N）。
    float PeakForce(float load, float mu) const { return mu * muScale() * load; }
    // 前後の力がピークになるスリップ率。ABS とトラクションコントロールが使う。
    float PeakSlipRatio() const { return peakSlipRatio(); }
    // 横の力がピークになるスリップ角（rad）。
    float PeakSlipAngle() const { return peakSlipAngle(); }
    // ピークまでの前後の傾き（N / スリップ率）。車輪の回転を陰的に解くのに使う。
    float LongitudinalStiffness(float load, float mu) const { return PeakForce(load, mu) / peakSlipRatio(); }

protected:
    virtual float peakSlipRatio() const = 0;
    virtual float peakSlipAngle() const = 0;  // rad
    virtual float slideRatio() const = 0;     // 滑りきったときの力 / ピークの力
    virtual float muScale() const = 0;        // 路面の μ に掛ける倍率
};

// グリップ走行用。ピークを越えても力があまり落ちず、滑ってもすぐ戻る。
// μ の倍率 1.45 は遊びやすさのための値。舗装路（μ 1.1）で約 1.6 になり、実車とアーケードの中間の手応えにする。
class GripTire : public ITireModel {
protected:
    float peakSlipRatio() const override { return 0.10f; }
    float peakSlipAngle() const override { return 0.12f; }
    float slideRatio() const override { return 0.9f; }
    float muScale() const override { return 1.45f; }
};

// ドリフト用。横のピークが早く、越えると力が大きく落ちるので、滑らせたまま保ちやすい。
class DriftTire : public ITireModel {
protected:
    float peakSlipRatio() const override { return 0.10f; }
    float peakSlipAngle() const override { return 0.09f; }
    float slideRatio() const override { return 0.7f; }
    float muScale() const override { return 1.38f; }  // グリップ用の 0.95 倍
};
