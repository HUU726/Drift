#pragma once

// 駆動系。アクセル開度と駆動輪の回転から、駆動輪の車軸に掛かるトルクを返す。
// Car は 1 つを持ち、返ったトルクを後輪の左右に均等に配る（オープンデフ）。
class IPowertrain {
public:
    virtual ~IPowertrain() = default;

    // throttle: 0..1、reverse: 後退ギアに入れる、wheelOmega: 駆動輪の平均の回転（rad/s、前へ回る向きが正）。
    // 返り値は車軸全体のトルク（N·m、前へ回す向きが正）。dt だけ内部の状態（変速）を進める。
    virtual float Update(float throttle, bool reverse, float wheelOmega, float dt) = 0;
    virtual void Reset() = 0;

    // HUD 用。Gear は 1 以上が前進、-1 が後退。
    virtual float Rpm() const = 0;
    virtual int Gear() const = 0;
};

// エンジン + 6 速の自動変速 + デフ。回転数は駆動輪の回転から逆算し、アイドルより下がらない（クラッチが滑る）。
class EnginePowertrain : public IPowertrain {
public:
    float Update(float throttle, bool reverse, float wheelOmega, float dt) override;
    void Reset() override;
    float Rpm() const override { return rpm_; }
    int Gear() const override { return gear_; }

    // トルクカーブ（rpm: N·m、間は線形補間）
    static float EngineTorque(float rpm);

    // 試験コースの調整の画面でプレイ中に変えられる値
    float torqueScale = 1.0f;     // トルクカーブ全体に掛ける倍率
    float finalDrive = 3.9f;      // 最終減速比
    float upshiftRpm = 7200.0f;   // この回転数を超えたら上げる
    float downshiftRpm = 3500.0f; // この回転数を下回ったら下げる

private:
    float ratio() const;

    int gear_ = 1;
    float rpm_ = 0;
    float shiftTimer_ = 0;  // 変速中は駆動を切る
};

// 電気モーター。ギアを持たず、低い回転では一定のトルク、高い回転では一定の出力になる。
class MotorPowertrain : public IPowertrain {
public:
    float Update(float throttle, bool reverse, float wheelOmega, float dt) override;
    void Reset() override { rpm_ = 0; reverse_ = false; }
    float Rpm() const override { return rpm_; }
    int Gear() const override { return reverse_ ? -1 : 1; }

private:
    float rpm_ = 0;
    bool reverse_ = false;
};
