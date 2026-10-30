#pragma once

#include "GameObject/GameObject.h"

#include <functional>
#include <string>
#include <vector>

class CarObject;

// 調整の画面。プレイ中に車の値を変える（試験コースで使う）。
// Tab で開閉し、開いている間は ↑↓ で項目を選び、←→ で値を変える。E で今の値を tuning.txt に書き出す。
// 開いている間、矢印キーは運転に使わない（運転は WASD）。
// 基本の項目は、車体（質量、回りにくさ、重心、足の硬さ、出力、ブレーキ）、タイヤ（静止摩擦、動摩擦、前後の配分）、
// ドリフト（ドリフト中の後輪のグリップ、角度の補助）、運転の補助、音量（全体、BGM、効果音）。F で、1 つずつの値を直接変える細かい項目に切り替える。どれも変えた瞬間に効く。
class TuningObject : public GameObject {
public:
    explicit TuningObject(Scene& scene) : GameObject(scene) { order = 110; }
    bool Init() override;
    void StartUpdate(float dt) override;
    void Draw2D(System::Renderer2D& hud) override;
    void UnInit() override;

private:
    struct Item {
        const char* label;   // 画面に出す名前
        const char* key;     // 書き出すときの名前
        float min, max, step;
        std::function<float()> get;
        std::function<void(float)> set;
        const char* offText = nullptr;  // 2 択の項目（値 0 / 1）のときの表示
        const char* onText = nullptr;
        const char* category = "";      // 見出しに出す分類
    };

    void buildBasicItems();
    void buildDetailItems();
    std::vector<Item>& items() { return detail_ ? detailItems_ : basicItems_; }
    std::string valueText(const Item& item) const;
    void exportValues();

    CarObject* car_ = nullptr;
    std::vector<Item> basicItems_;
    std::vector<Item> detailItems_;
    bool detail_ = false;   // 細かい項目を出しているか
    int selected_ = 0;
    int firstVisible_ = 0;  // 一覧の先頭に見えている項目（項目が多いのでスクロールする）
    bool open_ = false;
    std::string message_;    // 書き出した後などに、しばらく出す一言
    float messageTime_ = 0;
};
