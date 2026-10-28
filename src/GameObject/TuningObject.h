#pragma once

#include "GameObject/GameObject.h"

#include <functional>
#include <string>
#include <vector>

class CarObject;

// 調整の画面。プレイ中に車の値を変える（試験コースで使う）。
// Tab で開閉し、開いている間は ↑↓ で項目を選び、←→ で値を変える。E で今の値を tuning.txt に書き出す。
// 開いている間、矢印キーは運転に使わない（運転は WASD）。
// 項目はタイヤ、運転の補助、足回り、駆動、ブレーキとハンドル、車体。どれも変えた瞬間に効く。
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

    void buildItems();
    std::string valueText(const Item& item) const;
    void exportValues();

    CarObject* car_ = nullptr;
    std::vector<Item> items_;
    int selected_ = 0;
    int firstVisible_ = 0;  // 一覧の先頭に見えている項目（項目が多いのでスクロールする）
    bool open_ = false;
    std::string message_;    // 書き出した後などに、しばらく出す一言
    float messageTime_ = 0;
};
