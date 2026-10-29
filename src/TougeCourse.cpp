#include "TougeCourse.h"

#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kStep = 4.0f;  // 中心線の点の間隔（m）

// 区間。radius が 0 なら直線（length m）、それ以外は円弧（angle 度、右へ曲がるのが正）。
// grade は上りが正の勾配（高さの変化 / 水平の距離）。corner はコーナーの番号（直線は 0）。
struct Piece {
    float length;
    float radius;
    float angle;
    float grade;
    int corner;
    int nameId;
};

Piece straight(float length, float grade, int nameId) { return {length, 0, 0, grade, 0, nameId}; }
Piece arc(float radius, float angle, float grade, int corner, int nameId) {
    return {radius * std::fabs(angle) * kPi / 180.0f, radius, angle, grade, corner, nameId};
}

// ドリフトの峠でよく見る形を順に並べる。
// 下り: 速い右 → 左 → 切り返しの S 字 → 長い右の高速コーナー（D1 の単走の 1 コーナーのような、角度を保って
//       回り続ける区間）→ つづら折りのヘアピン 4 つ（サイドブレーキで向きを変える区間）→ 谷の直線
// 上り: 谷の直線を南へ → 右へ折れて登る → 左右の切り返し → 西へ登る → 高速の右 → 山頂へ戻る右（計算で決める）
const char* const kNames[] = {
    "SUMMIT STRAIGHT", "FIRST RIGHT",  "CEDAR LEFT",    "S CURVES",     "LONG RIGHT",
    "HAIRPIN 1",       "HAIRPIN 2",    "HAIRPIN 3",     "HAIRPIN 4",    "VALLEY RIGHT",
    "VALLEY STRAIGHT", "BRIDGE RIGHT", "CLIMB",         "CLIMB LEFT",   "CLIMBING RIGHT",
    "UPPER SWEEPER",   "SUMMIT RIGHT",
};

std::vector<Piece> layout() {
    return {
        straight(140, -0.02f, 0),
        arc(90, 45, -0.05f, 1, 1),
        straight(60, -0.05f, 1),
        arc(60, -65, -0.05f, 2, 2),
        straight(50, -0.05f, 2),
        arc(40, 75, -0.05f, 3, 3),
        straight(15, -0.05f, 3),
        arc(40, -85, -0.05f, 4, 3),
        straight(15, -0.05f, 3),
        arc(45, 70, -0.05f, 5, 3),
        straight(90, -0.05f, 3),
        arc(110, 95, -0.04f, 6, 4),
        straight(80, -0.05f, 4),
        arc(15, -175, -0.04f, 7, 5),
        straight(40, -0.09f, 5),
        arc(15, 175, -0.04f, 8, 6),
        straight(40, -0.09f, 6),
        arc(15, -175, -0.04f, 9, 7),
        straight(40, -0.09f, 7),
        arc(15, 175, -0.04f, 10, 8),
        straight(90, -0.05f, 8),
        arc(70, 60, -0.03f, 11, 9),
        straight(400, 0.055f, 10),
        arc(60, 30, 0.05f, 12, 11),
        straight(220, 0.095f, 12),
        arc(45, -55, 0.08f, 13, 13),
        arc(45, 95, 0.08f, 14, 14),
    };
}

// 右へ曲がる向きの、yaw の円の中心までの向き（右）。
void rightOf(float yaw, float& rx, float& rz) {
    rx = std::cos(yaw);
    rz = -std::sin(yaw);
}

// 始点へ戻る区間の名前と番号。first / last は 2 つの右の円弧、straight はその間の直線、runUp は最後の直線。
struct CloseNames {
    int firstCorner, firstName;
    int straightName;
    int lastCorner, lastName;
    int runUpName;
};

// (x, z) を向き yaw で出て、始点（原点）に北向きで戻る区間。右の円弧 → 直線 → 右の円弧（半径 radius）で
// 原点の runUp m 手前に北向きで着き、平らな直線（山頂の直線の手前側。スタート地点を置く）で原点へ入る。
// 高さは、直線の手前までの勾配で原点の高さ（0）に戻す。
std::vector<Piece> closeLoop(float x, float z, float y, float yaw, float radius, float runUp, const CloseNames& names) {
    float rx, rz;
    rightOf(yaw, rx, rz);
    float c1x = x + rx * radius;
    float c1z = z + rz * radius;
    float c2x = radius;  // (0, -runUp) を北向きで通る右の円の中心
    float c2z = -runUp;
    float heading = std::atan2(c2x - c1x, c2z - c1z);
    float length = std::hypot(c2x - c1x, c2z - c1z);
    auto positive = [](float a) {
        a = std::fmod(a, 2.0f * kPi);
        return a < 0 ? a + 2.0f * kPi : a;
    };
    float first = positive(heading - yaw) * 180.0f / kPi;
    float second = positive(-heading) * 180.0f / kPi;
    std::vector<Piece> pieces = {arc(radius, first, 0, names.firstCorner, names.firstName),
                                 straight(length, 0, names.straightName),
                                 arc(radius, second, 0, names.lastCorner, names.lastName)};
    float total = 0;
    for (const Piece& piece : pieces) {
        total += piece.length;
    }
    for (Piece& piece : pieces) {
        piece.grade = -y / total;
    }
    pieces.push_back(straight(runUp, 0.0f, names.runUpName));
    return pieces;
}

// 区間を順に置いて中心線を作り、始点へ戻る区間を足して 1 周にし、峠の見た目で Track を作る。
// 山頂（原点）の高さを 0 とし、北（+z）へ向かって出る。
bool buildCourse(Track& track, std::vector<Piece> pieces, const CloseNames& closeNames, const char* const* names,
                 int nameCount) {
    track.samples.clear();
    float x = 0;
    float z = 0;
    float y = 0;
    float yaw = 0;
    const size_t designed = pieces.size();
    for (size_t index = 0; index < pieces.size(); ++index) {
        const Piece piece = pieces[index];
        int steps = std::max(1, static_cast<int>(std::lround(piece.length / kStep)));
        float ds = piece.length / steps;
        float turnPerStep = piece.radius > 0 ? (piece.angle > 0 ? 1.0f : -1.0f) * ds / piece.radius : 0.0f;
        for (int k = 0; k < steps; ++k) {
            TrackSample sample;
            sample.x = x;
            sample.z = z;
            sample.y = y;
            // 曲がりの内側へ少し傾ける（右へ 1 m あたりの高さ。右へ曲がるときは右が低い）。
            sample.cross = piece.radius > 0 ? (piece.angle > 0 ? -0.03f : 0.03f) : 0.0f;
            sample.corner = piece.corner;
            sample.nameId = piece.nameId;
            track.samples.push_back(sample);
            // 円弧は区間の中ほどの向きで進める（中点法）。
            float mid = yaw + turnPerStep * 0.5f;
            x += std::sin(mid) * ds;
            z += std::cos(mid) * ds;
            y += piece.grade * ds;
            yaw += turnPerStep;
        }
        // 決めた区間を置き終えたら、残りは始点へ戻る区間を計算して足す。
        if (index + 1 == designed) {
            std::vector<Piece> back = closeLoop(x, z, y, yaw, 60.0f, 25.0f, closeNames);
            pieces.insert(pieces.end(), back.begin(), back.end());
        }
    }

    // 最後の点が始点に戻るように、位置と高さのずれを周回の距離に比例して全体に配る。
    const int n = static_cast<int>(track.samples.size());
    float total = 0;
    std::vector<float> along(n);
    for (int i = 0; i < n; ++i) {
        along[i] = total;
        const TrackSample& a = track.samples[i];
        float nx = i + 1 < n ? track.samples[i + 1].x : x;
        float nz = i + 1 < n ? track.samples[i + 1].z : z;
        total += std::sqrt((nx - a.x) * (nx - a.x) + (nz - a.z) * (nz - a.z));
    }
    for (int i = 0; i < n; ++i) {
        float t = along[i] / total;
        track.samples[i].x -= x * t;
        track.samples[i].z -= z * t;
        track.samples[i].y -= y * t;
    }
    // 傾きは前後の点とならし、直線とコーナーのつなぎ目を緩やかにする。
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<float> cross(n);
        for (int i = 0; i < n; ++i) {
            cross[i] = 0.25f * track.samples[(i + n - 1) % n].cross + 0.5f * track.samples[i].cross +
                       0.25f * track.samples[(i + 1) % n].cross;
        }
        for (int i = 0; i < n; ++i) {
            track.samples[i].cross = cross[i];
        }
    }

    track.names.assign(names, names + nameCount);
    track.halfWidth = 4.5f;   // 片側 1 車線ずつの 9 m。ドリフトで横を向いても収まる幅
    track.curbWidth = 0.5f;   // 側溝
    track.runoff = 1.5f;      // 砂利の路肩。その外はガードレール
    track.mountain = true;

    // スタートは、スタートラインの 15 m 手前。最後の右カーブを抜けた後の平らな直線の上。
    int spawn = n - static_cast<int>(15.0f / kStep);
    const TrackSample& a = track.samples[spawn];
    const TrackSample& b = track.samples[(spawn + 1) % n];
    track.spawnX = a.x;
    track.spawnZ = a.z;
    track.spawnY = a.y;
    track.spawnYaw = std::atan2(b.x - a.x, b.z - a.z);
    return track.build();
}

// 走り比べのコース。S 字とヘアピンを形を変えて並べ、どれが走って面白いかを決める。区間の名前が HUD に出る。
// 北へ: S-A, S-B, S-C → 右へ大きく回って南へ: ヘアピン H-A, H-B, H-C → S-D, S-E → ヘアピン H-D, H-E → 始点へ戻る。
// 区間の間には 100〜120 m の直線を入れ、毎回同じくらいの速さで入れるようにする。平ら（勾配 0）。
const char* const kLabNames[] = {
    "START STRAIGHT",     "S-A R40 GAP 15",  "S-B R40 GAP 35",      "S-C R60 GAP 25",
    "U TURN",             "H-A HAIRPIN R15", "H-B HAIRPIN R20",     "H-C HAIRPIN R12",
    "S-D R30 GAP 20",     "S-E TRIPLE R40",  "H-D DOUBLE APEX R25", "H-E OPEN HAIRPIN R25",
    "STRAIGHT",           "RETURN",
};

std::vector<Piece> labLayout() {
    const int kStraight = 12;
    // S 字は向きを交互にして、横へのずれを打ち消す。ヘアピンは左右 1 組で、向きは元に戻る。
    return {
        straight(120, 0, 0),
        arc(40, 70, 0, 1, 1), straight(15, 0, 1), arc(40, -70, 0, 1, 1),
        straight(120, 0, kStraight),
        arc(40, -70, 0, 2, 2), straight(35, 0, 2), arc(40, 70, 0, 2, 2),
        straight(120, 0, kStraight),
        arc(60, 60, 0, 3, 3), straight(25, 0, 3), arc(60, -60, 0, 3, 3),
        straight(100, 0, kStraight),
        arc(35, 90, 0, 4, 4), straight(150, 0, 4), arc(35, 90, 0, 4, 4),
        straight(100, 0, kStraight),
        arc(15, -180, 0, 5, 5), straight(40, 0, 5), arc(15, 180, 0, 5, 5),
        straight(100, 0, kStraight),
        arc(20, -180, 0, 6, 6), straight(40, 0, 6), arc(20, 180, 0, 6, 6),
        straight(100, 0, kStraight),
        arc(12, -180, 0, 7, 7), straight(40, 0, 7), arc(12, 180, 0, 7, 7),
        straight(120, 0, kStraight),
        arc(30, 80, 0, 8, 8), straight(20, 0, 8), arc(30, -80, 0, 8, 8),
        straight(120, 0, kStraight),
        arc(40, -60, 0, 9, 9), straight(20, 0, 9), arc(40, 120, 0, 9, 9), straight(20, 0, 9), arc(40, -60, 0, 9, 9),
        straight(100, 0, kStraight),
        arc(25, -90, 0, 10, 10), straight(20, 0, 10), arc(25, -90, 0, 10, 10), straight(40, 0, 10),
        arc(25, 90, 0, 10, 10), straight(20, 0, 10), arc(25, 90, 0, 10, 10),
        straight(100, 0, kStraight),
        arc(25, -150, 0, 11, 11), straight(40, 0, 11), arc(25, 150, 0, 11, 11),
        straight(100, 0, kStraight),
    };
}

}  // namespace

bool BuildTougeCourse(Track& track) {
    const CloseNames close{15, 15, 16, 16, 16, 0};
    return buildCourse(track, layout(), close, kNames, static_cast<int>(std::size(kNames)));
}

bool BuildLabCourse(Track& track) {
    const CloseNames close{12, 13, 13, 13, 13, 0};
    return buildCourse(track, labLayout(), close, kLabNames, static_cast<int>(std::size(kLabNames)));
}
