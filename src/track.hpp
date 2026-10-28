#pragma once

#include "meshbuild.hpp"

#include <string>
#include <vector>

struct TrackPose {
    bool found = false;
    float s = 0;
    float lateral = 0;  // 右が正
    float centerY = 0;
    float cross = 0;  // 右方向 1m あたりの高さ
    float yaw = 0;
    float rightX = 1;
    float rightZ = 0;
    float xzDist = 0;
    int corner = 0;
    int nameId = -1;
};

struct TrackSample {
    float x = 0;
    float z = 0;
    float y = 0;
    float cross = 0;
    float s = 0;
    int corner = 0;
    int nameId = -1;
    bool bridge = false;
};

// 鈴鹿サーキット。中心線は OSM、高さは国土地理院の標高。
struct Track {
    float originLat = 0;
    float originLon = 0;
    float halfWidth = 6.5f;
    float curbWidth = 0.8f;
    float runoff = 8.0f;
    float length = 0;
    float spawnX = 0;
    float spawnY = 0;
    float spawnZ = 0;
    float spawnYaw = 0;
    std::vector<TrackSample> samples;
    std::vector<std::string> names;
    CpuMesh visual;

    bool load(const std::string& path);
    // preferS が 0 以上のときは、その周回位置に近い路面を優先する。立体交差で上下を取り違えない。
    TrackPose query(float x, float z, float yHint, float preferS = -1.0f) const;
    // preferS の前後 kLocalWindow（m）の区間だけを調べる。1 ステップに何度も呼ぶ物理用。
    // preferS が負のとき、範囲内に路面がないとき、遠すぎるときは query（全区間）に切り替える。
    TrackPose queryLocal(float x, float z, float yHint, float preferS) const;
    static constexpr float kLocalWindow = 50.0f;
    float wallLateral() const { return halfWidth + curbWidth + runoff; }
    const char* nameOf(int nameId) const;
};
