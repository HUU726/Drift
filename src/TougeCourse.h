#pragma once

struct Track;

// 峠のコース（架空）。山頂から下り、つづら折りのヘアピンを抜けて谷へ降り、別の尾根を登って山頂へ戻る 1 周。
// 直線と円弧を順につないで中心線を作り、Track::build で路面と見た目を作る。
bool BuildTougeCourse(Track& track);

// 走り比べのコース（架空）。S 字とヘアピンを形を変えて並べた平らな 1 周。区間の名前（S-A など）が HUD に出る。
bool BuildLabCourse(Track& track);
