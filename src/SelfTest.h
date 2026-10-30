#pragma once

#include "track.hpp"

// 自動運転でコースを 50 秒走り、結果を表示する。0 なら成功。SDL と GL と音の出力は使わない（音は合成した値だけを調べる）。
int RunSelfTest(const Track& track);
