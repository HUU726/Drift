#pragma once

#include "track.hpp"

// 自動運転でコースを 50 秒走り、結果を表示する。0 なら成功。SDL と GL は使わない。
int RunSelfTest(const Track& track);
