/* ff-bgm1x: mGBA 의 GB 사운드 장치(GBAudio)를 원본 코어와 따로 하나 더 만들어 그림자 소리를 낸다.
 * 원본 코어와 같은 합성 코드라서 1배속 소리가 원본과 같은 방식으로 합성된다. */
#pragma once

#include <memory>

#include "GbDriverGeneric.h"

namespace QGBA {

// style: mGBA 의 enum GBAudioStyle 값 (원본 코어의 gb->audio.style)
std::unique_ptr<GbNativeApu> makeMgbaGbApu(int style);
// 확인용: 안의 mGBA 장치 상태 (struct GBAudio*)
const void* mgbaGbApuState(const GbNativeApu* apu);

}
