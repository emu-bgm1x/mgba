/* ff-bgm1x: 설명은 GbNativeApuMgba.h */
#include "GbNativeApuMgba.h"

#include <algorithm>
#include <cmath>

#include <mgba/core/timing.h>
#include <mgba/internal/gb/audio.h>

namespace QGBA {

namespace {

constexpr int kTimingFactor = 2;     // GB(1배속 CPU) 에서 mGBA 시간 단위 = 4.19MHz 사이클 x2
constexpr int kFrameUnits = 8192 * kTimingFactor;  // 길이·스윕·엔벨로프 박자 (512Hz, 원본은 DIV 로 친다)
constexpr int kChunkUnits = 32 * kTimingFactor;    // 출력을 이만큼마다 읽는다 (원본 코어의 표본 간격)

class MgbaGbApu : public GbNativeApu {
public:
	explicit MgbaGbApu(int style) {
		mTimingInit(&m_timing, &m_relative, &m_nextEvent);
		GBAudioInit(&m_audio, 2048, &m_nr52, static_cast<GBAudioStyle>(style));
		m_audio.p = nullptr;  // 게임 기기 없이 쓴다 (표본 이벤트는 쓰지 않고 출력은 run 이 직접 읽는다)
		m_audio.timing = &m_timing;
		GBAudioReset(&m_audio);
		mTimingDeschedule(&m_timing, &m_audio.sampleEvent);
	}
	~MgbaGbApu() override {
		GBAudioDeinit(&m_audio);
		mTimingDeinit(&m_timing);
	}

	void write(uint8_t reg, uint8_t v) override {
		m_timing.masterCycles = m_now;
		// 전원이 꺼진 동안의 쓰기는 원본 코어(io.c)처럼 무시한다
		if (reg < 0x26 && !m_audio.enable) {
			return;
		}
		switch (reg) {
		case 0x10: GBAudioWriteNR10(&m_audio, v); break;
		case 0x11: GBAudioWriteNR11(&m_audio, v); break;
		case 0x12: GBAudioWriteNR12(&m_audio, v); break;
		case 0x13: GBAudioWriteNR13(&m_audio, v); break;
		case 0x14: GBAudioWriteNR14(&m_audio, v); break;
		case 0x16: GBAudioWriteNR21(&m_audio, v); break;
		case 0x17: GBAudioWriteNR22(&m_audio, v); break;
		case 0x18: GBAudioWriteNR23(&m_audio, v); break;
		case 0x19: GBAudioWriteNR24(&m_audio, v); break;
		case 0x1A: GBAudioWriteNR30(&m_audio, v); break;
		case 0x1B: GBAudioWriteNR31(&m_audio, v); break;
		case 0x1C: GBAudioWriteNR32(&m_audio, v); break;
		case 0x1D: GBAudioWriteNR33(&m_audio, v); break;
		case 0x1E: GBAudioWriteNR34(&m_audio, v); break;
		case 0x20: GBAudioWriteNR41(&m_audio, v); break;
		case 0x21: GBAudioWriteNR42(&m_audio, v); break;
		case 0x22: GBAudioWriteNR43(&m_audio, v); break;
		case 0x23: GBAudioWriteNR44(&m_audio, v); break;
		case 0x24: GBAudioWriteNR50(&m_audio, v); break;
		case 0x25: GBAudioWriteNR51(&m_audio, v); break;
		case 0x26: GBAudioWriteNR52(&m_audio, v); break;
		default:
			if (reg >= 0x30 && reg < 0x40) {
				// 파형 RAM (원본 코어 io.c 와 같은 처리)
				GBAudioRun(&m_audio, m_now, 0x4);
				if (!m_audio.playingCh3) {
					m_audio.ch3.wavedata8[reg - 0x30] = v;
				} else if (m_audio.ch3.readable || m_audio.style == GB_AUDIO_CGB) {
					m_audio.ch3.wavedata8[m_audio.ch3.window >> 1] = v;
				}
			}
			break;
		}
	}

	void run(double cycles, float& left, float& right) override {
		m_carry += cycles * kTimingFactor;
		const int units = static_cast<int>(m_carry);
		m_carry -= units;
		if (units <= 0) {
			left = m_lastL;
			right = m_lastR;
			return;
		}
		double accL = 0, accR = 0;
		int left_units = units;
		while (left_units > 0) {
			const int step = std::min({left_units, kChunkUnits - m_chunkPos, m_untilFrame});
			m_now += step;
			m_chunkPos += step;
			m_untilFrame -= step;
			left_units -= step;
			m_timing.masterCycles = m_now;
			if (m_untilFrame <= 0) {
				GBAudioRun(&m_audio, m_now, 0x1F);
				GBAudioUpdateFrame(&m_audio);
				m_untilFrame += kFrameUnits;
			}
			if (m_chunkPos >= kChunkUnits) {
				m_chunkPos = 0;
				sample();
			}
			accL += m_lastL * step;
			accR += m_lastR * step;
		}
		left = static_cast<float>(accL / units);
		right = static_cast<float>(accR / units);
	}

	void setMutedChannels(uint8_t mask) override {
		for (int i = 0; i < 4; ++i) {
			m_audio.forceDisableCh[i] = (mask >> i) & 1;
		}
	}
	void setGain(float gain) override {
		const int q = static_cast<int>(std::lround(std::max(0.0f, gain) * 256));
		for (int i = 0; i < 4; ++i) {
			m_audio.ffbgmGainQ8[i] = q;
		}
	}

	const struct GBAudio* state() const { return &m_audio; }
	void setFramePhase(int cycles) override { m_untilFrame = std::max(1, cycles * kTimingFactor); }

private:
	// 원본 코어의 GBAudioSample 과 같은 계산 (직류 제거는 그림자 쪽에서 한다)
	void sample() {
		GBAudioRun(&m_audio, m_now, 0x1F);
		int16_t l = 0, r = 0;
		GBAudioSamplePSG(&m_audio, &l, &r);
		m_lastL = static_cast<float>(static_cast<int16_t>((l * m_audio.masterVolume * 6) >> 7));
		m_lastR = static_cast<float>(static_cast<int16_t>((r * m_audio.masterVolume * 6) >> 7));
	}

	struct mTiming m_timing {};
	int32_t m_relative = 0, m_nextEvent = 0;
	struct GBAudio m_audio {};
	uint8_t m_nr52 = 0;
	int32_t m_now = 0;
	double m_carry = 0;
	int m_chunkPos = 0;
	int m_untilFrame = kFrameUnits;
	float m_lastL = 0, m_lastR = 0;
};

}

const void* mgbaGbApuState(const GbNativeApu* apu) {
	return static_cast<const MgbaGbApu*>(apu)->state();
}

std::unique_ptr<GbNativeApu> makeMgbaGbApu(int style) {
	return std::unique_ptr<GbNativeApu>(new MgbaGbApu(style));
}

}
