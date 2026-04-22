#pragma once
#include <deque>
#include <cmath>
#include <algorithm>

// SpectralPulseDetector: peak-based beacon pulse detector operating on the
// FFT best-1kHz SNR in σ units (best1kHzSnrSigma from FFTProcessor).
//
// Pipeline:
//   best1kHzSnrSigma (per FFT frame) → energy ring buffer
//   → local-max ROI detection + σ thresholds + rhythm penalties
//   → phase lock on T_target
//   → live_etat = min(5, floor(Σ état(last 4T) / 3))  → PulseLevel
//
// Unlike AudioPulseDetector, no noise-reference division is needed:
// snrSigma is already self-normalised against the per-frame noise floor.
//
class SpectralPulseDetector {
public:
    enum class PulseLevel { NONE = 0, LOW = 1, MEDIUM = 2, STRONG = 3 };

    struct Config {
        float fsEnergy      = 20.f;    // FFT frame rate (Hz) — update via configure()
        float zDefaultS     = 0.666f;  // local-max zone half-width unlocked (s)
        float tTargetInit   = 1.75f;   // initial period estimate (s)
        float dtTolS        = 0.150f;  // ±tolerance for rhythm / lock (s)
        float snrMin        = 1.5f;    // min σ to admit ROI (requires dans_rythme)
        float snrRhythm     = 2.5f;    // σ for admission in-rhythm
        float snrStrong     = 4.0f;    // σ for unconditional admission (base = 5)
        float dispersionMax = 1.3f;
        int   sumNMax       = 7;
        float liveWindowT   = 4.0f;
        float liveDivisor   = 3.0f;
    };

    explicit SpectralPulseDetector() : SpectralPulseDetector(Config{}) {}
    explicit SpectralPulseDetector(const Config& cfg);

    void       configure(const Config& cfg);
    PulseLevel process(float snrSigma, float freqHz);

    PulseLevel pulseDetected()      const { return lastLevel_; }
    float      lastPulseStrength()  const { return lastSnr_; }
    bool       isLocked()           const { return isLocked_; }
    float      lockedPeriodS()      const { return tTarget_; }
    int        liveEtat()           const { return liveEtat_; }
    float      estimatedFreqHz()    const;
    void       reset();

private:
    Config cfg_;

    std::deque<float> eBuf_;
    std::deque<float> freqBuf_;   // parallel to eBuf_: best1kHz centre Hz per frame
    float             eBufT0_ = 0.f;

    float timeOfIdx(int i)   const;
    int   idxOfTime(float t) const;

    struct Roi { float t; int etat; float freqHz; };
    std::deque<Roi> rois_;
    float tLastRoi_    = -1.f;
    int   lastScanIdx_ = 0;

    struct FreqSample { float t; float freqHz; };
    std::deque<FreqSample> freqHistory_;  // last 30 admitted ROIs for linear regression
    static constexpr int kFreqHistoryMax = 30;

    bool  isLocked_  = false;
    float tTarget_;
    std::deque<float> last3Dts_;
    std::deque<float> histDts_;
    std::deque<int>   histN_;

    int        liveEtat_  = 0;
    float      lastSnr_   = 0.f;
    PulseLevel lastLevel_ = PulseLevel::NONE;

    void       onEnergyFrame(float snrSigma, float freqHz);
    void       detectRois();
    int        computeLiveEtat() const;
    PulseLevel toLevel(int etat) const;
};
