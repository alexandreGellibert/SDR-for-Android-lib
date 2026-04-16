#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <cmath>
#include <algorithm>

// PulseEngine: peak-based beacon pulse detector.
//
// Pipeline:
//   PCM → bandpass [fMin–fMax] → RMS/10ms → lowpass 5 Hz → energy buffer
//   → local-max ROI detection + SNR + rhythm penalties
//   → phase lock on T_target
//   → live_etat = min(5, floor(Σ état(last 4T) / 3))  → PulseLevel
//
class AudioPulseDetector {
public:
    enum class PulseLevel { NONE = 0, LOW = 1, MEDIUM = 2, STRONG = 3 };

    struct Config {
        float sampleRate    = 48000.f;
        float fMin          = 1500.f;    // bandpass low  (Hz)
        float fMax          = 4000.f;    // bandpass high (Hz)
        float fsEnergy      = 100.f;     // energy frame rate (10 ms steps)
        float smoothCutoff  = 5.f;       // energy lowpass cutoff (Hz)
        float zDefaultS     = 0.666f;    // local-max zone half-width unlocked (s)
        float tTargetInit   = 1.75f;     // initial period estimate (s)
        float dtTolS        = 0.150f;    // ±tolerance for rhythm / lock (s)
        float snrMin        = 1.0f;      // min SNR to admit ROI
        float snrRhythm     = 1.1f;      // SNR required when in-rhythm only
        float snrStrong     = 2.0f;      // unconditional admission
        float dispersionMax = 1.3f;      // confusion: max cumulative dt dispersion
        int   sumNMax       = 7;         // confusion: max ΣN_cycles in history
        float liveWindowT   = 4.0f;      // live state: look-back (× T_target)
        float liveDivisor   = 3.0f;      // live state divisor
        int   noiseRefFar   = 80;        // noise ref far  edge (energy frames back)
        int   noiseRefNear  = 40;        // noise ref near edge (energy frames back)
    };

    explicit AudioPulseDetector();
    explicit AudioPulseDetector(const Config& cfg);

    PulseLevel process(const std::vector<float>&   audio);
    PulseLevel process(const std::vector<int16_t>& pcm);

    PulseLevel pulseDetected()     const { return lastLevel_; }
    float      lastPulseStrength() const { return lastSnr_; }
    bool       isLocked()          const { return isLocked_; }
    float      lockedPeriodS()     const { return tTarget_; }
    int        liveEtat()          const { return liveEtat_; }
    void       reset();

private:
    Config cfg_;

    // ── Biquad SOS — Direct Form II Transposed ───────────────────────────────
    struct SOS { float b0,b1,b2, a1,a2, z1=0.f,z2=0.f; };
    std::vector<SOS> bandSOS_;   // HP(fMin) + LP(fMax)
    std::vector<SOS> lowSOS_;    // LP(smoothCutoff) on energy signal

    static SOS makeLP2(float fs, float fc, float Q = 0.7071f);
    static SOS makeHP2(float fs, float fc, float Q = 0.7071f);
    static float applyChain(std::vector<SOS>& chain, float x);

    // ── Audio → energy frames ────────────────────────────────────────────────
    int   frameSamples_ = 1;
    int   frameCount_   = 0;
    float frameAcc_     = 0.f;

    // ── Energy ring buffer (@ fsEnergy Hz) ──────────────────────────────────
    std::deque<float> eBuf_;
    float             eBufT0_ = 0.f;  // timestamp (s) of eBuf_[0]

    float timeOfIdx(int i)   const;
    int   idxOfTime(float t) const;
    float noiseRef (int i)   const;

    // ── ROI store ────────────────────────────────────────────────────────────
    struct Roi { float t; int etat; };
    std::deque<Roi> rois_;
    float tLastRoi_    = -1.f;
    int   lastScanIdx_ = 0;    // last buffer index fully scanned for ROIs

    // ── Phase lock ───────────────────────────────────────────────────────────
    bool  isLocked_  = false;
    float tTarget_;
    std::deque<float> last3Dts_;
    std::deque<float> histDts_;
    std::deque<int>   histN_;

    // ── Output ───────────────────────────────────────────────────────────────
    int        liveEtat_  = 0;
    float      lastSnr_   = 0.f;
    PulseLevel lastLevel_ = PulseLevel::NONE;

    void       onEnergyFrame(float rms);
    void       detectRois();
    int        computeLiveEtat() const;
    PulseLevel toLevel(int etat) const;
};
