#pragma once
#include <vector>
#include <cstdint>
#include <cmath>

class AudioPulseDetector {
public:
    struct Config {
        float sampleRate    = 48000.f;
        float shortWindowMs = 20.f;    // fenêtre courte — résolution temporelle
        float longWindowMs  = 800.f;   // mémoire bruit de fond — assez long pour être stable
        float burstRatio    = 4.0f;    // seuil de déclenchement
        float minDurationMs = 150.f;   // pulse min 150ms
        float maxDurationMs = 500.f;   // pulse max 500ms
        float refractoryMs  = 700.f;   // 700ms — évite les rebonds, < 1s min entre pulses
    };

    explicit AudioPulseDetector();
    explicit AudioPulseDetector(const Config& cfg);

    bool process(const std::vector<int16_t>& pcm);
    bool process(const std::vector<float>& audio);

    bool  pulseDetected()     const { return pulseConfirmed_; }
    float lastPulseStrength() const { return lastPulseStrength_; }
    float ambientEnergy()     const { return static_cast<float>(energyLong_); }
    float currentRatio()      const { return lastComputedRatio_; }
    void  reset();

private:
    Config cfg_;

    int shortWinSamples_   = 0;
    int minBurstSamples_   = 0;
    int maxBurstSamples_   = 0;
    int refractorySamples_ = 0;

    double energyShort_       = 0.0;
    double energyLong_        = 1e-6;
    int    shortCount_        = 0;
    float  lastComputedRatio_ = 1.0f;

    bool   inBurst_           = false;
    int    burstLen_          = 0;
    int    refractoryLeft_    = 0;
    bool   pulseConfirmed_    = false;
    float  lastPulseStrength_ = 0.f;

    bool processSample(float s);
};