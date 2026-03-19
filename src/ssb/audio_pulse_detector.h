#pragma once
#include <vector>
#include <cstdint>
#include <cmath>

class AudioPulseDetector {
public:
    struct Config {
        float sampleRate      = 48000.f;
        float shortWindowMs   = 20.f;   // fenêtre courte (énergie instantanée)
        float longWindowMs    = 600.f;  // fenêtre longue (bruit ambiant)
        float burstRatio      = 4.0f;   // seuil : énergie_courte > ratio × énergie_longue
        float minDurationMs   = 80.f;   // durée min d'une pulse valide
        float maxDurationMs   = 250.f;  // durée max
        float refractoryMs    = 600.f;  // silence minimum entre deux pulses
    };

    explicit AudioPulseDetector();
    explicit AudioPulseDetector(const Config& cfg);

    // Appeler à chaque bloc PCM audio (int16 ou float)
    // Retourne true si une nouvelle pulse vient d'être confirmée
    bool process(const std::vector<int16_t>& pcm);
    bool process(const std::vector<float>& audio);

    // Accesseurs
    bool pulseDetected()      const { return pulseConfirmed_; }
    float lastPulseStrength() const { return lastPulseStrength_; }
    void reset();

private:
    Config cfg_;

    // Compteurs de samples
    int shortWinSamples_  = 0;
    int minBurstSamples_  = 0;
    int maxBurstSamples_  = 0;
    int refractorySamples_= 0;

    // État interne
    double energyShort_   = 0.0;   // RMS² accumulé fenêtre courte
    double energyLong_    = 1e-6;  // RMS² fenêtre longue (bruit ambiant)
    int    shortCount_    = 0;     // samples dans la fenêtre courte courante
    int    longCount_     = 0;

    bool   inBurst_       = false;
    int    burstLen_      = 0;
    int    refractoryLeft_= 0;
    bool   pulseConfirmed_= false;
    float  lastPulseStrength_ = 0.f;

    bool processSample(float s);
};