#include "audio_pulse_detector.h"

AudioPulseDetector::AudioPulseDetector() : AudioPulseDetector(Config()) {}

AudioPulseDetector::AudioPulseDetector(const Config& cfg) : cfg_(cfg) {
    shortWinSamples_   = static_cast<int>(cfg_.sampleRate * cfg_.shortWindowMs  / 1000.f);
    minBurstSamples_   = static_cast<int>(cfg_.sampleRate * cfg_.minDurationMs  / 1000.f);
    maxBurstSamples_   = static_cast<int>(cfg_.sampleRate * cfg_.maxDurationMs  / 1000.f);
    refractorySamples_ = static_cast<int>(cfg_.sampleRate * cfg_.refractoryMs   / 1000.f);
}

void AudioPulseDetector::reset() {
    energyShort_       = 0.0;
    energyLong_        = 1e-6;
    shortCount_        = 0;
    lastComputedRatio_ = 1.0f;
    inBurst_           = false;
    burstLen_          = 0;
    refractoryLeft_    = 0;
    pulseConfirmed_    = false;
    lastPulseStrength_ = 0.f;
}

bool AudioPulseDetector::processSample(float s) {
    pulseConfirmed_ = false;

    // 1. Période réfractaire — on accumule quand même l'énergie
    //    mais on ne déclenche pas
    bool inRefractory = (refractoryLeft_ > 0);
    if (inRefractory) refractoryLeft_--;

    // 2. Accumulation fenêtre courte
    energyShort_ += static_cast<double>(s * s);
    shortCount_++;

    if (shortCount_ < shortWinSamples_) return false;

    // 3. Fenêtre courte complète — calcul RMS²
    double rmsShort = energyShort_ / shortCount_;
    energyShort_ = 0.0;
    shortCount_  = 0;

    // 4. Mise à jour du ratio caché pour la calibration
    if (energyLong_ > 1e-12) {
        lastComputedRatio_ = static_cast<float>(rmsShort / energyLong_);
    }

    // 5. Mise à jour bruit de fond — uniquement hors burst
    //    (ne pas contaminer le plancher avec le signal de la balise)
    constexpr double alpha = 0.05;
    if (!inBurst_) {
        energyLong_ = (1.0 - alpha) * energyLong_ + alpha * rmsShort;
    }

    // 6. Pas de détection pendant la réfractaire
    if (inRefractory) return false;

    // 7. Décision d'activité
    bool isActive = (rmsShort > cfg_.burstRatio * energyLong_);

    // 8. Détection burst
    if (!inBurst_ && isActive) {
        inBurst_  = true;
        burstLen_ = shortWinSamples_;
    } else if (inBurst_) {
        if (isActive) {
            burstLen_ += shortWinSamples_;
            if (burstLen_ > maxBurstSamples_) {
                // Trop long → signal continu, pas une pulse balise
                inBurst_ = false;
                burstLen_ = 0;
            }
        } else {
            // Fin du burst — valider la durée
            if (burstLen_ >= minBurstSamples_) {
                pulseConfirmed_    = true;
                lastPulseStrength_ = static_cast<float>(
                        10.0 * std::log10(rmsShort / (energyLong_ + 1e-12)));
                refractoryLeft_    = refractorySamples_;
            }
            inBurst_  = false;
            burstLen_ = 0;
        }
    }

    return pulseConfirmed_;
}

bool AudioPulseDetector::process(const std::vector<float>& audio) {
    bool fired = false;
    for (float s : audio) fired |= processSample(s);
    return fired;
}

bool AudioPulseDetector::process(const std::vector<int16_t>& pcm) {
    bool fired = false;
    constexpr float inv = 1.f / 32767.f;
    for (int16_t s : pcm) fired |= processSample(s * inv);
    return fired;
}