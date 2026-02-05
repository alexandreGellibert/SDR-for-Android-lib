//
// Created by florent on 05/11/2025.
//

#ifndef SSB_DEMOD_BOOST_H
#define SSB_DEMOD_BOOST_H

#pragma once
#include <vector>
#include <complex>
#include <cstdint>

// ============================================================
// Structures de filtres IIR / Biquad
// ============================================================

// IIR2 (biquad simple pour filtrage RF)
struct IIR2 {
    float a0, a1, a2;
    float b1, b2;
    float z1, z2;
};

// Biquad (pour shaping audio)
struct Biquad {
    float a0, a1, a2;
    float b1, b2;
    float z1, z2;
};

// ============================================================
// Fonctions utilitaires
// ============================================================

// Conversion IQ 8-bit -> complex<float>
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq);

// Suppression DC lente
void removeDC(std::vector<std::complex<float>>& iq, float alpha);

// IIR2 lowpass init & processing
void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q = 0.707f);
void iir2Process(IIR2 &f, std::vector<std::complex<float>> &sig);

// Démodulation SSB
void demodSSB(const std::vector<std::complex<float>>& iq, std::vector<float>& audio, bool upperSideband);

// AGC adaptatif
void adaptiveAGC(std::vector<float>& audio, float target, float fastAttack, float slowDecay);

// Décimation FIR rapide
std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel = 0.45f);

// Biquad audio
void biquadInitBandpass(Biquad& f, float fs, float f0, float Q);
void biquadProcess(Biquad& f, std::vector<float>& x);

// Conversion float -> PCM16
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain = 1.0f);

// Pipeline complet de traitement SSB
void processSSB_boost(const unsigned char *buffer,
                int len,
                uint32_t sampleRate,
                bool upperSideband,
                std::vector<int16_t> &pcmOut,
                float gain = 1.0f,
                bool sensitivityBoost = false);


#endif //SSB_DEMOD_BOOST_H
