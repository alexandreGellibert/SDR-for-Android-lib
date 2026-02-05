//
// Created by florent on 13/11/2025.
//

#ifndef SSB_DEMOD_OPT_BOOST_H
#define SSB_DEMOD_OPT_BOOST_H

#include <cstdint>
#include <vector>
#include <complex>

// ----------------------------------------------------
// Structures filtres IIR et Biquad
// ----------------------------------------------------
struct IIR2 {
    float a0 = 0, a1 = 0, a2 = 0;
    float b1 = 0, b2 = 0;
    float z1 = 0, z2 = 0;
};

struct Biquad {
    float a0 = 0, a1 = 0, a2 = 0;
    float b1 = 0, b2 = 0;
    float z1 = 0, z2 = 0;
};

// ----------------------------------------------------
// Fonctions publiques
// ----------------------------------------------------

// 1) Conversion IQ 8-bit -> complex<float>
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq);

// 2) Suppression DC
void removeDC(std::vector<std::complex<float>>& iq, float alpha);

// 3) Filtre passe-bas IIR2
void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q);
void iir2Process(IIR2 &f, std::vector<std::complex<float>> &sig);

// 4) Démod SSB
void demodSSB(const std::vector<std::complex<float>>& iq, std::vector<float>& audio, bool upper);

// 5) AGC amélioré
void adaptiveAGC(std::vector<float>& audio, float target, float fastAttack, float slowDecay);

// 6) Décimation FIR simple
std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel = 0.45f);

// 7) Biquad bandpass audio
void biquadInitHighpass(Biquad& f, float fs, float f0, float Q);
void biquadInitBandpass(Biquad& f, float fs, float f0, float Q);
void biquadProcess(Biquad& f, std::vector<float>& x);

// 8) Transient boost
void transientBoost(std::vector<float>& x, float coeff);

// 9) Conversion Float -> PCM16
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain);

// 10) Détection de pulses audio
bool detectPulse(const std::vector<float>& x, float thresh, int minConsecutive=1) ;

// 11) Pipeline complet
void processSSB_opt_boost(const unsigned char *buffer, int len, uint32_t sampleRate,
                         bool upperSideband, std::vector<int16_t> &pcmOut, float gain,
                         bool sensitivityBoost = false);

// 12) Récupération du dernier état de détection
bool lastBlockContainsPulse();

// 13) Réglages dynamiques
void setPulseThreshold(float thresh);
void setTransientCoeff(float coeff);
void setFIRLength(int N);
void setAGCParams(float target, float fastAttack, float slowDecay);

#endif //SSB_DEMOD_OPT_BOOST_H
