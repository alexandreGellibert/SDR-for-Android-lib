//
// Created by florent on 05/11/2025.
//

#ifndef SSB_DEMOD_HIGH_H
#define SSB_DEMOD_HIGH_H


#include <vector>
#include <complex>
#include <cstdint>

// ============================================================
// Structures internes de filtres
// ============================================================

struct IIR2 {
    float a0, a1, a2;
    float b1, b2;
    float z1, z2;
};

struct Biquad {
    float a0, a1, a2;
    float b1, b2;
    float z1, z2;
};

// ============================================================
// Fonctions utilitaires
// ============================================================

/**
 * Convertit un buffer IQ (unsigned char I/Q alternés) en complexe float.
 */
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq);

/**
 * Supprime la composante DC du signal I/Q (filtre passe-haut adaptatif).
 * @param alpha : facteur de lissage (0.99 - 0.9999)
 */
void removeDC(std::vector<std::complex<float>>& iq, float alpha = 0.9995f);

/**
 * Initialise un filtre IIR2 passe-bas.
 */
void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q = 0.707f);

/**
 * Applique le filtre IIR2 à un signal complexe.
 */
void iir2Process(IIR2 &f, std::vector<std::complex<float>> &sig);

/**
 * Démodule une bande SSB (upper ou lower sideband).
 */
void demodSSB(const std::vector<std::complex<float>>& iq,
              std::vector<float>& audio,
              bool upper);

/**
 * AGC adaptatif (attaque rapide / décroissance lente)
 */
void adaptiveAGC(std::vector<float>& audio, float target = 0.6f,
                 float fastAttack = 0.005f, float slowDecay = 0.0003f);

/**
 * Décimation FIR simple (filtre court type Hanning)
 */
std::vector<float> simpleFIRDecimate(const std::vector<float>& in,
                                     int decim,
                                     float cutoffRel = 0.45f);

/**
 * Initialise un biquad passe-bande pour l'audio.
 */
void biquadInitBandpass(Biquad& f, float fs, float f0, float Q);

/**
 * Applique le biquad à un signal float.
 */
void biquadProcess(Biquad& f, std::vector<float>& x);

/**
 * Conversion float (-1..1) -> PCM 16 bits.
 */
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain);

/**
 * Fonction principale de traitement SSB :
 * - Démodulation SSB
 * - Filtrage RF et audio
 * - AGC
 * - Décimation
 * - Conversion PCM
 */
void processSSB_high(const unsigned char *buffer,
                int len,
                uint32_t sampleRate,
                bool upperSideband,
                std::vector<int16_t> &pcmOut,
                float gain = 1.0f);

#endif // SSB_DEMOD_HIGH_H
