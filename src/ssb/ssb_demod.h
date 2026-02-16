//
// Created by florent on 27/10/2025.
//

#ifndef SSB_DEMOD_H
#define SSB_DEMOD_H

#include <vector>
#include <complex>
#include <cstdint>
#include <string>

// 2nd-order IIR RF filter
struct IIR2 {
    float a0, a1, a2, b1, b2;
    float z1 = 0.0f, z2 = 0.0f;
};

// Biquad audio filter
struct Biquad {
    float a0, a1, a2, b1, b2;
    float z1 = 0.0f, z2 = 0.0f;
};

// ------------------------------
// Core smart SSB processing
// ------------------------------
void processSSB(const unsigned char *buffer, int len, uint32_t sampleRate,
                                   bool upperSideband, std::vector<int16_t> &pcmOut, float gain = 0.9f);

// ------------------------------
// Utility helpers
// ------------------------------
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq);
void removeDC(std::vector<std::complex<float>>& iq, float alpha = 0.999f);

void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q = 0.707f);
void iir2Process(IIR2 &f, std::vector<std::complex<float>> &sig);

std::vector<float> demodSSB(const std::vector<std::complex<float>>& sig, bool upperSideband);
void fastAGC(std::vector<float>& audio, float target = 0.25f, float attack = 0.005f, float decay = 0.999f);

void biquadInitBandpass(Biquad& f, float fs, float f0, float Q);
void biquadProcess(Biquad& f, std::vector<float>& x);

std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel = 0.45f);
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain = 0.9f);

#endif // SSB_DEMOD_H