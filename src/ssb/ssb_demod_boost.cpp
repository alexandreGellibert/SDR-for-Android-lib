//
// Created by florent on 05/11/2025.
//
#include "ssb_demod_boost.h"
#include <cmath>
#include <algorithm>

// ============================================================
// 1. IQ 8-bit -> complex<float>
// ============================================================
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq) {
    int sampCount = len / 2;
    iq.resize(sampCount);
    for (int i = 0; i < sampCount; ++i) {
        float I = (buffer[2*i]   - 127.4f) / 128.0f;
        float Q = (buffer[2*i+1] - 127.4f) / 128.0f;
        iq[i] = std::complex<float>(I, Q);
    }
}

// ============================================================
// 2. DC Removal
// ============================================================
void removeDC(std::vector<std::complex<float>>& iq, float alpha) {
    std::complex<float> dc(0.0f, 0.0f);
    for (auto &s : iq) {
        dc = alpha * dc + (1.0f - alpha) * s;
        s -= dc;
    }
}

// ============================================================
// 3. IIR2 Lowpass Init & Process
// ============================================================
void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q) {
    float w0 = 2.0f * M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float norm = 1.0f / (1.0f + alpha);
    f.a0 = (1.0f - cosw0) / 2.0f * norm;
    f.a1 = (1.0f - cosw0) * norm;
    f.a2 = f.a0;
    f.b1 = -2.0f * cosw0 * norm;
    f.b2 = (1.0f - alpha) * norm;
    f.z1 = f.z2 = 0.0f;
}

void iir2Process(IIR2 &f, std::vector<std::complex<float>> &sig) {
    float z1 = f.z1, z2 = f.z2;
    for (auto &s : sig) {
        float x = s.real();
        float y = f.a0*x + f.a1*z1 + f.a2*z2 - f.b1*z1 - f.b2*z2;
        z2 = z1; z1 = y;
        s = std::complex<float>(y, y);
    }
    f.z1 = z1; f.z2 = z2;
}

// ============================================================
// 4. SSB Demodulation
// ============================================================
void demodSSB(const std::vector<std::complex<float>>& iq, std::vector<float>& audio, bool upper) {
    audio.resize(iq.size());
    for (size_t i = 0; i < iq.size(); ++i)
        audio[i] = upper ? iq[i].real() + iq[i].imag()
                         : iq[i].real() - iq[i].imag();
}

// ============================================================
// 5. AGC améliorée avec clamping doux
// ============================================================
void adaptiveAGC(std::vector<float>& audio, float target, float fastAttack, float slowDecay) {
    float gain = 1.0f;
    for (auto &x : audio) {
        float mag = std::fabs(x) + 1e-9f;
        float desired = target / mag;
        if (desired < gain)
            gain = gain*(1.0f - fastAttack) + desired*fastAttack;
        else
            gain = gain*(1.0f - slowDecay) + desired*slowDecay;

        x = std::clamp(x * gain, -1.0f, 1.0f);
        if (std::fabs(x) < 0.003f) x *= 0.2f; // atténue le souffle
    }
}

// ============================================================
// 6. Simple FIR Decimator
// ============================================================
std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel) {
    int N = 63;
    std::vector<float> h(N);
    int M = N - 1;
    float fc = cutoffRel / decim;
    for (int n = 0; n < N; n++) {
        int k = n - M/2;
        float sinc = (k == 0) ? 2*M_PI*fc : sinf(2*M_PI*fc*k)/(float)k;
        float w = 0.5f - 0.5f*cosf(2*M_PI*n/M);
        h[n] = (sinc/M_PI)*w;
    }
    float sum = 0; for (auto v : h) sum += v; for (auto &v : h) v /= sum;

    std::vector<float> out; out.reserve(in.size()/decim);
    for (size_t i=0; i+N<=in.size(); i+=decim){
        float acc=0;
        for(int k=0;k<N;k++) acc+=in[i+k]*h[k];
        out.push_back(acc);
    }
    return out;
}

// ============================================================
// 7. Biquad Bandpass
// ============================================================
void biquadInitBandpass(Biquad& f, float fs, float f0, float Q) {
    float w0 = 2.0f * M_PI * f0 / fs;
    float alpha = sinf(w0)/(2.0f*Q);
    float cosw0 = cosf(w0);
    float b0=alpha, b1=0.0f, b2=-alpha;
    float a0=1.0f+alpha, a1=-2.0f*cosw0, a2=1.0f-alpha;
    f.a0=b0/a0; f.a1=b1/a0; f.a2=b2/a0;
    f.b1=a1/a0; f.b2=a2/a0;
    f.z1=f.z2=0.0f;
}

void biquadProcess(Biquad& f, std::vector<float>& x) {
    float z1=f.z1, z2=f.z2;
    for (auto &in : x) {
        float y = f.a0*in + f.a1*z1 + f.a2*z2 - f.b1*z1 - f.b2*z2;
        z2=z1; z1=y;
        in=y;
    }
    f.z1=z1; f.z2=z2;
}

// ============================================================
// 8. Float -> PCM16
// ============================================================
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain) {
    std::vector<int16_t> pcm(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        float v = std::clamp(in[i]*gain, -1.0f, 1.0f);
        pcm[i] = static_cast<int16_t>(v * 32767.0f);
    }
    return pcm;
}

// ============================================================
// 9. Full Processing Pipeline
// ============================================================
void processSSB_boost(const unsigned char *buffer, int len, uint32_t sampleRate,
                bool upperSideband, std::vector<int16_t> &pcmOut,
                float gain, bool sensitivityBoost)
{
    int sampCount = len / 2;
    static std::vector<std::complex<float>> iq;
    static std::vector<float> audio;
    iq.resize(sampCount);
    audio.resize(sampCount);

    // --- Étape 1 : IQ conversion ---
    convertIQ(buffer, len, iq);

    // --- Étape 2 : DC removal ---
    removeDC(iq, 0.9995f);

    // --- Étape 3 : RF filtering ---
    static IIR2 rfFilter;
    static bool rfInit = false;
    if (!rfInit) {
        iir2InitLowpass(rfFilter, (float)sampleRate, sensitivityBoost ? 2800.0f : 2000.0f);
        rfInit = true;
    }
    iir2Process(rfFilter, iq);

    // --- Étape 4 : SSB demod ---
    demodSSB(iq, audio, upperSideband);

    // --- Étape 5 : AGC ---
    if (sensitivityBoost)
        adaptiveAGC(audio, 0.7f, 0.008f, 0.0002f);
    else
        adaptiveAGC(audio, 0.5f, 0.005f, 0.0005f);

    // --- Étape 6 : Décimation ---
    int decim = std::max(1, static_cast<int>(sampleRate / 48000u));
    auto audio48k = simpleFIRDecimate(audio, decim, 0.45f);

    // --- Étape 7 : audio shaping ---
    static Biquad a1, a2;
    static bool audioInit = false;
    if (!audioInit) {
        biquadInitBandpass(a1, 48000.0f, 1000.0f, 0.7f);
        biquadInitBandpass(a2, 48000.0f, 2200.0f, 0.7f);
        audioInit = true;
    }
    biquadProcess(a1, audio48k);
    biquadProcess(a2, audio48k);

    // --- Étape 8 : sortie PCM ---
    pcmOut = floatToPCM(audio48k, gain);
}
