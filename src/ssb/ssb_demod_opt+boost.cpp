//
// Created by florent on 13/11/2025.
//
#include "ssb_demod_opt+boost.h"
#include <cmath>
#include <algorithm>
#include <complex>
#include <numeric>

// ------------------------------
// Internal globals for detector state
// ------------------------------
static bool g_lastPulseDetected = false;
static float g_pulseThreshold = 0.08f;
static float g_transientCoeff = 0.55f;
static int g_firLength = 127;
static float g_agcTarget = 0.35f;
static float g_agcFast = 0.006f;
static float g_agcSlow = 0.00035f;

// ------------------------------
// 1) Convert IQ 8-bit -> complex<float>
// ------------------------------
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq) {
    int sampCount = len / 2;
    iq.resize(sampCount);
    // center at ~0.0, scale to [-1,1]
    const float offset = 127.4f;
    const float scale = 1.0f/128.0f;
    for (int i = 0; i < sampCount; ++i) {
        float I = (buffer[2*i] - offset) * scale;
        float Q = (buffer[2*i+1] - offset) * scale;
        iq[i] = {I, Q};
    }
}

// ------------------------------
// 2) DC removal (IIR 1st order)
// ------------------------------
void removeDC(std::vector<std::complex<float>>& iq, float alpha) {
    std::complex<float> dc = {0.0f, 0.0f};
    for (auto &s : iq) {
        dc = alpha * dc + (1.0f - alpha) * s;
        s -= dc;
    }
}

// ------------------------------
// 3) IIR2 Lowpass
// ------------------------------
void iir2InitLowpass(IIR2 &f, float fs, float fc, float Q ) {
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
    float z1r = f.z1, z2r = f.z2;
    for (size_t i = 0; i < sig.size(); ++i) {
        float x = sig[i].real();
        float y = f.a0*x + f.a1*z1r + f.a2*z2r - f.b1*z1r - f.b2*z2r;
        z2r = z1r; z1r = y;
        sig[i] = {y, y};
    }
    f.z1 = z1r; f.z2 = z2r;
}

// ------------------------------
// 4) Demod SSB (IQ -> audio)
// ------------------------------
void demodSSB(const std::vector<std::complex<float>>& iq, std::vector<float>& audio, bool upper) {
    audio.resize(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        // simple phasing method: I +/- Q
        audio[i] = upper ? (iq[i].real() + iq[i].imag())
                         : (iq[i].real() - iq[i].imag());
    }
}

// ------------------------------
// 5) AGC amélioré (attack/decay adaptatifs) avec racine carrée pour réduire le souffle
// ------------------------------
void adaptiveAGC(std::vector<float>& audio, float target, float fastAttack, float slowDecay) {
    float gain = 1.0f;
    for (size_t i = 0; i < audio.size(); ++i) {
        float x = audio[i];
        float mag = std::fabs(x) + 1e-8f;
        // use sqrt to reduce heavy amplification of noise
        float desired = target / (std::sqrt(mag) + 1e-6f);

        // Attack rapide sur grands écarts, lente sinon
        float rate = (desired < gain) ? fastAttack : slowDecay;
        gain = gain * (1.0f - rate) + desired * rate;

        audio[i] = std::clamp(x * gain, -1.0f, 1.0f);
    }
}

// ------------------------------
// 6) Mini FIR decimation (reduced N for better impulse preservation)
//    returns audio decimated to approximately (sampleRate/decim) // normal N 63 boost 127
// ------------------------------
std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel) {
    int N = 127; // reduced from 255 to preserve transients and cut CPU
    if (N > (int)in.size()) N = (int)in.size() | 1; // ensure odd
    std::vector<float> h(N);
    int M = N - 1;
    float fc = cutoffRel / decim; // relative

    for (int n = 0; n < N; n++) {
        int k = n - M/2;
        float sinc = (k == 0) ? 2.0f * M_PI * fc : sinf(2.0f * M_PI * fc * k) / (float)k;
        float w = 0.5f - 0.5f * cosf(2.0f * M_PI * n / M); // Hanning
        h[n] = (sinc / M_PI) * w;
    }
    float sum = 0.0f; for (auto v : h) sum += v; if (sum != 0.0f) for (auto &v : h) v /= sum;

    std::vector<float> out; out.reserve(in.size() / decim + 4);
    for (size_t i = 0; i + N <= in.size(); i += decim) {
        float acc = 0.0f;
        for (int k = 0; k < N; k++) acc += in[i + k] * h[k];
        out.push_back(acc);
    }
    return out;
}

// ------------------------------
// 7) Audio EQ helpers: HP and BP
// ------------------------------
void biquadInitHighpass(Biquad& f, float fs, float f0, float Q) {
    float w0 = 2.0f * M_PI * f0 / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float b0 = (1 + cosw0) / 2.0f;
    float b1 = -(1 + cosw0);
    float b2 = (1 + cosw0) / 2.0f;
    float a0 = 1 + alpha;
    float a1 = -2 * cosw0;
    float a2 = 1 - alpha;

    f.a0 = b0 / a0; f.a1 = b1 / a0; f.a2 = b2 / a0;
    f.b1 = a1 / a0; f.b2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

void biquadInitBandpass(Biquad& f, float fs, float f0, float Q) {
    float w0 = 2.0f*M_PI*f0/fs;
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
    for (size_t i=0;i<x.size();++i){
        float in = x[i];
        float y = f.a0*in + f.a1*z1 + f.a2*z2 - f.b1*z1 - f.b2*z2;
        z2 = z1; z1 = y;
        x[i] = y;
    }
    f.z1 = z1; f.z2 = z2;
}

// ------------------------------
// 8) Transient boost (emphase sur les fronts rapides)
// ------------------------------
void transientBoost(std::vector<float>& x, float coeff) {
    float prev = 0.0f;
    for (size_t i=0;i<x.size();++i){
        float diff = x[i] - prev;
        prev = x[i];
        x[i] = x[i] + coeff * diff;
    }
}

// ------------------------------
// 9) Float->PCM16
// ------------------------------
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain){
    std::vector<int16_t> pcm(in.size());
    for(size_t i=0;i<in.size();++i){
        float v = std::clamp(in[i]*gain, -1.0f, 1.0f);
        pcm[i] = static_cast<int16_t>(v * 32767.0f);
    }
    return pcm;
}

// ------------------------------
// 10) Simple pulse detector (lightweight)
// ------------------------------
bool detectPulse(const std::vector<float>& x, float thresh, int minConsecutive) {
    int cnt=0;
    for (float v : x) {
        if (std::fabs(v) > thresh) {
            if (++cnt >= minConsecutive) return true;
        } else cnt = 0;
    }
    return false;
}

// ------------------------------
// 11) Full SSB pipeline (optimized)
// ------------------------------
void processSSB_opt_boost(const unsigned char *buffer, int len, uint32_t sampleRate,
                         bool upperSideband, std::vector<int16_t> &pcmOut, float gain,
                         bool sensitivityBoost)
{
    int sampCount = len / 2;
    static std::vector<std::complex<float>> iq;
    static std::vector<float> audio;
    iq.resize(sampCount);
    audio.resize(sampCount);

    // --- 1) Conversion IQ
    convertIQ(buffer, len, iq);

    // --- 2) DC removal
    removeDC(iq, 0.9995f);

    // --- 3) RF Lowpass (slightly wider to preserve pulses) // normal 3.2khz Q 0.9 boost 2.2 khz Q 1.2
    static IIR2 rfFilter;
    static bool rfInit = false;
    if (!rfInit) {
        iir2InitLowpass(rfFilter, (float)sampleRate,
                        sensitivityBoost ? 1200.0f : 3200.0f,
                        sensitivityBoost ? 1.2f : 0.9f);
        rfInit = true;
    }
    iir2Process(rfFilter, iq);

    // --- 4) Démodulation SSB
    demodSSB(iq, audio, upperSideband);

    // --- 5) AGC amélioré // normal AGCtarget 0.35 AGCattack 0.006 boost AGCtarget 0.45 AGCattack 0.008
    adaptiveAGC(audio,
                sensitivityBoost ? 0.45f : g_agcTarget,
                sensitivityBoost ? 0.008f : g_agcFast,
                sensitivityBoost ? 0.00035f : g_agcSlow);

    // --- 6) Décimation vers ~48 kHz // modif FIR normal 63 boost 127
    int decim = std::max(1, static_cast<int>(sampleRate / 48000.0f));
    auto audio48k = simpleFIRDecimate(audio, decim,0.45f);

    // --- 7) EQ audio final: HP then BP to isolate clicks //AQ center normal 2.4khz boost 2.3khz
    static Biquad hp; static Biquad bp; static bool eqInit = false;
    if (!eqInit){
        biquadInitHighpass(hp, 48000.0f, 1200.0f, 0.7f); // cut below 1.2kHz
        biquadInitBandpass(bp, 48000.0f, 2400.0f, 0.6f); // emphasise 2.4kHz
        eqInit = true;
    }
    if (!audio48k.empty()) {
        biquadProcess(hp, audio48k);
        biquadProcess(bp, audio48k);

        // --- transient boost to highlight sharp edges // normal 0.55 boost 0.7
        transientBoost(audio48k, sensitivityBoost ? 0.7f : g_transientCoeff);
    }

    // --- 10) pulse detection for lastBlock (lightweight)
    g_lastPulseDetected = detectPulse(audio48k, 0.08f, 2);

    // --- 9) Conversion PCM
    pcmOut = floatToPCM(audio48k, gain);
}

// ----------------------------------------------------
// Fonctions d’accès / réglage
// ----------------------------------------------------
bool lastBlockContainsPulse() { return g_lastPulseDetected; }

void setPulseThreshold(float thresh) { g_pulseThreshold = thresh; }
void setTransientCoeff(float coeff) { g_transientCoeff = coeff; }
void setFIRLength(int N) { if (N > 15 && (N % 2 == 1)) g_firLength = N; }
void setAGCParams(float target, float fastAttack, float slowDecay) {
    g_agcTarget = target;
    g_agcFast = fastAttack;
    g_agcSlow = slowDecay;
}
