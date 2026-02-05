//
// Created by florent on 05/11/2025.
//

//
// ssb_demod_optimized.cpp
//

#include "ssb_demod_high.h"
#include <cmath>
#include <algorithm>
#include <complex>
#include <vector>

// ------------------------------
// 1) Convert IQ 8-bit -> complex<float>
// ------------------------------
void convertIQ(const unsigned char* buffer, int len, std::vector<std::complex<float>>& iq) {
    int sampCount = len / 2;
    iq.resize(sampCount);
    for (int i = 0; i < sampCount; ++i) {
        float I = (buffer[2*i] - 127.4f) / 128.0f;
        float Q = (buffer[2*i+1] - 127.4f) / 128.0f;
        iq[i] = {I, Q};
    }
}

// ------------------------------
// 2) DC removal
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
// 4) Demod SSB
// ------------------------------
void demodSSB(const std::vector<std::complex<float>>& iq, std::vector<float>& audio, bool upper) {
    audio.resize(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        audio[i] = upper ? (iq[i].real() + iq[i].imag())
                         : (iq[i].real() - iq[i].imag());
    }
}

// ------------------------------
// 5) AGC amélioré (attack/decay adaptatifs)
// ------------------------------
void adaptiveAGC(std::vector<float>& audio, float target, float fastAttack, float slowDecay) {
    float gain = 1.0f;
    for (size_t i = 0; i < audio.size(); ++i) {
        float x = audio[i];
        float mag = std::fabs(x) + 1e-8f;
        float desired = target / mag;

        // Attack rapide sur grands écarts, lente sinon
        float rate = (desired < gain) ? fastAttack : slowDecay;
        gain = gain * (1.0f - rate) + desired * rate;

        audio[i] = std::clamp(x * gain, -1.0f, 1.0f);
    }
}

// ------------------------------
// 6) Mini FIR decimation
// ------------------------------
std::vector<float> simpleFIRDecimate(const std::vector<float>& in, int decim, float cutoffRel) {
    int N = 255; // court => léger CPU
    std::vector<float> h(N);
    int M = N - 1;
    float fc = cutoffRel / decim;

    for (int n = 0; n < N; n++) {
        int k = n - M/2;
        float sinc = (k == 0) ? 2*M_PI*fc : sinf(2*M_PI*fc*k)/(float)k;
        float w = 0.5f - 0.5f*cosf(2*M_PI*n/M); // Hanning
        h[n] = (sinc/M_PI)*w;
    }
    float sum=0; for(auto v:h) sum+=v; for(auto &v:h) v/=sum;

    std::vector<float> out; out.reserve(in.size()/decim);
    for (size_t i=0; i+N<=in.size(); i+=decim){
        float acc=0;
        for(int k=0;k<N;k++) acc+=in[i+k]*h[k];
        out.push_back(acc);
    }
    return out;
}

// ------------------------------
// 7) Audio EQ (léger bandpass)
// ------------------------------
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
        float in=x[i];
        float y=f.a0*in + f.a1*z1 + f.a2*z2 - f.b1*z1 - f.b2*z2;
        z2=z1; z1=y;
        x[i]=y;
    }
    f.z1=z1; f.z2=z2;
}

// ------------------------------
// 8) Float->PCM16
// ------------------------------
std::vector<int16_t> floatToPCM(const std::vector<float>& in, float gain){
    std::vector<int16_t> pcm(in.size());
    for(size_t i=0;i<in.size();++i){
        float v=std::clamp(in[i]*gain,-1.0f,1.0f);
        pcm[i]=static_cast<int16_t>(v*32767.0f);
    }
    return pcm;
}

// ------------------------------
// 9) Full SSB pipeline
// ------------------------------
void processSSB_high(const unsigned char *buffer, int len, uint32_t sampleRate,
                bool upperSideband, std::vector<int16_t> &pcmOut, float gain)
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

    // --- 3) RF Lowpass (plus doux)
    static IIR2 rfFilter;
    static bool rfInit = false;
    if (!rfInit) { iir2InitLowpass(rfFilter, (float)sampleRate, 2800.0f, 0.707f); rfInit = true; }
    iir2Process(rfFilter, iq);

    // --- 4) Démodulation SSB
    demodSSB(iq, audio, upperSideband);

    // --- 5) AGC amélioré
    adaptiveAGC(audio, 0.4f, 0.005f, 0.0003f);

    // --- 6) Décimation vers 48 kHz
    int decim = std::max(1, static_cast<int>(sampleRate / 48000.0f));
    auto audio48k = simpleFIRDecimate(audio, decim);

    // --- 7) EQ audio final
    static Biquad b1;
    static bool eqInit = false;
    if (!eqInit){
        biquadInitBandpass(b1, 48000.0f, 1800.0f, 0.7f);
        eqInit = true;
    }
    biquadProcess(b1, audio48k);

    // --- 8) Conversion PCM
    pcmOut = floatToPCM(audio48k, gain);
}

