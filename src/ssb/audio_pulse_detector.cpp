#include "audio_pulse_detector.h"
#include <numeric>

static constexpr float kPi = 3.14159265358979f;

// ─── Constructor ──────────────────────────────────────────────────────────────

AudioPulseDetector::AudioPulseDetector() : AudioPulseDetector(Config{}) {}

AudioPulseDetector::AudioPulseDetector(const Config& cfg)
    : cfg_(cfg), tTarget_(cfg.tTargetInit)
{
    // Bandpass = 2nd-order HP at fMin (rolls off < 1500 Hz)
    //          + 2nd-order LP at fMax (rolls off > 4000 Hz)
    bandSOS_.push_back(makeHP2(cfg_.sampleRate, cfg_.fMin));
    bandSOS_.push_back(makeLP2(cfg_.sampleRate, cfg_.fMax));

    // Energy envelope smoothing: 2nd-order LP at smoothCutoff Hz
    // — applied to the 100-Hz energy signal, so fs = fsEnergy here
    lowSOS_.push_back(makeLP2(cfg_.fsEnergy, cfg_.smoothCutoff));

    frameSamples_ = std::max(1, static_cast<int>(cfg_.sampleRate / cfg_.fsEnergy));
}

// ─── Biquad design (bilinear-transform Butterworth, Direct Form II) ───────────

AudioPulseDetector::SOS AudioPulseDetector::makeLP2(float fs, float fc, float Q) {
    const float K    = std::tan(kPi * fc / fs);
    const float K2   = K * K;
    const float norm = K2 + K / Q + 1.f;
    SOS s;
    s.b0 =  K2 / norm;
    s.b1 =  2.f * K2 / norm;
    s.b2 =  K2 / norm;
    s.a1 =  2.f * (K2 - 1.f) / norm;
    s.a2 = (K2 - K / Q + 1.f) / norm;
    return s;
}

AudioPulseDetector::SOS AudioPulseDetector::makeHP2(float fs, float fc, float Q) {
    const float K    = std::tan(kPi * fc / fs);
    const float K2   = K * K;
    const float norm = K2 + K / Q + 1.f;
    SOS s;
    s.b0 =  1.f / norm;
    s.b1 = -2.f / norm;
    s.b2 =  1.f / norm;
    s.a1 =  2.f * (K2 - 1.f) / norm;
    s.a2 = (K2 - K / Q + 1.f) / norm;
    return s;
}

float AudioPulseDetector::applyChain(std::vector<SOS>& chain, float x) {
    for (auto& s : chain) {
        const float y = s.b0 * x + s.z1;
        s.z1 = s.b1 * x - s.a1 * y + s.z2;
        s.z2 = s.b2 * x - s.a2 * y;
        x = y;
    }
    return x;
}

// ─── Time / index helpers ─────────────────────────────────────────────────────

float AudioPulseDetector::timeOfIdx(int i) const {
    return eBufT0_ + static_cast<float>(i) / cfg_.fsEnergy;
}

int AudioPulseDetector::idxOfTime(float t) const {
    int i = static_cast<int>((t - eBufT0_) * cfg_.fsEnergy);
    return std::max(0, std::min(i, static_cast<int>(eBuf_.size()) - 1));
}

// Trailing noise reference: mean energy from [i-noiseRefFar .. i-noiseRefNear)
float AudioPulseDetector::noiseRef(int i) const {
    int far  = i - cfg_.noiseRefFar;
    int near = i - cfg_.noiseRefNear;
    if (near <= 0 || far >= near) return -1.f;  // not enough history
    far  = std::max(far,  0);
    near = std::min(near, static_cast<int>(eBuf_.size()));
    if (far >= near) return -1.f;

    float sum = 0.f;
    for (int j = far; j < near; j++) sum += eBuf_[j];
    return sum / static_cast<float>(near - far);
}

// ─── Public process ───────────────────────────────────────────────────────────

AudioPulseDetector::PulseLevel
AudioPulseDetector::process(const std::vector<float>& audio) {
    for (float s : audio) {
        const float filtered = applyChain(bandSOS_, s);
        frameAcc_  += filtered * filtered;
        frameCount_++;
        if (frameCount_ >= frameSamples_) {
            onEnergyFrame(std::sqrt(frameAcc_ / frameSamples_));
            frameAcc_  = 0.f;
            frameCount_ = 0;
        }
    }
    return lastLevel_;
}

AudioPulseDetector::PulseLevel
AudioPulseDetector::process(const std::vector<int16_t>& pcm) {
    constexpr float inv = 1.f / 32767.f;
    for (int16_t s : pcm) {
        const float filtered = applyChain(bandSOS_, s * inv);
        frameAcc_  += filtered * filtered;
        frameCount_++;
        if (frameCount_ >= frameSamples_) {
            onEnergyFrame(std::sqrt(frameAcc_ / frameSamples_));
            frameAcc_  = 0.f;
            frameCount_ = 0;
        }
    }
    return lastLevel_;
}

// ─── Per energy-frame update ──────────────────────────────────────────────────

void AudioPulseDetector::onEnergyFrame(float rms) {
    const float smoothed = applyChain(lowSOS_, rms);

    if (eBuf_.empty()) eBufT0_ = 0.f;
    eBuf_.push_back(smoothed);

    // Keep at most 10 s of energy history (1000 frames @ 100 Hz)
    const int maxBuf = static_cast<int>(10.f * cfg_.fsEnergy);
    while (static_cast<int>(eBuf_.size()) > maxBuf) {
        eBuf_.pop_front();
        eBufT0_ += 1.f / cfg_.fsEnergy;
        if (lastScanIdx_ > 0) lastScanIdx_--;
    }

    detectRois();

    liveEtat_  = computeLiveEtat();
    lastLevel_ = toLevel(liveEtat_);
}

// ─── ROI detection ────────────────────────────────────────────────────────────

void AudioPulseDetector::detectRois() {
    const int n = static_cast<int>(eBuf_.size());
    const float z_s   = isLocked_ ? 0.75f * tTarget_ : cfg_.zDefaultS;
    const int   idx_z = std::max(1, static_cast<int>(z_s * cfg_.fsEnergy));

    // Only scan where the full ±idx_z window is available
    const int safe_idx = n - idx_z;
    if (safe_idx <= idx_z) return;

    // Start from where we last left off — never re-scan old data, never miss new data.
    // lastScanIdx_ is decremented when the ring buffer trims from the front,
    // so it always points to the correct position relative to the current buffer.
    int scan_from = std::max(idx_z, lastScanIdx_);

    for (int i = scan_from; i < safe_idx; i++) {
        const float val = eBuf_[i];

        // ── Local max in [i-idx_z … i+idx_z] ─────────────────────────────
        bool is_max = true;
        for (int j = i - idx_z; j <= i + idx_z && is_max; j++)
            if (j != i && eBuf_[j] >= val) is_max = false;
        if (!is_max) continue;

        // ── SNR against trailing noise reference ──────────────────────────
        const float noise = noiseRef(i);
        if (noise <= 0.f) continue;
        const float snr = val / noise;
        if (snr < cfg_.snrMin) continue;

        // ── dt / N_cycles ─────────────────────────────────────────────────
        const float t_roi  = timeOfIdx(i);
        const float dt     = (tLastRoi_ >= 0.f) ? t_roi - tLastRoi_ : 0.f;
        int   N_cycles = 1;
        float norm_dt  = dt;
        if (dt > 0.f) {
            N_cycles = std::max(1, static_cast<int>(std::round(dt / tTarget_)));
            // Reject N_cycles > 1 if residual is too large
            if (N_cycles > 1 && std::fabs(dt - N_cycles * tTarget_) > cfg_.dtTolS)
                N_cycles = 1;
            norm_dt = dt / static_cast<float>(N_cycles);
        }
        const bool dans_rythme = (dt > 0.f) &&
                                 (std::fabs(norm_dt - tTarget_) < cfg_.dtTolS);

        // ── Admission ─────────────────────────────────────────────────────
        const bool admis = (snr >= cfg_.snrStrong) ||
                           (snr >= cfg_.snrRhythm && dans_rythme);
        if (!admis) continue;

        // ── Phase lock ────────────────────────────────────────────────────
        if (dt > 0.f) {
            last3Dts_.push_back(norm_dt);
            if ((int)last3Dts_.size() > 3) last3Dts_.pop_front();
            if ((int)last3Dts_.size() == 3) {
                const float mn = *std::min_element(last3Dts_.begin(), last3Dts_.end());
                const float mx = *std::max_element(last3Dts_.begin(), last3Dts_.end());
                if (mx - mn < cfg_.dtTolS) {
                    isLocked_ = true;
                    tTarget_  = (last3Dts_[0] + last3Dts_[1] + last3Dts_[2]) / 3.f;
                }
            }
            histDts_.push_back(norm_dt);
            if ((int)histDts_.size() > 5) histDts_.pop_front();
            histN_.push_back(N_cycles);
            if ((int)histN_.size() > 5) histN_.pop_front();
        }

        // ── Base state from SNR ───────────────────────────────────────────
        const int base = (snr >= 2.0f) ? 5 :
                         (snr >= 1.5f) ? 4 :
                         (snr >= 1.2f) ? 3 :
                         (snr >= 1.1f) ? 2 : 1;

        // ── Rhythm penalty ────────────────────────────────────────────────
        const int pen_rythme = (dt > 0.f && !dans_rythme) ? 2 : 0;

        // ── Confusion penalty ─────────────────────────────────────────────
        int pen_confusion = 0;
        if ((int)histDts_.size() >= 4) {
            float disp = 0.f;
            for (int j = 1; j < (int)histDts_.size(); j++)
                disp += std::fabs(histDts_[j] - histDts_[j-1]);
            int sumN = 0;
            for (int nv : histN_) sumN += nv;
            if (disp > cfg_.dispersionMax || sumN > cfg_.sumNMax)
                pen_confusion = 2;
        }
        // Locked 3 dts → clear confusion
        if ((int)last3Dts_.size() == 3) {
            const float mn = *std::min_element(last3Dts_.begin(), last3Dts_.end());
            const float mx = *std::max_element(last3Dts_.begin(), last3Dts_.end());
            if (mx - mn < cfg_.dtTolS) pen_confusion = 0;
        }

        const int etat = std::max(0, base - pen_rythme - pen_confusion);

        rois_.push_back({t_roi, etat});
        tLastRoi_ = t_roi;
        lastSnr_  = snr;

        // Trim ROIs older than 20 s
        const float cutoff = t_roi - 20.f;
        while (!rois_.empty() && rois_.front().t < cutoff)
            rois_.pop_front();

        i += idx_z;  // skip past the zone to avoid double-detection
        lastScanIdx_ = i + 1;  // +1 because the for loop will do i++ next
    }

    // Mark everything up to safe_idx as scanned so the next call starts fresh
    lastScanIdx_ = std::max(lastScanIdx_, safe_idx);
}

// ─── Live état ────────────────────────────────────────────────────────────────

int AudioPulseDetector::computeLiveEtat() const {
    if (rois_.empty()) return 0;
    const float now          = timeOfIdx(static_cast<int>(eBuf_.size()) - 1);
    const float window_start = now - cfg_.liveWindowT * tTarget_;
    float sum = 0.f;
    for (const auto& r : rois_)
        if (r.t >= window_start) sum += static_cast<float>(r.etat);
    return std::min(5, static_cast<int>(std::floor(sum / cfg_.liveDivisor)));
}

AudioPulseDetector::PulseLevel AudioPulseDetector::toLevel(int etat) const {
    if (etat >= 5) return PulseLevel::STRONG;
    if (etat >= 3) return PulseLevel::MEDIUM;
    if (etat >= 1) return PulseLevel::LOW;
    return PulseLevel::NONE;
}

// ─── Reset ────────────────────────────────────────────────────────────────────

void AudioPulseDetector::reset() {
    for (auto& s : bandSOS_) { s.z1 = s.z2 = 0.f; }
    for (auto& s : lowSOS_)  { s.z1 = s.z2 = 0.f; }
    frameCount_ = 0;
    frameAcc_   = 0.f;
    eBuf_.clear();
    eBufT0_    = 0.f;
    rois_.clear();
    tLastRoi_    = -1.f;
    lastScanIdx_ = 0;
    isLocked_  = false;
    tTarget_   = cfg_.tTargetInit;
    last3Dts_.clear();
    histDts_.clear();
    histN_.clear();
    liveEtat_  = 0;
    lastSnr_   = 0.f;
    lastLevel_ = PulseLevel::NONE;
}
