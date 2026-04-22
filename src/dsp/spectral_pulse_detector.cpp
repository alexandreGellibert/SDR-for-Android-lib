#include "spectral_pulse_detector.h"

SpectralPulseDetector::SpectralPulseDetector(const Config& cfg)
    : cfg_(cfg), tTarget_(cfg.tTargetInit) {}

void SpectralPulseDetector::configure(const Config& cfg) {
    cfg_ = cfg;
}

SpectralPulseDetector::PulseLevel SpectralPulseDetector::process(float snrSigma, float freqHz) {
    onEnergyFrame(snrSigma, freqHz);
    return lastLevel_;
}

// ─── Time / index helpers ─────────────────────────────────────────────────────

float SpectralPulseDetector::timeOfIdx(int i) const {
    return eBufT0_ + static_cast<float>(i) / cfg_.fsEnergy;
}

int SpectralPulseDetector::idxOfTime(float t) const {
    int i = static_cast<int>((t - eBufT0_) * cfg_.fsEnergy);
    return std::max(0, std::min(i, static_cast<int>(eBuf_.size()) - 1));
}

// ─── Per energy-frame update ──────────────────────────────────────────────────

void SpectralPulseDetector::onEnergyFrame(float snrSigma, float freqHz) {
    if (eBuf_.empty()) eBufT0_ = 0.f;
    eBuf_.push_back(snrSigma);
    freqBuf_.push_back(freqHz);

    const int maxBuf = static_cast<int>(10.f * cfg_.fsEnergy);
    while (static_cast<int>(eBuf_.size()) > maxBuf) {
        eBuf_.pop_front();
        freqBuf_.pop_front();
        eBufT0_ += 1.f / cfg_.fsEnergy;
        if (lastScanIdx_ > 0) lastScanIdx_--;
    }

    detectRois();
    liveEtat_  = computeLiveEtat();
    lastLevel_ = toLevel(liveEtat_);
}

// ─── ROI detection ────────────────────────────────────────────────────────────

void SpectralPulseDetector::detectRois() {
    const int n = static_cast<int>(eBuf_.size());
    const float z_s   = isLocked_ ? 0.75f * tTarget_ : cfg_.zDefaultS;
    const int   idx_z = std::max(1, static_cast<int>(z_s * cfg_.fsEnergy));

    const int safe_idx = n - idx_z;
    if (safe_idx <= idx_z) return;

    int scan_from = std::max(idx_z, lastScanIdx_);

    for (int i = scan_from; i < safe_idx; i++) {
        const float val = eBuf_[i];

        // ── Local max in [i-idx_z … i+idx_z] ─────────────────────────────
        bool is_max = true;
        for (int j = i - idx_z; j <= i + idx_z && is_max; j++)
            if (j != i && eBuf_[j] >= val) is_max = false;
        if (!is_max) continue;

        // ── SNR threshold (snrSigma is already normalised) ────────────────
        const float snr = val;
        if (snr < cfg_.snrMin) continue;

        // ── dt / N_cycles ─────────────────────────────────────────────────
        const float t_roi  = timeOfIdx(i);
        const float dt     = (tLastRoi_ >= 0.f) ? t_roi - tLastRoi_ : 0.f;
        int   N_cycles = 1;
        float norm_dt  = dt;
        if (dt > 0.f) {
            N_cycles = std::max(1, static_cast<int>(std::round(dt / tTarget_)));
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

        // ── Base state from σ ─────────────────────────────────────────────
        const int base = (snr >= cfg_.snrStrong) ? 5 :
                         (snr >= 3.0f)           ? 4 :
                         (snr >= cfg_.snrRhythm) ? 3 :
                         (snr >= 2.0f)           ? 2 : 1;

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
        if ((int)last3Dts_.size() == 3) {
            const float mn = *std::min_element(last3Dts_.begin(), last3Dts_.end());
            const float mx = *std::max_element(last3Dts_.begin(), last3Dts_.end());
            if (mx - mn < cfg_.dtTolS) pen_confusion = 0;
        }

        const int etat = std::max(0, base - pen_rythme - pen_confusion);

        const float roiFreqHz = freqBuf_[i];
        rois_.push_back({t_roi, etat, roiFreqHz});
        tLastRoi_ = t_roi;
        lastSnr_  = snr;

        freqHistory_.push_back({t_roi, roiFreqHz});
        if ((int)freqHistory_.size() > kFreqHistoryMax) freqHistory_.pop_front();

        const float cutoff = t_roi - 20.f;
        while (!rois_.empty() && rois_.front().t < cutoff)
            rois_.pop_front();

        i += idx_z;
        lastScanIdx_ = i + 1;
    }

    lastScanIdx_ = std::max(lastScanIdx_, safe_idx);
}

// ─── Live état ────────────────────────────────────────────────────────────────

int SpectralPulseDetector::computeLiveEtat() const {
    if (rois_.empty()) return 0;
    const float now          = timeOfIdx(static_cast<int>(eBuf_.size()) - 1);
    const float window_start = now - cfg_.liveWindowT * tTarget_;
    float sum = 0.f;
    for (const auto& r : rois_)
        if (r.t >= window_start) sum += static_cast<float>(r.etat);
    return std::min(5, static_cast<int>(std::floor(sum / cfg_.liveDivisor)));
}

SpectralPulseDetector::PulseLevel SpectralPulseDetector::toLevel(int etat) const {
    if (etat >= 5) return SpectralPulseDetector::PulseLevel::STRONG;
    if (etat >= 3) return SpectralPulseDetector::PulseLevel::MEDIUM;
    if (etat >= 1) return SpectralPulseDetector::PulseLevel::LOW;
    return SpectralPulseDetector::PulseLevel::NONE;
}

// ─── Frequency estimator ─────────────────────────────────────────────────────

float SpectralPulseDetector::estimatedFreqHz() const {
    const int n = static_cast<int>(freqHistory_.size());
    if (n < 2) return 0.f;

    // OLS: f(t) = a*t + b, projected to t_now
    const float t_now = timeOfIdx(static_cast<int>(eBuf_.size()) - 1);

    double sum_t = 0, sum_f = 0, sum_tt = 0, sum_tf = 0;
    for (const auto& s : freqHistory_) {
        sum_t  += s.t;
        sum_f  += s.freqHz;
        sum_tt += s.t * s.t;
        sum_tf += s.t * s.freqHz;
    }
    const double denom = n * sum_tt - sum_t * sum_t;
    if (std::fabs(denom) < 1e-9) return static_cast<float>(sum_f / n);

    const double a = (n * sum_tf - sum_t * sum_f) / denom;
    const double b = (sum_f - a * sum_t) / n;
    return static_cast<float>(a * t_now + b);
}

// ─── Reset ────────────────────────────────────────────────────────────────────

void SpectralPulseDetector::reset() {
    eBuf_.clear();
    freqBuf_.clear();
    eBufT0_    = 0.f;
    rois_.clear();
    tLastRoi_    = -1.f;
    lastScanIdx_ = 0;
    isLocked_  = false;
    tTarget_   = cfg_.tTargetInit;
    last3Dts_.clear();
    histDts_.clear();
    histN_.clear();
    freqHistory_.clear();
    liveEtat_  = 0;
    lastSnr_   = 0.f;
    lastLevel_ = SpectralPulseDetector::PulseLevel::NONE;
}
