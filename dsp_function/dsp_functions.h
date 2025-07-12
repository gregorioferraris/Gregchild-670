#ifndef GREGCHILD_H
#define GREGCHILD_H

// ... (existing includes) ...

#include "dsp_functions/dsp_functions.h"

// ... (GREGCHILD_URI define) ...

class Gregchild {
public:
    // ... (constructor, destructor, activate, deactivate, run, connect_port) ...

private:
    // Pointers to port data
    // ... (existing port pointers) ...

    const float *vca_soft_knee_on_param; // Nuovo
    const float *vca_knee_width_param;   // Nuovo

    // ... (existing output peak pointers) ...

    double sample_rate;

    // Stato dei compressori
    Gua76DSP::OptoCellState opto_state;
    Gua76DSP::JfetCompressorState jfet_state;
    Gua76DSP::VcaCompressorState vca_state; // Nuovo

    // ... (existing VariMuProgramParams and fairchild_programs) ...

    // ... (existing meter smoothing states) ...

    // ... (existing last_peak_l/r) ...

    // Funzioni helper
    // Per un compressore VCA, potremmo usare un detector RMS più specifico
    float calculate_rms_level(float sample_l, float sample_r, float alpha_attack, float alpha_release); // Nuovo helper
    float process_sidechain(float input_l, float input_r, float sidechain_l, float sidechain_r); // Questo userà il calculate_rms_level o peak
    void update_meter(float &meter_value, float current_peak, float decay_coeff);
};

// Enumerazione delle porte LV2 (DEVE corrispondere a gua76.ttl)
enum PortIndex {
    AUDIO_IN_L = 0,
    // ... (existing audio and control ports up to FAIRCHILD_PROGRAM = 29) ...

    COMPRESSOR_MODE = 28, // Era 28
    FAIRCHILD_PROGRAM = 29, // Era 29

    VCA_SOFT_KNEE_ON = 30, // Nuovo
    VCA_KNEE_WIDTH = 31,   // Nuovo

    PEAK_GR = 32, // Output (adattare gli indici, erano da 30 in poi)
    PEAK_IN_L = 33,
    PEAK_IN_R = 34,
    PEAK_OUT_L = 35,
    PEAK_OUT_R = 36
};

#endif // GREGCHILD_H
