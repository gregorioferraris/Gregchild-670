#include "gua76.h"

// Implementazione del Costruttore
Gregchild::Gregchild(double sr, const LV2_Feature *const *features)
    : sample_rate(sr),
      gr_smooth_meter(0.0f), in_l_smooth_meter(0.0f), in_r_smooth_meter(0.0f),
      out_l_smooth_meter(0.0f), out_r_smooth_meter(0.0f),
      last_peak_l(0.0f), last_peak_r(0.0f)
{
    jfet_state.setSampleRate(sample_rate);
    vca_state.setSampleRate(sample_rate); // Inizializza lo stato del compressore VCA

    // Inizializzazione dei programmi Fairchild (Vari-Mu)
    // ... (existing Fairchild programs) ...

    // Qui potresti inizializzare i coefficienti per OptoCellState e VcaCompressorState
    // ...
}

// Implementazione di activate()
void Gregchild::activate() {
    // Reset dello stato al momento dell'attivazione
    // ... (existing meter and peak resets) ...

    // Reset degli stati dei compressori
    opto_state = Gua76DSP::OptoCellState();
    jfet_state = Gua76DSP::JfetCompressorState();
    jfet_state.setSampleRate(sample_rate);
    vca_state = Gua76DSP::VcaCompressorState(); // Reset VCA state
    vca_state.setSampleRate(sample_rate); // Ensure sample rate is set
}

// Implementazione di connect_port()
void Gregchild::connect_port(uint32_t port, void *data) {
    switch (static_cast<PortIndex>(port)) {
        // ... (existing connections) ...

        case COMPRESSOR_MODE:           compressor_mode_param = static_cast<const float *>(data); break;
        case FAIRCHILD_PROGRAM:         fairchild_program_param = static_cast<const float *>(data); break;

        case VCA_SOFT_KNEE_ON:          vca_soft_knee_on_param = static_cast<const float *>(data); break; // Nuovo
        case VCA_KNEE_WIDTH:            vca_knee_width_param = static_cast<const float *>(data); break;   // Nuovo

        case PEAK_GR:                   peak_gr_output = static_cast<float *>(data); break; // Indici aggiornati
        case PEAK_IN_L:                 peak_in_l_output = static_cast<float *>(data); break;
        case PEAK_IN_R:                 peak_in_r_output = static_cast<float *>(data); break;
        case PEAK_OUT_L:                peak_out_l_output = static_cast<float *>(data); break;
        case PEAK_OUT_R:                peak_out_r_output = static_cast<float *>(data); break;
    }
}

// NUOVA funzione helper per un detector RMS (es. per VCA)
float Gregchild::calculate_rms_level(float sample_l, float sample_r, float alpha_attack, float alpha_release) {
    // Un semplice filtro di primo ordine per l'inviluppo RMS (squared envelope)
    // Usiamo il detector_env dalla VcaCompressorState per mantenere lo stato
    float input_power = (sample_l * sample_l + sample_r * sample_r) * 0.5f; // Potenza media L/R

    float current_alpha = (input_power > vca_state.detector_env) ? alpha_attack : alpha_release;
    vca_state.detector_env += (input_power - vca_state.detector_env) * current_alpha;

    // Protezione da valori negativi o NaN
    if (vca_state.detector_env < 0.0f) vca_state.detector_env = 0.0f;

    // Restituisce il livello RMS in dB
    return Gua76DSP::linear_to_db(std::sqrt(vca_state.detector_env));
}

// Modifica process_sidechain per usare i nuovi detector
float Gregchild::process_sidechain(float input_l, float input_r, float sidechain_l, float sidechain_r) {
    // ... (existing sidechain selection and HPF/LPF placeholders) ...

    float detector_input_abs_l = std::abs(sidechain_detector_l);
    float detector_input_abs_r = std::abs(sidechain_detector_r);

    float combined_detector_level = 0.0f; // Questo sarà il livello finale per il detector

    int stereo_mode = static_cast<int>(*stereo_link_mode_param);
    int comp_mode = static_cast<int>(*compressor_mode_param);

    if (stereo_mode == 0) { // Stereo Link (tutti i compressori)
        if (comp_mode == 2) { // JFET (usa peak)
            // Aggiorna last_peak_l/r e usa i valori DB per JFET
            last_peak_l = std::max(last_peak_l * 0.99f, detector_input_abs_l);
            last_peak_r = std::max(last_peak_r * 0.99f, detector_input_abs_r);
            // Per JFET linkato, prendiamo il massimo dei picchi e lo convertiamo in dB
            combined_detector_level = Gua76DSP::linear_to_db(std::max(last_peak_l, last_peak_r));
        } else if (comp_mode == 3) { // VCA (usa RMS)
            // Usa RMS per VCA
            combined_detector_level = calculate_rms_level(detector_input_abs_l, detector_input_abs_r, 0.01f, 0.001f); // Esempio alpha
        } else { // Vari-Mu / Opto (RMS-like inviluppo)
            combined_detector_level = (detector_input_abs_l + detector_input_abs_r) * 0.5f;
            // Questo è un inviluppo semplice, un vero RMS sarebbe migliore
        }
    } else if (stereo_mode == 1) { // Dual Mono - questa funzione deve ritornare il livello per il canale corrente
        // Per Dual Mono, la logica del detector e GR deve essere duplicata per L e R nel main loop.
        // Questa funzione helper non si adatta bene a Dual Mono.
        // Sarà più efficiente calcolare detector_level_for_comp_l e _r direttamente nel run() loop.
        // Per ora, restituiamo un valore, ma l'implementazione sarà diversa.
        combined_detector_level = detector_input_abs_l; // Placeholder
    } else { // Mid/Side
        float M = (sidechain_detector_l + sidechain_detector_r) * 0.5f;
        float S = (sidechain_detector_l - sidechain_detector_r) * 0.5f;

        if (*mid_side_link_param > 0.5f) { // Mid/Side Linked
            if (comp_mode == 2) { // JFET (peak)
                last_peak_l = std::max(last_peak_l * 0.99f, std::abs(M));
                last_peak_r = std::max(last_peak_r * 0.99f, std::abs(S));
                combined_detector_level = Gua76DSP::linear_to_db(std::max(last_peak_l, last_peak_r));
            } else if (comp_mode == 3) { // VCA (RMS)
                 combined_detector_level = calculate_rms_level(M, S, 0.01f, 0.001f);
            } else { // Vari-Mu / Opto (RMS-like inviluppo)
                combined_detector_level = (std::abs(M) + std::abs(S)) * 0.5f;
            }
        } else { // Mid/Side Unlinked
            // Anche qui, il detector_level_for_comp_l e _r dovrà essere gestito separatamente nel run()
            combined_detector_level = std::abs(M); // Placeholder
        }
    }
    
    // Converti a DB per JFET e VCA, o rimani lineare per Vari-Mu/Opto, a seconda del comp_mode
    if (comp_mode == 0 || comp_mode == 1) { // Vari-Mu / Opto (si aspettano valori lineari)
        return combined_detector_level;
    } else { // JFET / VCA (si aspettano valori in dB)
        return combined_detector_level; // Già in dB dal calcolo sopra per JFET/VCA
    }
}


// Implementazione di run()
void Gregchild::run(uint32_t n_samples) {
    // ... (existing parameter setup) ...

    const int compressor_mode = static_cast<int>(*compressor_mode_param); // Ora può essere 0, 1, 2, 3 (Vari-Mu, Opto, JFET, VCA)
    const int fairchild_program_idx = static_cast<int>(*fairchild_program_param);

    // Parametri JFET (usati solo in modalità JFET)
    jfet_state.attack_time_ms = *attack_param;
    jfet_state.release_time_ms = *release_param;
    jfet_state.ratio = *ratio_param;
    jfet_state.all_button_mode_on = (static_cast<int>(*ratio_param) == 4);
    //jfet_state.threshold_db = Gua76DSP::linear_to_db(*input_gain_param * 1.5f); // Example threshold

    // Parametri VCA (usati solo in modalità VCA)
    vca_state.threshold_db = Gua76DSP::linear_to_db(*input_gain_param * 1.5f); // Esempio: soglia dal gain input
    vca_state.ratio = *ratio_param; // Usa la stessa ratio enumerata del JFET per semplicità
    vca_state.attack_time_ms = *attack_param;
    vca_state.release_time_ms = *release_param;
    vca_state.soft_knee_on = (*vca_soft_knee_on_param > 0.5f);
    vca_state.knee_width_db = *vca_knee_width_param;
    
    // ... (existing Opto parameters) ...

    // Loop principale di elaborazione per sample
    for (uint32_t i = 0; i < n_samples; ++i) {
        float sample_l = audio_in_l[i];
        float sample_r = audio_in_r[i];

        // --- 1. Processing Pre-Compression ---
        // ... (existing pad, input softclip, input gain, drive/saturation) ...

        // --- 2. Sidechain Processing and Detector ---
        float sidechain_detector_l_val = audio_in_l[i]; // Default: input audio
        float sidechain_detector_r_val = audio_in_r[i];

        if (sidechain_in_l) sidechain_detector_l_val = sidechain_in_l[i];
        if (sidechain_in_r) sidechain_detector_r_val = sidechain_in_r[i];

        // QUI VANNO I FILTRI DEL SIDECHAIN (HPF/LPF) - placeholders
        // Filtra sidechain_detector_l_val e _r_val
        // ...

        float detector_level_l = 0.0f; // Livello per il calcolo della GR canale L
        float detector_level_r = 0.0f; // Livello per il calcolo della GR canale R

        int stereo_mode = static_cast<int>(*stereo_link_mode_param);
        if (stereo_mode == 0) { // Stereo Link (per tutti i tipi di compressore)
            // Calcola un singolo livello detector combinato L+R
            if (compressor_mode == 2) { // JFET (usa peak in dB)
                last_peak_l = std::max(last_peak_l * 0.99f, std::abs(sidechain_detector_l_val));
                last_peak_r = std::max(last_peak_r * 0.99f, std::abs(sidechain_detector_r_val));
                detector_level_l = Gua76DSP::linear_to_db(std::max(last_peak_l, last_peak_r));
                detector_level_r = detector_level_l;
            } else if (compressor_mode == 3) { // VCA (usa RMS in dB)
                detector_level_l = calculate_rms_level(sidechain_detector_l_val, sidechain_detector_r_val, 0.01f, 0.001f);
                detector_level_r = detector_level_l;
            } else { // Vari-Mu / Opto (usa inviluppo lineare)
                detector_level_l = (std::abs(sidechain_detector_l_val) + std::abs(sidechain_detector_r_val)) * 0.5f;
                detector_level_r = detector_level_l;
            }
        } else if (stereo_mode == 1) { // Dual Mono (detector separato per L e R)
            if (compressor_mode == 2) { // JFET
                last_peak_l = std::max(last_peak_l * 0.99f, std::abs(sidechain_detector_l_val));
                last_peak_r = std::max(last_peak_r * 0.99f, std::abs(sidechain_detector_r_val));
                detector_level_l = Gua76DSP::linear_to_db(last_peak_l);
                detector_level_r = Gua76DSP::linear_to_db(last_peak_r);
            } else if (compressor_mode == 3) { // VCA
                // Avremmo bisogno di 2 istanze di VcaCompressorState per il vero dual mono
                // Per ora, useremo lo stesso stato, ma il detector RMS dovrà essere applicato per ogni canale.
                // Questa è una semplificazione che NON è un vero dual mono.
                detector_level_l = Gua76DSP::linear_to_db(std::sqrt(sidechain_detector_l_val * sidechain_detector_l_val)); // Semplice RMS per un sample
                detector_level_r = Gua76DSP::linear_to_db(std::sqrt(sidechain_detector_r_val * sidechain_detector_r_val));
            } else { // Vari-Mu / Opto
                detector_level_l = std::abs(sidechain_detector_l_val);
                detector_level_r = std::abs(sidechain_detector_r_val);
            }
        } else { // Mid/Side
            float M = (sample_l + sample_r) * 0.5f; // Converti l'audio in M/S per il processamento
            float S = (sample_l - sample_r) * 0.5f;

            float detector_M_val = (sidechain_detector_l_val + sidechain_detector_r_val) * 0.5f;
            float detector_S_val = (sidechain_detector_l_val - sidechain_detector_r_val) * 0.5f;

            if (*mid_side_link_param > 0.5f) { // Mid/Side Linked (un singolo detector per M+S)
                if (compressor_mode == 2) { // JFET
                    last_peak_l = std::max(last_peak_l * 0.99f, std::abs(detector_M_val));
                    last_peak_r = std::max(last_peak_r * 0.99f, std::abs(detector_S_val));
                    detector_level_l = Gua76DSP::linear_to_db(std::max(last_peak_l, last_peak_r));
                    detector_level_r = detector_level_l;
                } else if (compressor_mode == 3) { // VCA
                    detector_level_l = calculate_rms_level(detector_M_val, detector_S_val, 0.01f, 0.001f);
                    detector_level_r = detector_level_l;
                } else { // Vari-Mu / Opto
                    detector_level_l = (std::abs(detector_M_val) + std::abs(detector_S_val)) * 0.5f;
                    detector_level_r = detector_level_l;
                }
            } else { // Mid/Side Unlinked (detector separati per M e S)
                if (compressor_mode == 2) { // JFET
                    last_peak_l = std::max(last_peak_l * 0.99f, std::abs(detector_M_val));
                    last_peak_r = std::max(last_peak_r * 0.99f, std::abs(detector_S_val));
                    detector_level_l = Gua76DSP::linear_to_db(last_peak_l); // M detector
                    detector_level_r = Gua76DSP::linear_to_db(last_peak_r); // S detector
                } else if (compressor_mode == 3) { // VCA
                    detector_level_l = Gua76DSP::linear_to_db(std::sqrt(detector_M_val * detector_M_val)); // M RMS
                    detector_level_r = Gua76DSP::linear_to_db(std::sqrt(detector_S_val * detector_S_val)); // S RMS
                } else { // Vari-Mu / Opto
                    detector_level_l = std::abs(detector_M_val); // M inviluppo
                    detector_level_r = std::abs(detector_S_val); // S inviluppo
                }
            }
            sample_l = M; // Per continuare il processamento su M
            sample_r = S; // Per continuare il processamento su S
        }
        
        // --- 3. Calcolo della Gain Reduction (GR) in base al Compressor Mode ---
        float gr_lin_l = 1.0f;
        float gr_lin_r = 1.0f;
        float current_gr_db_l = 0.0f; // Per il meter

        if (!bypass_on) {
            if (compressor_mode == 0) { // Vari-Mu (Fairchild 670 Style)
                const VariMuProgramParams& program = fairchild_programs[fairchild_program_idx];
                current_gr_db_l = Gua76DSP::calculate_vari_mu_gr_db(detector_level_l, program.detector_threshold_lin, program.max_gr_db_val);
                gr_lin_l = Gua76DSP::db_to_linear(current_gr_db_l);

                if (stereo_mode == 1 || (stereo_mode == 2 && *mid_side_link_param <= 0.5f)) { // Dual Mono / M/S Unlinked
                    float vari_mu_gr_db_r = Gua76DSP::calculate_vari_mu_gr_db(detector_level_r, program.detector_threshold_lin, program.max_gr_db_val);
                    gr_lin_r = Gua76DSP::db_to_linear(vari_mu_gr_db_r);
                } else {
                    gr_lin_r = gr_lin_l;
                }
            } else if (compressor_mode == 1) { // Opto (LA-2A Style)
                current_gr_db_l = Gua76DSP::calculate_opto_gr_db(detector_level_l, Gua76DSP::db_to_linear(-10.0f), 20.0f, &opto_state);
                gr_lin_l = Gua76DSP::db_to_linear(current_gr_db_l);

                if (stereo_mode == 1 || (stereo_mode == 2 && *mid_side_link_param <= 0.5f)) {
                    // Nota: per un vero dual mono o M/S unlinked per Opto, avresti bisogno di un'altra OptoCellState per R/S
                    float opto_gr_db_r = Gua76DSP::calculate_opto_gr_db(detector_level_r, Gua76DSP::db_to_linear(-10.0f), 20.0f, &opto_state);
                    gr_lin_r = Gua76DSP::db_to_linear(opto_gr_db_r);
                } else {
                    gr_lin_r = gr_lin_l;
                }
            } else if (compressor_mode == 2) { // JFET (1176 Style)
                jfet_state.threshold_db = *input_gain_param * 40.0f - 20.0f; // Esempio di soglia in dB
                current_gr_db_l = Gua76DSP::calculate_jfet_gr_db(detector_level_l, &jfet_state);
                gr_lin_l = Gua76DSP::db_to_linear(current_gr_db_l);

                if (stereo_mode == 1 || (stereo_mode == 2 && *mid_side_link_param <= 0.5f)) {
                    // Nota: per un vero dual mono JFET, servirebbe un'altra JfetCompressorState per R/S
                    float jfet_gr_db_r = Gua76DSP::calculate_jfet_gr_db(detector_level_r, &jfet_state);
                    gr_lin_r = Gua76DSP::db_to_linear(jfet_gr_db_r);
                } else {
                    gr_lin_r = gr_lin_l;
                }
            } else { // VCA (Nuovo!)
                vca_state.threshold_db = *input_gain_param * 40.0f - 20.0f; // Esempio di soglia in dB
                current_gr_db_l = Gua76DSP::calculate_vca_gr_db(detector_level_l, &vca_state);
                gr_lin_l = Gua76DSP::db_to_linear(current_gr_db_l);

                if (stereo_mode == 1 || (stereo_mode == 2 && *mid_side_link_param <= 0.5f)) {
                    // Nota: per un vero dual mono VCA, avresti bisogno di un'altra VcaCompressorState per R/S
                    float vca_gr_db_r = Gua76DSP::calculate_vca_gr_db(detector_level_r, &vca_state);
                    gr_lin_r = Gua76DSP::db_to_linear(vca_gr_db_r);
                } else {
                    gr_lin_r = gr_lin_l;
                }
            }
        }
        
        // --- 4. Applicazione della Gain Reduction e Output Gain ---
        // Se in modalità Mid/Side, applica la GR a M e S
        if (stereo_mode == 2 && *mid_side_link_param <= 0.5f) { // Mid/Side Unlinked
            sample_l *= gr_lin_l; // M channel
            sample_r *= gr_lin_r; // S channel

            // Riconverti M/S in L/R per l'uscita
            float output_l = (sample_l + sample_r);
            float output_r = (sample_l - sample_r);
            sample_l = output_l;
            sample_r = output_r;
        } else {
            sample_l *= gr_lin_l;
            sample_r *= gr_lin_r;
        }

        // ... (existing output gain and output softclip) ...

        // --- 5. Metering ---
        // ... (existing metering logic) ...
        update_meter(gr_smooth_meter, current_gr_db_l, 0.1f); // GR in dB (usiamo l'L/M channel per il meter comune)
    }
    // ... (existing meter output) ...
}
