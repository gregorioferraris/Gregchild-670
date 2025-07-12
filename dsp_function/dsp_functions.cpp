#include "dsp_functions.h"

namespace Gua76DSP {

// --- Funzioni di Saturazione Valvolare (Softclip) ---

float tanh_saturate(float x, float drive) {
    // drive amplifica il segnale prima della tanh, aumentando la saturazione.
    // Un buon range per 'drive' potrebbe essere da 1.0 a 20.0 (o più).
    return std::tanh(x * drive);
}

float cubic_saturate(float x, float strength) {
    // strength controlla l'intensità della curvatura.
    // Clamped_x per evitare valori estremi con x^3 che possono causare NaN o valori enormi.
    // Questo è un softclip molto blando.
    float clamped_x = std::clamp(x, -1.5f, 1.5f); // Limita l'input per stabilità
    return clamped_x - (strength * clamped_x * clamped_x * clamped_x) / 3.0f;
}

float arctan_saturate(float x, float drive) {
    // drive controlla la pendenza della curva, similmente a tanh.
    // Il fattore (2.0f / M_PI) normalizza l'output tra -1.0 e 1.0.
    return (2.0f / M_PI) * std::atan(x * drive);
}

// --- Funzioni per il Compressore ---

float calculate_vari_mu_gr_db(float detector_level_lin, float threshold_lin, float max_gr_db) {
    // Questa è una modellazione semplificata che simula una ratio crescente.
    // Il Fairchild è molto più complesso, ma questo è un buon punto di partenza.

    detector_level_lin = std::max(0.0f, detector_level_lin);
    threshold_lin = std::max(0.000001f, threshold_lin); // Evita divisione per zero, ~-120dB

    if (detector_level_lin < threshold_lin) {
        return 0.0f; // Nessuna compressione sotto la soglia
    }

    // Calcola il livello sopra la soglia
    float over_threshold_ratio = detector_level_lin / threshold_lin;

    // Utilizzo di una funzione logaritmica per simulare la ratio crescente
    // Un valore più grande per log_base_factor renderà la curva più "dura"
    float log_base_factor = 1.0f; // Calibra questo valore
    float gr_factor = std::log10(1.0f + log_base_factor * (over_threshold_ratio - 1.0f));

    // Mappa questo fattore alla riduzione di guadagno in dB
    // Il -max_gr_db assicura che GR vada da 0 a max_gr_db (negativo)
    float gr_db = -max_gr_db * gr_factor;

    // Assicurati che la riduzione non superi il max_gr_db e non sia positiva
    return std::clamp(gr_db, -max_gr_db, 0.0f);
}

float calculate_opto_gr_db(float input_detector_level_lin, float threshold_lin, float max_gr_db_opto, OptoCellState* state) {
    // Converti la soglia da lineare a dB per un confronto più intuitivo
    // Per un Opto, il 'threshold' spesso influisce indirettamente sul 'trigger' del comp
    // Qui, la 'threshold_lin' agisce più come un gain prima del detector
    
    input_detector_level_lin = std::max(0.0f, input_detector_level_lin); // Assicurati che sia positivo
    
    // Il target light level è proporzionale al segnale sopra la soglia.
    // La scala (es. *10.0f) serve a rendere il segnale più incisivo per accendere la luce.
    float target_light_level = std::min(1.0f, input_detector_level_lin * (1.0f / threshold_lin));

    // Smoothing per l'attacco (accensione della luce)
    state->current_light_level += (target_light_level - state->current_light_level) * state->attack_smoothing_coeff;

    // Smoothing per il rilascio (spegnimento della luce) con due stadi
    // La luce si spegne più velocemente all'inizio, poi rallenta
    float effective_release_coeff = state->release_smoothing_coeff_fast;
    if (state->current_light_level < state->release_transition_level) {
        effective_release_coeff = state->release_smoothing_coeff_slow;
    }
    // Applica il rilascio solo se il target è inferiore al corrente (luce che si spegne)
    if (target_light_level < state->current_light_level) {
        state->current_light_level += (target_light_level - state->current_light_level) * effective_release_coeff;
    }


    // Limita il livello della luce a un range sensato (0.0 - 1.0)
    state->current_light_level = std::clamp(state->current_light_level, 0.0f, 1.0f);

    // Calcola il fattore di riduzione di guadagno basato sul livello di luce
    // Più luce = più compressione. L'esponente crea la non linearità.
    float gr_factor = std::pow(state->current_light_level, 0.8f); // Calibra l'esponente

    // Applica una riduzione di guadagno massima in dB
    float gr_db = -max_gr_db_opto * gr_factor;

    // Smooting finale della GR per evitare clicks/pops (opzionale, ma consigliato)
    state->gr_smooth_current += (gr_db - state->gr_smooth_current) * 0.01f; // Valore piccolo per smoothing morbido

    return std::clamp(state->gr_smooth_current, -max_gr_db_opto, 0.0f);
}

float calculate_jfet_gr_db(float peak_detector_level_db, JfetCompressorState* state) {
    // Calcola il target GR in base alla soglia e alla ratio
    float target_gr_db = 0.0f;
    if (peak_detector_level_db > state->threshold_db) {
        target_gr_db = (state->threshold_db - peak_detector_level_db) / state->ratio;
    }

    // Modalità All-Button: Aumenta la ratio e modifica i tempi per il "pomping"
    if (state->all_button_mode_on) {
        // Questi valori dovrebbero essere calibrati per replicare il comportamento 1176
        // Il 1176 in All-Button ha una ratio non costante (tipo tra 12:1 e 20:1 o più)
        // e tempi di attacco/rilascio molto specifici e aggressivi.
        // Qui, impostiamo una ratio fissa molto alta. Potresti voler una curva più complessa.
        if (peak_detector_level_db > state->threshold_db) {
             target_gr_db = (state->threshold_db - peak_detector_level_db) / 20.0f; // Esempio: 20:1
        }
        // Il "pomping" in all-button è spesso un effetto collaterale di un rilascio molto veloce
        // combinato con un attacco istantaneo e la saturazione.
        // I coefficienti di smoothing qui sotto dovrebbero essere molto aggressivi.
    }

    // Calcolo dei coefficienti di smoothing per attacco e rilascio
    // Utilizziamo un filtro di primo ordine per smoothing esponenziale (tipico compressore)
    // 1000.0f per convertire ms in secondi
    float attack_coeff = 1.0f;
    if (state->attack_time_ms > 0.0f) {
        attack_coeff = 1.0f - std::exp(-1000.0f / (state->attack_time_ms * state->sample_rate));
    } else { // Attacco istantaneo
        attack_coeff = 1.0f;
    }

    float release_coeff = 1.0f;
    if (state->release_time_ms > 0.0f) {
        release_coeff = 1.0f - std::exp(-1000.0f / (state->release_time_ms * state->sample_rate));
    } else { // Rilascio istantaneo
        release_coeff = 1.0f;
    }

    // smoothing: La GR attuale si muove verso la GR target
    if (target_gr_db < state->current_gr_db) { // Attacco (GR sta diminuendo/diventando più negativa)
        state->current_gr_db += (target_gr_db - state->current_gr_db) * attack_coeff;
    } else { // Rilascio (GR sta aumentando/diventando meno negativa)
        state->current_gr_db += (target_gr_db - state->current_gr_db) * release_coeff;
    }

    // Assicurati che GR sia non-positiva
    return std::min(0.0f, state->current_gr_db);
}


// --- Altre Utilità DSP ---

float db_to_linear(float db) {
    return std::pow(10.0f, db / 20.0f);
}

float linear_to_db(float linear) {
    if (linear <= 0.000000001f) { // Un valore molto piccolo vicino a zero
        return -120.0f; // Un valore molto basso per evitare log(0)
    }
    return 20.0f * std::log10(linear);
}

} // namespace Gua76DSP
