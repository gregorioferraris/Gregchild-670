#include "dsp_functions.h"

namespace Gua76DSP {

// --- Funzioni di Saturazione Valvolare (Softclip) ---

float tanh_saturate(float x, float drive) {
    // Il fattore 'drive' amplifica il segnale prima della tanh, aumentando la saturazione.
    // Un buon range per 'drive' potrebbe essere 1.0 a 10.0 (o più).
    return std::tanh(x * drive);
}

float cubic_saturate(float x, float strength) {
    // strength controlla l'intensità della curvatura.
    // clamped_x per evitare valori estremi con x^3
    float clamped_x = std::clamp(x, -2.0f, 2.0f); // Limita l'input per stabilità
    return clamped_x - (strength * clamped_x * clamped_x * clamped_x) / 3.0f;
}

float arctan_saturate(float x, float drive) {
    // drive controlla la pendenza della curva, similmente a tanh.
    return (2.0f / M_PI) * std::atan(x * drive); // M_PI è in cmath su molti sistemi, altrimenti definiscilo.
}

// --- Funzioni per il Compressore Vari-Mu ---

float calculate_vari_mu_gr_db(float detector_level_lin, float threshold_lin, float max_gr_db) {
    // Questa è una modellazione semplificata che simula una ratio crescente.
    // Il Fairchild è molto più complesso, ma questo è un buon punto di partenza.

    // Assicurati che i livelli siano positivi e normalizzati (es. 0.0 a 1.0)
    detector_level_lin = std::max(0.0f, detector_level_lin);
    threshold_lin = std::max(0.0001f, threshold_lin); // Evita divisione per zero

    if (detector_level_lin < threshold_lin) {
        return 0.0f; // Nessuna compressione sotto la soglia
    }

    // Calcola il livello sopra la soglia
    float over_threshold_level = detector_level_lin / threshold_lin;

    // Questa formula simula una ratio che aumenta all'aumentare del segnale.
    // Potrebbe essere necessario calibrare i coefficienti (es. 0.5f, 2.0f)
    // per ottenere la curva Vari-Mu desiderata.
    // Qui, 'over_threshold_level' agisce come il fattore 'drive' per la compressione.
    // Un logaritmo o una potenza possono dare una risposta Vari-Mu.
    // Esempio con una funzione di potenza per una ratio non lineare:
    float gr_factor = std::pow(over_threshold_level, 0.5f); // 0.5 è come una radice quadrata

    // Mappa questo fattore alla riduzione di guadagno in dB
    // Il -max_gr_db assicura che GR vada da 0 a max_gr_db (negativo)
    float gr_db = -max_gr_db * (gr_factor - 1.0f); // Normalizza gr_factor per iniziare da 0

    // Assicurati che la riduzione non superi il max_gr_db e non sia positiva
    return std::clamp(gr_db, max_gr_db, 0.0f);
}

// --- Altre Utilità DSP ---

float db_to_linear(float db) {
    return std::pow(10.0f, db / 20.0f);
}

float linear_to_db(float linear) {
    if (linear <= 0.0f) {
        return -96.0f; // Un valore molto basso per evitare log(0)
    }
    return 20.0f * std::log10(linear);
}

} // namespace Gua76DSP
