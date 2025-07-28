#pragma once

#define BUZZER_SINGLE_CLICK_MS 100

// Buzzer state type
typedef enum
{
    BUZZER_OFF,
    BUZZER_SINGLE,
    BUZZER_REPEAT
} buzzer_state_t;

// Buzzer control structure
typedef struct
{
    buzzer_state_t state;     // Current buzzer state
    uint32_t on_time_ms;      // Duration of beep (milliseconds)
    uint32_t off_time_ms;     // Interval time (milliseconds)
    uint32_t elapsed_time_ms; // Elapsed time (milliseconds)
    bool is_on;               // Whether currently beeping
} buzzer_t;

typedef struct
{
    uint32_t frequency; // Frequency of the tone in Hz
    uint32_t duration;  // Duration of the tone in milliseconds
} buzzer_tone_t;

typedef struct
{
    const buzzer_tone_t *tones; // Pointer to an array of tones
    size_t tone_count;          // Number of tones in the array
    size_t current_tone;        // Index of the current tone being played
    uint32_t elapsed_time_ms;   // Time elapsed for the current tone
} buzzer_tone_sequence_t;

extern buzzer_tone_t doremi[];
void buzzer_init(void);
void buzzer_update(uint32_t delta_time_ms);
void buzzer_play_tone_sequence(const buzzer_tone_t *tones, size_t tone_count);
void buzzer_set_state(buzzer_state_t state, uint32_t on_time_ms, uint32_t off_time_ms);