/**
 * Data types for the wave audio format.
 * 
 * @author Edward A. Lee
 */

/**
 * Waveform in 16-bit linear-PCM format.
 * The waveform element is an array containing audio samples.
 * If there are two channels, then they are interleaved
 * left and right channel. The length is the total number
 * of samples, a multiple of the number of channels.
 */
typedef struct lf_sample_waveform_t {
    uint32_t length;
    uint16_t num_channels;
    int16_t* waveform;
} lf_sample_waveform_t;

/**
 * The top-level 'chunk' of a .wav file is a 'RIFF'
 * chunk, identified by an ID that is an int formed
 * by the sequence of four chars 'RIFF'.
 */
typedef struct {
    char chunk_id[4];    // "RIFF"
    uint32_t chunk_size; // 36 + subchunk_size
    char format[4];      // "WAVE"
} lf_wav_riff_t;

/**
 * The first subchunk within a .wav file is a 'fmt '
 * chunk, identified by an ID that is an int formed
 * by the sequence of four chars 'fmt '.
 */
typedef struct {
    char subchunk_id[4];    // 'fmt '
    uint32_t subchunk_size; // 16 for linear PCM.
    uint16_t audio_format;  // 1 for linear PCM
    uint16_t num_channels;  // 1 for mono = 1, 2 for stereo, etc.
    uint32_t sample_rate;   // 44100
    uint32_t byte_rate;     // sample_rate * num_channels * bits_per_sample/8
    uint16_t BlockAlign;    /* = num_channels * bits_per_sample/8 */
    uint16_t bits_per_sample; /* 8bits, 16bits, etc. */
} lf_wav_format_t;

/**
 * Header for the subchunk containing the data.
 * This is a 'data' chunk, identified by an ID that
 * is an int formed by the sequenced of four chars 'data'.
 */
typedef struct {
    char subchunk_id[4];    // 'data'
    uint32_t subchunk_size; // data size in bytes
} lf_wav_data_t;

/**
 * Overall wave data.
 */
typedef struct {
   lf_wav_riff_t riff;
   lf_wav_format_t fmt;
   lf_wav_data_t data;
} lf_wav_t;
