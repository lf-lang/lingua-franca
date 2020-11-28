/**
 * Utility functions for importing audio files with the
 * wave audio format. This code has few dependencies, so
 * it should run on just about any platform.
 * 
 * @author Edward A. Lee
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "wave_file_reader.h"

/**
 * Open a wave file, check that the format is supported,
 * allocate memory for the sample data, and fill the memory
 * with the sample data. It is up to the caller to free the
 * memory when done with it. That code should first free the
 * waveform element of the returned struct, then the struct itself.
 * This implementation supports only 16-bit linear PCM files.
 * On a Mac, you can convert audio files into this format
 * using the afconvert utility.
 * 
 * @param path The path to the file.
 * @return An array of sample data or NULL if the file can't be opened
 *  or has an usupported format.
 */
lf_sample_waveform_t* read_wave_file(const char* path) {
    FILE *fp = NULL;
    
    lf_wav_t wav;
    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "WARNING: Failed to open waveform sample file: %s\n", path);
        return NULL;
    }
 
    fread(&wav, 1, sizeof(lf_wav_t), fp);
     
    lf_wav_format_t fmt = wav.fmt;
    lf_wav_data_t data = wav.data;
 
    // Wave file format is described here:
    // https://sites.google.com/site/musicgapi/technical-documents/wav-file-format
    uint32_t expected_chunk_id = 'FFIR';      // Little-endian version of RIFF.
    uint32_t expected_format = 'EVAW';       // Little-endian version of WAVE.
    uint32_t expected_subchunk_id = ' tmf';  // Little-endian version of 'fmt '.
    uint32_t expected_data_id = 'atad';      // Little-endian version of 'data'.
    if (*(uint32_t*)wav.riff.chunk_id != expected_chunk_id
        || *(uint32_t*)wav.riff.format != expected_format
        || *(uint32_t*)fmt.subchunk_id != expected_subchunk_id
        || fmt.subchunk_size != 16
        || fmt.audio_format != 1
        || fmt.sample_rate != 44100
        || fmt.bits_per_sample != 16
        || *(uint32_t*)data.subchunk_id != expected_data_id
    ) {
        fprintf(stderr, "WARNING: Waveform sample not a supported format.\n");
        fprintf(stderr, "Chunk ID was expected to be 'RIFF'. Got: '%c%c%c%c'.\n",
                wav.riff.chunk_id[0], wav.riff.chunk_id[1], wav.riff.chunk_id[2], wav.riff.chunk_id[3]);
        fprintf(stderr, "Format was expected to be 'WAVE'. Got: '%c%c%c%c'.\n",
                wav.riff.format[0], wav.riff.format[1], wav.riff.format[2], wav.riff.format[3]);
        fprintf(stderr, "Subchunk ID was expected to be 'fmt '. Got: '%c%c%c%c'.\n",
                fmt.subchunk_id[0], fmt.subchunk_id[1], fmt.subchunk_id[2], fmt.subchunk_id[3]);
        fprintf(stderr, "Subchunk size was expected to be 16. Got: '%d'.\n",
                fmt.subchunk_size);
        fprintf(stderr, "Audio format was expected to be 1 (LPCM, no compression). Got: '%d'.\n",
                fmt.audio_format);
        fprintf(stderr, "Sample rate was expected to be 44100). Got: '%d'.\n",
                fmt.sample_rate);
        fprintf(stderr, "Bits per sample was expected to be 16. Got: '%d'.\n",
                fmt.bits_per_sample);
        fprintf(stderr, "Data ID was expected to be 'data'. Got: '%c%c%c%c'.\n",
                data.subchunk_id[0], data.subchunk_id[1], data.subchunk_id[2], data.subchunk_id[3]);
    }

    uint32_t chunk_size = wav.riff.chunk_size;
    // printf("Chunk size \t%d\n", wav.riff.chunk_size);
    uint16_t num_channels = fmt.num_channels;     

    // Ignoring the following fields. Should we?
    // printf("byte_rate \t%d\n", fmt.byte_rate);
    // printf("BlockAlign \t%d\n", fmt.BlockAlign);

    // printf("Data subchunk size \t%d\n", data.subchunk_size);

    lf_sample_waveform_t* result = (lf_sample_waveform_t*)malloc(sizeof(lf_sample_waveform_t));
    // printf("Size of lf_sample_waveform_t %d", sizeof(lf_sample_waveform_t));
    result->length = data.subchunk_size/2; // Subchunk size is in bytes, but length is number of samples.
    result->num_channels = num_channels;
    result->waveform = (int16_t*)calloc(data.subchunk_size/2, sizeof(int16_t));

    size_t bytes_read = fread(result->waveform, sizeof(int16_t), data.subchunk_size/2 , fp);
    if (bytes_read != data.subchunk_size/2) {
        fprintf(stderr, "WARNING: Expected %d bytes, but got %zu.\n", data.subchunk_size, bytes_read);
    }

    // printf("duration \t%f\n", (data.subchunk_size * 1.0) / fmt.byte_rate);
    return result;
 }
