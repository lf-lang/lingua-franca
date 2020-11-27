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
 * @param path The path to the file.
 * @return An array of sample data or NULL if the file can't be opened
 *  or has an usupported format.
 */
Sample_Waveform_t* read_wave_file(const char* path) {
    FILE *fp = NULL;
    
    Wav wav;
    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "WARNING: Failed to open waveform sample file: %s\n", path);
        return NULL;
    }
 
    fread(&wav, 1, sizeof(Wav), fp);
     
    FMT_t fmt = wav.fmt;
    Data_t data = wav.data;
 
    // Wave file format is described here:
    // https://sites.google.com/site/musicgapi/technical-documents/wav-file-format
    uint32_t expected_chunkID = 'FFIR';      // Little-endian version of RIFF.
    uint32_t expected_format = 'EVAW';       // Little-endian version of WAVE.
    uint32_t expected_subchunk_id = ' tmf';  // Little-endian version of 'fmt '.
    uint32_t expected_data_id = 'atad';      // Little-endian version of 'data'.
    if (*(uint32_t*)wav.riff.ChunkID != expected_chunkID
        || *(uint32_t*)wav.riff.Format != expected_format
        || *(uint32_t*)fmt.Subchunk1ID != expected_subchunk_id
        || fmt.Subchunk1Size != 16
        || fmt.AudioFormat != 1
        || fmt.SampleRate != 44100
        || fmt.BitsPerSample != 16
        || *(uint32_t*)data.Subchunk2ID != expected_data_id
    ) {
        fprintf(stderr, "WARNING: Waveform sample not a supported format.\n");
        fprintf(stderr, "Chunk ID was expected to be 'RIFF'. Got: '%c%c%c%c'.\n",
                wav.riff.ChunkID[0], wav.riff.ChunkID[1], wav.riff.ChunkID[2], wav.riff.ChunkID[3]);
        fprintf(stderr, "Format was expected to be 'WAVE'. Got: '%c%c%c%c'.\n",
                wav.riff.Format[0], wav.riff.Format[1], wav.riff.Format[2], wav.riff.Format[3]);
        fprintf(stderr, "Subchunk ID was expected to be 'fmt '. Got: '%c%c%c%c'.\n",
                fmt.Subchunk1ID[0], fmt.Subchunk1ID[1], fmt.Subchunk1ID[2], fmt.Subchunk1ID[3]);
        fprintf(stderr, "Subchunk size was expected to be 16. Got: '%d'.\n",
                fmt.Subchunk1Size);
        fprintf(stderr, "Audio format was expected to be 1 (LPCM, no compression). Got: '%d'.\n",
                fmt.AudioFormat);
        fprintf(stderr, "Sample rate was expected to be 44100). Got: '%d'.\n",
                fmt.SampleRate);
        fprintf(stderr, "Bits per sample was expected to be 16. Got: '%d'.\n",
                fmt.BitsPerSample);
        fprintf(stderr, "Data ID was expected to be 'data'. Got: '%c%c%c%c'.\n",
                data.Subchunk2ID[0], data.Subchunk2ID[1], data.Subchunk2ID[2], data.Subchunk2ID[3]);
    }

    uint32_t chunk_size = wav.riff.ChunkSize;
    // printf("Chunk size \t%d\n", wav.riff.ChunkSize);
    uint16_t num_channels = fmt.NumChannels;     

    // Ignoring the following fields. Should we?
    // printf("ByteRate \t%d\n", fmt.ByteRate);
    // printf("BlockAlign \t%d\n", fmt.BlockAlign);

    uint32_t block_size = data.Subchunk2Size;
    // printf("Data subchunk size \t%d\n", data.Subchunk2Size);

    Sample_Waveform_t* result = (Sample_Waveform_t*)malloc(sizeof(Sample_Waveform_t));
    // printf("Size of Sample_Waveform_t %d", sizeof(Sample_Waveform_t));
    result->length = block_size/2; // Block size appears to be in samples, not bytes.
    result->num_channels = num_channels;
    result->waveform = (int16_t*)calloc(block_size/2, sizeof(int16_t));

    size_t bytes_read = fread(result->waveform, sizeof(int16_t), block_size/2 , fp);
    if (bytes_read != block_size/2) {
        fprintf(stderr, "WARNING: Expected %d bytes, but got %zu.\n", block_size, bytes_read);
    }

    // printf("duration \t%f\n", (data.Subchunk2Size * 1.0) / fmt.ByteRate);
    return result;
 }
