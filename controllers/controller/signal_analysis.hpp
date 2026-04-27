#include <math.h>
#include "kiss_fft/kiss_fft.h" // FFT library

#define SIGNAL_LENGTH 1024 // length of the signal to be analyzed

/**
 * @brief KISS FFT USAGE EXAMPLE
 * @param signal_data pointer to the array containing the signal to be analyzed 
 */
void kiss_fft_demo(double* signal_data){

    /* fft preparation */
    kiss_fft_cfg cfg = kiss_fft_alloc( SIGNAL_LENGTH, 0, NULL, NULL );

     /* fft variables */  
    kiss_fft_cpx cx_in[SIGNAL_LENGTH],  // input signal (time domain)
                 cx_out[SIGNAL_LENGTH]; // output signal (frequency domain)
    
    // prepare the input (add signal in the 'in' structure of the kiss_fft)
    for (int n=0; n<SIGNAL_LENGTH; n++) {
      cx_in[n].r = signal_data[n]; // the real part of the signal is the data
      cx_in[n].i = 0.; // set the imaginary part to zero
    }
    
    // run the fft (the fourier transform is stored in the 'out' structure of the kiss_fft)
    kiss_fft( cfg , cx_in , cx_out ); 

    // compute the magnitude of the complex numbers
    double mag[SIGNAL_LENGTH];
    for (int n=0; n<SIGNAL_LENGTH; n++) {
      mag[n] = sqrt(cx_out[n].r*cx_out[n].r + cx_out[n].i*cx_out[n].i);
    }

    // TODO: do something with the magnitude, real, imaginary part of the fft
    // ...

    // free fft memory once done with it
    free(cfg);
} 
