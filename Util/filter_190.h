#ifndef SAMPLEFILTER_H_190
#define SAMPLEFILTER_H_190

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 200 Hz

* 0 Hz - 40 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 2.356662011183384 dB

* 60 Hz - 100 Hz
  gain = 0
  desired attenuation = -45.5 dB
  actual attenuation = -45.54190725372767 dB

*/

#define SAMPLEFILTER_TAP_NUM_190 15
#ifndef SAMPLEFILTER
#define SAMPLEFILTER
typedef struct {
  float history[SAMPLEFILTER_TAP_NUM_190];
  unsigned int last_index;
} SampleFilter;
#endif
void SampleFilter_init_190(SampleFilter* f);
void SampleFilter_put_190(SampleFilter* f, float input);
float SampleFilter_get_190(SampleFilter* f);

#endif

