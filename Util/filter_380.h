#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 380 Hz

* 0 Hz - 30 Hz
  gain = 1
  desired ripple = 3.5 dB
  actual ripple = 2.454533876216781 dB

* 55 Hz - 190 Hz
  gain = 0
  desired attenuation = -55 dB
  actual attenuation = -56.12596514501477 dB

*/

#define SAMPLEFILTER_TAP_NUM_380 29
#ifndef SampleFilter
typedef struct {
  float history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;
#endif
void SampleFilter_init_380(SampleFilter* f);
void SampleFilter_put_380(SampleFilter* f, float input);
float SampleFilter_get_380(SampleFilter* f);

