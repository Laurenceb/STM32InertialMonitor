#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 1350 Hz

* 0 Hz - 30 Hz
  gain = 1
  desired ripple = 3.5 dB
  actual ripple = 2.788049286098028 dB

* 55 Hz - 675 Hz
  gain = 0
  desired attenuation = -55 dB
  actual attenuation = -55.03593469204165 dB

*/

#define SAMPLEFILTER_TAP_NUM_1350 95
#ifndef SampleFilter
typedef struct {
  float history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;
#endif
void SampleFilter_init_1350(SampleFilter* f);
void SampleFilter_put_1350(SampleFilter* f, float input);
float SampleFilter_get_1350(SampleFilter* f);

#endif
