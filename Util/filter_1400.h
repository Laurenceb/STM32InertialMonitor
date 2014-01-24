#ifndef SAMPLEFILTER_H_1400
#define SAMPLEFILTER_H_1400

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 1400 Hz

* 0 Hz - 35 Hz
  gain = 1
  desired ripple = 4 dB
  actual ripple = 2.736468522892423 dB

* 75 Hz - 700 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = -51.449674796414484 dB

*/

#define SAMPLEFILTER_TAP_NUM_1400 59
#ifndef SAMPLEFILTER
#define SAMPLEFILTER
typedef struct {
  float history[SAMPLEFILTER_TAP_NUM_1400];
  unsigned int last_index;
} SampleFilter;
#endif
void SampleFilter_init_1400(SampleFilter* f);
void SampleFilter_put_1400(SampleFilter* f, float input);
float SampleFilter_get_1400(SampleFilter* f);

#endif

