#ifndef SAMPLEFILTER_H_1200
#define SAMPLEFILTER_H_1200

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 1200 Hz

* 0 Hz - 35 Hz
  gain = 1
  desired ripple = 4 dB
  actual ripple = 3.244185171598952 dB

* 65 Hz - 600 Hz
  gain = 0
  desired attenuation = -51.931 dB
  actual attenuation = -51.93110286977447 dB

*/

#define SAMPLEFILTER_TAP_NUM_1200 61
#ifndef SAMPLEFILTER
#define SAMPLEFILTER
typedef struct {
  float history[SAMPLEFILTER_TAP_NUM_1200];
  unsigned int last_index;
} SampleFilter;
#endif
void SampleFilter_init_1200(SampleFilter* f);
void SampleFilter_put_1200(SampleFilter* f, float input);
float SampleFilter_get_1200(SampleFilter* f);

#endif
