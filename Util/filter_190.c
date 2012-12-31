#include "filter_190.h"

static const float filter_taps[SAMPLEFILTER_TAP_NUM_190] = {
  0.010161220120326199,
  0.0463747587330119,
  0.05153992752170604,
  -0.02518282102367497,
  -0.09253330956019078,
  0.029172665311026646,
  0.31321989691396773,
  0.4693295562322271,
  0.31321989691396773,
  0.029172665311026646,
  -0.09253330956019078,
  -0.02518282102367497,
  0.05153992752170604,
  0.0463747587330119,
  0.010161220120326199
};

void SampleFilter_init_190(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM_190; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put_190(SampleFilter* f, float input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM_190)
    f->last_index = 0;
}

float SampleFilter_get_190(SampleFilter* f) {
  float acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM_190; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM_190-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}
