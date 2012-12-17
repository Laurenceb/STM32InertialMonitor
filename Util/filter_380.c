#include "filter_380.h"

static const float filter_taps[SAMPLEFILTER_TAP_NUM_380] = {
  0.0017662513974908976,
  0.0019528414440318665,
  0.00046629218788863666,
  -0.004631014220359179,
  -0.013931321835506858,
  -0.025908848540986636,
  -0.03643079833164208,
  -0.03940541891057635,
  -0.0286822653759347,
  -0.0006492925737543573,
  0.04350960770008883,
  0.09699727800433586,
  0.14866733724283565,
  0.18616355978332427,
  0.19987007850127547,
  0.18616355978332427,
  0.14866733724283565,
  0.09699727800433586,
  0.04350960770008883,
  -0.0006492925737543573,
  -0.0286822653759347,
  -0.03940541891057635,
  -0.03643079833164208,
  -0.025908848540986636,
  -0.013931321835506858,
  -0.004631014220359179,
  0.00046629218788863666,
  0.0019528414440318665,
  0.0017662513974908976
};

void SampleFilter_init_380(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM_380; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put_380(SampleFilter* f, float input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM_380)
    f->last_index = 0;
}

float SampleFilter_get_380(SampleFilter* f) {
  float acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM_380-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}
