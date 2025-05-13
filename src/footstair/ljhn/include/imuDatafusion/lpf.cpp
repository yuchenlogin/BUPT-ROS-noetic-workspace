#include "lpf.h"
#include "math.h"

LowPassFilter2p::LowPassFilter2p(float sample_freq, float cutoff_freq)
{
  // set initial parameters
  set_cutoff_frequency(sample_freq, cutoff_freq);
  _delay_element_1 = _delay_element_2 = 0;
}

void LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
  _cutoff_freq = cutoff_freq;
  if (_cutoff_freq <= 0.0f)
  {
    // no filtering
    return;
  }
  float fr = sample_freq / _cutoff_freq;
  float ohm = tanf(M_PI / fr);
  float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
  _b0 = ohm * ohm / c;
  _b1 = 2.0f * _b0;
  _b2 = _b0;
  _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  _a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
}

float LowPassFilter2p::apply(float sample)
{
  if (_cutoff_freq <= 0.0f)
  {
    // no filtering
    return sample;
  }
  // do the filtering
  float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
  if (isnan(delay_element_0) || isinf(delay_element_0))
  {
    // don't allow bad values to propogate via the filter
    delay_element_0 = sample;
  }
  float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

  _delay_element_2 = _delay_element_1;
  _delay_element_1 = delay_element_0;

  // return the value.  Should be no need to check limits
  return output;
}
