#ifndef HX711_H
#define HX711_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hx711_init(void);
bool hx711_is_ready(void);
bool hx711_read(int32_t * p_raw);
bool hx711_prepare(int32_t * p_offset);
bool hx711_read_filtered(int32_t * p_raw);
int32_t hx711_net_to_weight_0p1g(int32_t net);

#ifdef __cplusplus
}
#endif

#endif /* HX711_H */
