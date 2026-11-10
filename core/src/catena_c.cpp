#include "catena/catena_c.h"

extern "C" {

void catena_default_params(catena_params_t* out) {
  const catena_params_t defaults = CATENA_PARAMS_DEFAULT;
  *out = defaults;
}

size_t catena_sizeof_params(void) { return sizeof(catena_params_t); }

}  // extern "C"
