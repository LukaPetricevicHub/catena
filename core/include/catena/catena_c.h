/* C API of the Catena core. The simulator loads it with ctypes; the firmware links it. */
#ifndef CATENA_C_H
#define CATENA_C_H

#include <stddef.h>
#include <stdint.h>

#include "catena/params_gen.h" /* catena_params_t, generated from config/robot.yaml */

#if defined(__GNUC__)
#define CATENA_API __attribute__((visibility("default")))
#else
#define CATENA_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Parameters compiled in from config/robot.yaml. */
CATENA_API void catena_default_params(catena_params_t* out);

/* Lets a binding check that its struct layout matches this build. */
CATENA_API size_t catena_sizeof_params(void);

#ifdef __cplusplus
}
#endif

#endif
