/*
 * Channel Sounding reflector (peripheral) role.
 */
#ifndef APP_CS_REFLECTOR_H_
#define APP_CS_REFLECTOR_H_

#include "cs_shared.h"

/** @brief Start advertising + CS as reflector in the given ranging mode. */
int cs_reflector_start(enum cs_mode mode);

/** @brief Stop reflector activity and disconnect. */
void cs_reflector_stop(void);

#endif /* APP_CS_REFLECTOR_H_ */
