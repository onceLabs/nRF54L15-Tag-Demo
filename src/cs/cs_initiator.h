/*
 * Channel Sounding initiator (central) role.
 */
#ifndef APP_CS_INITIATOR_H_
#define APP_CS_INITIATOR_H_

#include "cs_shared.h"

/** @brief Start scanning + CS as initiator in the given ranging mode. */
int cs_initiator_start(enum cs_mode mode);

/** @brief Stop initiator activity and disconnect. */
void cs_initiator_stop(void);

#endif /* APP_CS_INITIATOR_H_ */
