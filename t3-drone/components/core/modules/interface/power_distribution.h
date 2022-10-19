
#ifndef __POWER_DISTRIBUTION_H__
#define __POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"
#include "espnow.h"

void powerDistributionInit(void);
bool powerDistributionTest(void);
void powerDistribution(const control_t *control);
void powerStop();


#endif //__POWER_DISTRIBUTION_H__
