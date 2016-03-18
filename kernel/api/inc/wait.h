#ifndef _WAIT_H_
#define _WAIT_H_
#include "api.h"

typedef OS_EVENT *wait_event_t;

#define WAIT_EVENT_TIMEOUT 1

wait_event_t init_event(void);

int wait_event(wait_event_t wq);

int wait_event_timeout(wait_event_t wq, unsigned int timeout);

void wake_up(wait_event_t wq);

void clear_wait_event(wait_event_t wq);

#endif
