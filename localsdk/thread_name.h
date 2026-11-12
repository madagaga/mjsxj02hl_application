#ifndef LOCALSDK_THREAD_NAME_H
#define LOCALSDK_THREAD_NAME_H

#include <sys/prctl.h>

/* Name the calling thread (max 15 chars) so it shows up in `top -H` and
   /proc/<pid>/task/<tid>/comm. Call it first thing in each thread function. */
static inline void thread_set_name(const char *name)
{
    prctl(PR_SET_NAME, name, 0, 0, 0);
}

#endif /* LOCALSDK_THREAD_NAME_H */
