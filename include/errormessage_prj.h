#ifndef ERRORMESSAGE_PRJ_H_INCLUDED
#define ERRORMESSAGE_PRJ_H_INCLUDED

#define ERROR_BUF_SIZE 4

/* ENGAGEFAILED: the engage/emergency-clamp sequence timed out without
 *               reaching the target clamp current. */
#define ERROR_MESSAGE_LIST \
   ERROR_MESSAGE_ENTRY(ENGAGEFAILED,   ERROR_STOP)    \

#endif // ERRORMESSAGE_PRJ_H_INCLUDED
