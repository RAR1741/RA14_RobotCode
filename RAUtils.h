#ifndef RA_UTIL_H__
#define RA_UTIL_H__

#include <stdio.h>
#include <assert.h>

#ifndef RA_NDEBUG
#define RA_DEBUG(expr) (::fprintf(stderr, #expr " = %g\n", expr))
#else
#define RA_DEBUG(expr) 
#endif
#endif
