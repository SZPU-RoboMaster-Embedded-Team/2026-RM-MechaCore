#include <cassert>

#define assert_always(expr) ((expr) ? (void)0 : __assert_func(__FILE__, __LINE__, __ASSERT_FUNC, #expr))

#ifndef __ASSERT_FUNC
/* Use g++'s demangled names in C++.  */
#if defined __cplusplus && defined __GNUC__
#define __ASSERT_FUNC __PRETTY_FUNCTION__

/* C99 requires the use of __func__.  */
#elif __STDC_VERSION__ >= 199901L
#define __ASSERT_FUNC __func__

/* Older versions of gcc don't have __func__ but can use __FUNCTION__.  */
#elif __GNUC__ >= 2
#define __ASSERT_FUNC __FUNCTION__

/* failed to detect __func__ support.  */
#else
#define __ASSERT_FUNC ((char *)0)
#endif
#endif /* !__ASSERT_FUNC */
