#ifndef TESTLIB_H
#define TESTLIB_H

#include <stdio.h>
#include <math.h>
#include <string.h>

int g_passes   = 0;
int g_failures = 0;

#define ASSERT_EQ_INT(actual, expected, name)                              \
    do {                                                                   \
        int _a = (int)(actual);                                            \
        int _e = (int)(expected);                                          \
        if (_a != _e) {                                                    \
            printf("  FAIL  %s\n        expected %d  got %d\n",           \
                   (name), _e, _a);                                        \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_EQ_UINT(actual, expected, name)                             \
    do {                                                                   \
        unsigned int _a = (unsigned int)(actual);                          \
        unsigned int _e = (unsigned int)(expected);                        \
        if (_a != _e) {                                                    \
            printf("  FAIL  %s\n        expected %u  got %u\n",           \
                   (name), _e, _a);                                        \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_EQ_FLOAT(actual, expected, tol, name)                       \
    do {                                                                   \
        double _a = (double)(actual);                                      \
        double _e = (double)(expected);                                    \
        double _t = (double)(tol);                                         \
        if (fabs(_a - _e) > _t) {                                         \
            printf("  FAIL  %s\n        expected %.6f  got %.6f\n",       \
                   (name), _e, _a);                                        \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_EQ_STR(actual, expected, name)                              \
    do {                                                                   \
        const char *_a = (actual);                                         \
        const char *_e = (expected);                                       \
        if (strcmp(_a, _e) != 0) {                                        \
            printf("  FAIL  %s\n        expected \"%s\"  got \"%s\"\n",   \
                   (name), _e, _a);                                        \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_EQ_STATUS(actual, expected, name)                           \
    do {                                                                   \
        AisStatus _a = (actual);                                           \
        AisStatus _e = (expected);                                         \
        if (_a != _e) {                                                    \
            printf("  FAIL  %s\n        expected %s  got %s\n",           \
                   (name), ais_status_str(_e), ais_status_str(_a));        \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_NULL(ptr, name)                                             \
    do {                                                                   \
        if ((ptr) != NULL) {                                               \
            printf("  FAIL  %s  (expected NULL)\n", (name));              \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define ASSERT_NOT_NULL(ptr, name)                                         \
    do {                                                                   \
        if ((ptr) == NULL) {                                               \
            printf("  FAIL  %s  (expected non-NULL)\n", (name));          \
            g_failures++;                                                  \
        } else {                                                           \
            printf("  PASS  %s\n", (name));                               \
            g_passes++;                                                    \
        }                                                                  \
    } while (0)

#define RUN_SUITE(fn)                                                      \
    do {                                                                   \
        printf("\n[%s]\n", #fn);                                           \
        fn();                                                              \
    } while (0)

#endif /* TESTLIB_H */
