#pragma once
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Test framework structures and types
typedef struct {
    char name[100];
    bool (*test_func)(void);
    bool run;
    bool passed;
} test_case_t;

typedef struct {
    char name[100];
    test_case_t* cases;
    int num_cases;
    int passed;
    int failed;
} test_suite_t;

// Assertion macros
#define ASSERT_TRUE(condition) \
do { \
if (!(condition)) { \
printf("Assertion failed: %s\n", #condition); \
return false; \
} \
} while (0)

#define ASSERT_FALSE(condition) ASSERT_TRUE(!(condition))

#define ASSERT_EQUAL(expected, actual) \
do { \
if ((expected) != (actual)) { \
printf("Assertion failed: expected %d, got %d\n", (expected), (actual)); \
return false; \
} \
} while (0)

#define ASSERT_FLOAT_EQUAL(expected, actual, epsilon) \
do { \
if (fabs((expected) - (actual)) > (epsilon)) { \
printf("Assertion failed: expected %f, got %f (epsilon: %f)\n", \
(float)(expected), (float)(actual), (float)(epsilon)); \
return false; \
} \
} while (0)

// Test runner functions
void run_test_suite(test_suite_t* suite);
void print_test_results(const test_suite_t* suite);