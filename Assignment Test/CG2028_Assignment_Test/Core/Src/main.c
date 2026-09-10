/******************************************************************************
 * @file           : main_test.c
 * @brief          : Open-test harness for the CG2028 Assignment EWMA filter
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

#include "main.h"
#include <stdio.h>

extern void initialise_monitor_handles(void);
extern int ewma_filter(int new_data, int old_output, int alpha_percent);

static int ewma_filter_C(int new_data, int old_output, int alpha_percent);
static int run_test_case(const char *name,
                         int alpha_percent,
                         const int sensor_data_x[16],
                         const int sensor_data_y[16],
                         const int sensor_data_z[16]);

static const int test1_x[16] =
    {1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015};
static const int test1_y[16] =
    {1016,1017,1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031};
static const int test1_z[16] =
    {1032,1033,1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047};

static const int test2_x[16] =
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static const int test2_y[16] =
    {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
static const int test2_z[16] =
    {1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000};

static const int test3_x[16] =
    {6030,6000,4389,-4488,6734,5009,-7318,6040,5000,-5643,3888,3488,-3488,3488,4876,5010};
static const int test3_y[16] =
    {-9800,9800,6573,6753,-5004,5895,4321,-6753,9004,3005,-6934,4444,7981,-5144,5867,4178};
static const int test3_z[16] =
    {5455,-6177,8653,7777,6520,-4566,7860,5199,-6233,5444,9822,-3455,6445,7888,-4113,4998};

static const int test4_x[16] =
    {0,0,0,0,4000,4000,4000,4000,-4000,-4000,-4000,-4000,1000,2000,3000,4000};
static const int test4_y[16] =
    {-2000,-1500,-1000,-500,0,500,1000,1500,2000,1500,1000,500,0,-500,-1000,-1500};
static const int test4_z[16] =
    {100,-100,200,-200,400,-400,800,-800,1600,-1600,3200,-3200,6400,-6400,1280,-1280};

static const int test5_x[16] =
    {1,2,3,4,5,6,7,8,9,10,-1,-2,-3,-4,-5,-6};
static const int test5_y[16] =
    {99,100,101,102,-99,-100,-101,-102,333,-333,666,-666,999,-999,50,-50};
static const int test5_z[16] =
    {1234,2345,-3456,4567,-5678,6789,-7890,8901,-9012,101,202,-303,404,-505,606,-707};

int main(void)
{
    initialise_monitor_handles();
    HAL_Init();

    int failures = 0;
    failures += run_test_case("Open test case 1", 25, test1_x, test1_y, test1_z);
    failures += run_test_case("Open test case 2", 50, test2_x, test2_y, test2_z);
    failures += run_test_case("Open test case 3", 10, test3_x, test3_y, test3_z);
    failures += run_test_case("Open test case 4", 75, test4_x, test4_y, test4_z);
    failures += run_test_case("Open test case 5", 33, test5_x, test5_y, test5_z);

    if (failures == 0)
    {
        printf("\nALL 5 OPEN TEST CASES PASSED\n");
    }
    else
    {
        printf("\nTEST FAILED: %d mismatch(es) detected\n", failures);
    }

    printf("Exiting main\n");
    while (1) { }
}

static int run_test_case(const char *name,
                         int alpha_percent,
                         const int sensor_data_x[16],
                         const int sensor_data_y[16],
                         const int sensor_data_z[16])
{
    int old_c[3] = {0, 0, 0};
    int old_asm[3] = {0, 0, 0};
    int failures = 0;

    printf("\n%s (alpha = %d%%)\n", name, alpha_percent);

    for (int i = 0; i < 16; i++)
    {
        int input[3] = {sensor_data_x[i], sensor_data_y[i], sensor_data_z[i]};
        int expected[3];
        int actual[3];

        for (int axis = 0; axis < 3; axis++)
        {
            expected[axis] = ewma_filter_C(input[axis], old_c[axis], alpha_percent);
            actual[axis] = ewma_filter(input[axis], old_asm[axis], alpha_percent);
            old_c[axis] = expected[axis];
            old_asm[axis] = actual[axis];
        }

        printf("Sample %2d expected {%d,%d,%d}; actual {%d,%d,%d}\n",
               i + 1,
               expected[0], expected[1], expected[2],
               actual[0], actual[1], actual[2]);

        if ((expected[0] != actual[0]) ||
            (expected[1] != actual[1]) ||
            (expected[2] != actual[2]))
        {
            failures++;
        }
    }

    printf("%s: %s\n", name, failures == 0 ? "PASSED" : "FAILED");
    return failures;
}

static int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}
