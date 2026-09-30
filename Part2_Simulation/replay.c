/* Replay recorded trials through the real detector, on a PC.
 *
 * capture.py records the raw sensor columns from the board. This tool feeds
 * those raw values through the EWMA and fall_detector.c that the firmware
 * uses, so a threshold change can be judged against every trial in seconds
 * instead of re-staging each fall by hand.
 *
 * Build and run from the repository root:
 *   gcc -O2 -IAssignment/CG2028_Assignment/Core/Inc Part2_Simulation/replay.c \
 *       Assignment/CG2028_Assignment/Core/Src/fall_detector.c -o replay
 *   ./replay Part2_Simulation/recordings/[*].csv        summary table
 *   ./replay -v Part2_Simulation/recordings/hard_fall_1.csv   event by event
 *
 * A trial counts as expecting a fall when its name contains "fall", which is
 * how capture.py names them. Everything else must stay quiet.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fall_detector.h"

#define ALPHA_ACCEL 50
#define ALPHA_GYRO  30
#define MAX_SAMPLES 20000

static const char *event_name(FallEvent e)
{
    switch (e) {
    case FALL_EVENT_NONE:           return "-";
    case FALL_EVENT_READY:          return "READY";
    case FALL_EVENT_FREE_FALL:      return "FREE FALL";
    case FALL_EVENT_IMPACT:         return "IMPACT";
    case FALL_EVENT_FALL_CONFIRMED: return "FALL CONFIRMED";
    case FALL_EVENT_NEAR_FALL:      return "near-fall rejected";
    case FALL_EVENT_RECOVERED:      return "recovered";
    case FALL_EVENT_ACKNOWLEDGED:   return "acknowledged";
    case FALL_EVENT_LONG_LIE:       return "LONG LIE";
    case FALL_EVENT_SOS:            return "SOS";
    default:                        return "?";
    }
}

/* The same recurrence the assembly routine implements. */
static int ewma(int new_data, int old_output, int alpha_percent)
{
    return (alpha_percent * new_data + (100 - alpha_percent) * old_output) / 100;
}

typedef struct {
    int confirmed;          /* FALL_CONFIRMED events                     */
    int near_falls;         /* triggers that were rejected               */
    int long_lies;
    int samples;
    int filter_mismatch;    /* replayed EWMA != the board's f* columns    */
    uint32_t first_fall_ms;
} Result;

static int replay_file(const char *path, int verbose, Result *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }

    FallDetector detector;
    int accel_filt[3] = {0, 0, 0};
    int gyro_filt[3] = {0, 0, 0};
    int first = 1;
    char line[512];
    memset(out, 0, sizeof(*out));

    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\r' || line[0] == '\n') continue;
        if (line[0] == 't' && line[1] == '_') continue;          /* column names */

        unsigned long t_ms;
        int a[3], g[3], fa[3], fg[3], phase, event, asm_ok;
        if (sscanf(line, "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   &t_ms, &a[0], &a[1], &a[2], &g[0], &g[1], &g[2],
                   &fa[0], &fa[1], &fa[2], &fg[0], &fg[1], &fg[2],
                   &phase, &event, &asm_ok) != 16) {
            continue;
        }

        if (first) {
            FallDetector_Init(&detector, (uint32_t)t_ms);
            first = 0;
        }

        for (int i = 0; i < 3; i++) {
            accel_filt[i] = ewma(a[i], accel_filt[i], ALPHA_ACCEL);
            gyro_filt[i]  = ewma(g[i], gyro_filt[i], ALPHA_GYRO);
            /* Cross-check against what the board computed at capture time. */
            if (accel_filt[i] != fa[i] || gyro_filt[i] != fg[i]) {
                out->filter_mismatch++;
            }
        }

        FallEvent e = FallDetector_Update(&detector, accel_filt, gyro_filt,
                                         (uint32_t)t_ms);
        out->samples++;

        if (e == FALL_EVENT_FALL_CONFIRMED) {
            if (out->confirmed == 0) out->first_fall_ms = (uint32_t)t_ms;
            out->confirmed++;
        } else if (e == FALL_EVENT_NEAR_FALL) {
            out->near_falls++;
        } else if (e == FALL_EVENT_LONG_LIE) {
            out->long_lies++;
        }

        if (verbose && e != FALL_EVENT_NONE) {
            printf("  %7.2f s  %-20s |a|=%4d mg  |w|=%4d dps  tilt=%3d deg",
                   t_ms / 1000.0, event_name(e), detector.accel_mag_mg,
                   detector.gyro_mag_mdps / 1000, detector.tilt_deg);
            if (e == FALL_EVENT_NEAR_FALL) printf("   [%s]", detector.last_reason);
            if (e == FALL_EVENT_FALL_CONFIRMED)
                printf("   rot=%d dps posture=%d deg",
                       detector.last_rotation_mdps / 1000, detector.last_posture_deg);
            printf("\n");
        }
    }
    fclose(f);
    return 0;
}

static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

int main(int argc, char **argv)
{
    int verbose = 0, first_arg = 1;

    if (argc > 1 && strcmp(argv[1], "-v") == 0) {
        verbose = 1;
        first_arg = 2;
    }
    if (first_arg >= argc) {
        fprintf(stderr, "usage: %s [-v] recording.csv [more.csv ...]\n", argv[0]);
        return 2;
    }

    int tp = 0, fn = 0, fp = 0, tn = 0, mismatches = 0;

    printf("%-26s %-9s %-9s %s\n", "trial", "expected", "detected", "detail");
    printf("---------------------------------------------------------------"
           "-----------------\n");

    for (int i = first_arg; i < argc; i++) {
        Result r;
        if (verbose) printf("\n%s:\n", base_name(argv[i]));
        if (replay_file(argv[i], verbose, &r) != 0) continue;

        int expect_fall = (strstr(base_name(argv[i]), "fall") != NULL);
        int detected = (r.confirmed > 0);

        if (expect_fall && detected)        tp++;
        else if (expect_fall && !detected)  fn++;
        else if (!expect_fall && detected)  fp++;
        else                                tn++;
        mismatches += r.filter_mismatch;

        char detail[120];
        if (detected) {
            snprintf(detail, sizeof(detail), "at %.1f s%s, %d near-fall(s)",
                     r.first_fall_ms / 1000.0,
                     r.long_lies ? ", long lie" : "", r.near_falls);
        } else {
            snprintf(detail, sizeof(detail), "%d near-fall(s) rejected", r.near_falls);
        }

        const char *verdict = (expect_fall == detected) ? "ok" : "MISS";
        if (!expect_fall && detected) verdict = "FALSE ALARM";

        printf("%-26s %-9s %-9s %-12s %s\n",
               base_name(argv[i]), expect_fall ? "fall" : "no fall",
               detected ? "FALL" : "-", verdict, detail);
    }

    printf("\nfalls detected      %d of %d\n", tp, tp + fn);
    printf("false alarms        %d of %d non-fall trials\n", fp, fp + tn);
    if (mismatches) {
        printf("\nWARNING: replayed EWMA differs from the board's filtered columns "
               "in %d places.\nCheck that ALPHA_ACCEL/ALPHA_GYRO here match the "
               "firmware that recorded these files.\n", mismatches);
    } else {
        printf("replayed EWMA matches the board's filtered columns exactly\n");
    }
    return (fn > 0 || fp > 0) ? 1 : 0;
}
