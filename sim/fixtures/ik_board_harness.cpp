// Reads commands on stdin and prints what the sketch's mirror IK (arduino/arm_receiver/arm_ik.h) would do. Used by sim/test_ik_board.py.
//   cal l1 h1 l2 h2 l3 h3 l4 h4 home1 home2 home3 home4   -> "cal ok" or "cal refused"
//   pose x y z angle                                       -> four servo degrees, or "fail"
#include <stdio.h>
#include <string.h>
#include "arm_ik.h"

int main()
{
    char word[16];
    while (scanf("%15s", word) == 1) {
        if (!strcmp(word, "cal")) {
            int c[12];
            for (int i = 0; i < 12; i++) scanf("%d", &c[i]);
            const int lims[4][2] = {{c[0], c[1]}, {c[2], c[3]}, {c[4], c[5]}, {c[6], c[7]}};
            const int homes[4] = {c[8], c[9], c[10], c[11]};
            printf(MirrorSetCalibration(lims, homes) ? "cal ok\n" : "cal refused\n");
        } else if (!strcmp(word, "pose")) {
            float x, y, z, a, d[4];
            scanf("%f %f %f %f", &x, &y, &z, &a);
            if (MirrorPoseToServoDegrees(x, y, z, a, d)) printf("%.4f %.4f %.4f %.4f\n", d[0], d[1], d[2], d[3]);
            else printf("fail\n");
        }
    }
    return 0;
}
