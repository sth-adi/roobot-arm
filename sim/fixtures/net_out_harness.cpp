// Drives arduino/arm_receiver/net_out.h from stdin. Used by sim/test_net_out.py.
//   push <text>   take <n>   clear   stat
#include <stdio.h>
#include <string.h>
#include "net_out.h"

int main()
{
    ByteQueue<16> q;
    char cmd[16], arg[256];
    while (scanf("%15s", cmd) == 1) {
        if (!strcmp(cmd, "push")) {
            scanf("%255s", arg);
            for (const char *p = arg; *p; p++) q.push((uint8_t)*p);
        } else if (!strcmp(cmd, "take")) {
            int n;
            scanf("%d", &n);
            uint8_t out[64] = {0};
            size_t got = q.take(out, (size_t)n);
            printf("took %zu:%.*s\n", got, (int)got, (const char *)out);
        } else if (!strcmp(cmd, "clear")) {
            q.clear();
        } else if (!strcmp(cmd, "stat")) {
            printf("count %zu dropped %lu\n", q.count, q.dropped);
        }
    }
    return 0;
}
