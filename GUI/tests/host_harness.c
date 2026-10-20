/* host test harness: reads commands on stdin, prints results. Not part of the firmware. */
#include "esc_params.h"
#include "esc_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
int main(void)
{
    char line[2048];
    esc_params_t p; params_defaults(&p);
    while (fgets(line, sizeof line, stdin)) {
        char cmd[32], a[1024], b[64];
        int n = sscanf(line, "%31s %1023s %63s", cmd, a, b);
        if (n < 1) continue;
        if (!strcmp(cmd, "setname")) { char *q = strstr(line, "setname"); q += 7; while (*q == ' ') q++; q[strcspn(q, "\r\n")] = 0; printf("%s\n", params_set_name(&p, q) ? "OK" : "ERR"); }
        else if (!strcmp(cmd, "set") && n == 3) printf("%s\n", params_set(&p, a, strtof(b, NULL)) ? "OK" : "ERR");
        else if (!strcmp(cmd, "get")) { char buf[96]; for (int i = 0; i < params_count(); i++) { params_format(&p, i, buf, sizeof buf); fputs(buf, stdout);} params_format_name(&p, buf, sizeof buf); fputs(buf, stdout); printf("OK\n"); }
        else if (!strcmp(cmd, "derive")) { esc_eff_t e; params_derive(&p, strtof(a, NULL), &e); printf("D,%.6f,%.6f,%.6f,%.6f,%.6f\n", e.boost_v, e.align_v, e.max_mod, e.rpm_limit, params_max_rpm(&p, strtof(a, NULL))); }
        else if (!strcmp(cmd, "pack")) { esc_saved_t s; float off[3] = {1350.f, 1351.f, 1352.f}; params_pack(&s, SAVED_FLAG_PAR, &p, off); const unsigned char *u = (const unsigned char *)&s; for (size_t i = 0; i < sizeof s; i++) printf("%02x", u[i]); printf("\n"); }
        else if (!strcmp(cmd, "unpack") && n == 2) { esc_saved_t s; for (size_t i = 0; i < sizeof s; i++) { unsigned v; sscanf(a + 2*i, "%2x", &v); ((unsigned char*)&s)[i] = (unsigned char)v; } esc_params_t q; uint32_t fl; float off[3]; bool ok = params_unpack(&s, &fl, &q, off); printf("%s\n", ok ? "VALID" : "INVALID"); if (ok && (fl & SAVED_FLAG_PAR)) p = q; }
        else if (!strcmp(cmd, "defaults")) { params_defaults(&p); printf("OK\n"); }
        else if (!strcmp(cmd, "size")) printf("%zu %zu\n", sizeof(esc_params_t), sizeof(esc_saved_t));
        fflush(stdout);
    }
    return 0;
}
