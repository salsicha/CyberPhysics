#include "demo.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
static volatile sig_atomic_t running=1;
static void stop(int sig) { (void)sig; running=0; }
bool cp_keep_running(void) { return running != 0; }
void cp_set_led(bool enabled) { printf("LED %d\n", enabled); fflush(stdout); }
int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (argc != 3) { fprintf(stderr, "Usage: %s LOCATOR ROS_DOMAIN_ID\n", argv[0]); return 2; }
    char *end=NULL;
    errno=0;
    unsigned long domain=strtoul(argv[2], &end, 10);
    if (errno || !*argv[2] || *end || domain>232) return 2;
    signal(SIGINT,stop); signal(SIGTERM,stop);
    return cp_demo_run(argv[1], (uint32_t)domain);
}
