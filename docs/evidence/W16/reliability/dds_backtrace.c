#define _GNU_SOURCE
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
static void crash(int sig) {
    void *frames[96];
    const char marker[] = "\nW16 diagnostic crash backtrace (original fatal signal retained)\n";
    write(2, marker, sizeof(marker)-1);
    int n = backtrace(frames, 96);
    backtrace_symbols_fd(frames, n, 2);
    /* SA_RESETHAND: return to the faulting instruction and original disposition. */
}
__attribute__((constructor)) static void setup(void) {
    struct sigaction action = {0};
    action.sa_handler = crash;
    action.sa_flags = SA_RESETHAND;
    sigemptyset(&action.sa_mask);
    sigaction(SIGSEGV, &action, 0);
    sigaction(SIGABRT, &action, 0);
}
