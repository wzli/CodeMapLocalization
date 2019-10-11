#include "tests.h"

int main() {
    run_all_tests();
    printf("%s\ntests ran: %d\n", test_error, test_count);
    return 0;
}
