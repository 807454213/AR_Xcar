#include "function.h"

#include <iostream>

int main()
{
    odomReset();
    const EncoderRawState empty = odomGetEncoderRawState();
    if (empty.valid || empty.pair_seq != 0) {
        std::cerr << "reset encoder raw state should be invalid\n";
        return 1;
    }

    (void)odomAccumEncoderTicks(-20, 40);
    const EncoderRawState first = odomGetEncoderRawState();
    if (!first.valid ||
        first.left_delta != -20 ||
        first.right_delta != 40 ||
        first.avg_abs_delta != 30 ||
        first.pair_seq != 1) {
        std::cerr << "first encoder raw pair mismatch: valid="
                  << first.valid << " left=" << first.left_delta
                  << " right=" << first.right_delta
                  << " avg=" << first.avg_abs_delta
                  << " seq=" << first.pair_seq << "\n";
        return 2;
    }

    (void)odomAccumEncoderTicks(10, 14);
    const EncoderRawState second = odomGetEncoderRawState();
    if (!second.valid ||
        second.left_delta != 10 ||
        second.right_delta != 14 ||
        second.avg_abs_delta != 12 ||
        second.pair_seq != first.pair_seq + 1) {
        std::cerr << "second encoder raw pair mismatch\n";
        return 3;
    }

    return 0;
}
