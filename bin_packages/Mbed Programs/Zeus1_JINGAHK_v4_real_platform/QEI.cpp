#include "QEI.h"

QEI::QEI(PinName channelA,
         PinName channelB,
         PinName index,
         int pulsesPerRev,
         Encoding encoding) : channelA_(channelA), channelB_(channelB),
        index_(index) {

    pulses_       = 0;
    revolutions_  = 0;
    pulsesPerRev_ = pulsesPerRev;
    encoding_     = encoding;

    //Workout what the current state is.
    int chanA = channelA_.read();
    int chanB = channelB_.read();

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);
    prevState_ = currState_;

    //X2 encoding uses interrupts on only channel A.
    //X4 encoding uses interrupts on      channel A,
    //and on channel B.
    channelA_.rise(callback(this, &QEI::encode));
    channelA_.fall(callback(this, &QEI::encode));

    //If we're using X4 encoding, then attach interrupts to channel B too.
    if (encoding == X4_ENCODING) {
        channelB_.rise(callback(this, &QEI::encode));
        channelB_.fall(callback(this, &QEI::encode));
    }
    //Index is optional.
    if (index !=  NC) {
        index_.rise(callback(this, &QEI::index));
    }

}

void QEI::reset(void) {

    pulses_      = 0;
    revolutions_ = 0;

}

int QEI::getCurrentState(void) {

    return currState_;

}

int QEI::getPulses(void) {

    return pulses_;

}

int QEI::getRevolutions(void) {

    return revolutions_;

}

void QEI::encode(void) {

    int change = 0;
    int chanA  = channelA_.read();
    int chanB  = channelB_.read();

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);

    if (encoding_ == X2_ENCODING) {

        //11->00->11->00 is counter clockwise rotation or "forward".
        if ((prevState_ == 0x3 && currState_ == 0x0) ||
                (prevState_ == 0x0 && currState_ == 0x3)) {

            pulses_++;

        }
        //10->01->10->01 is clockwise rotation or "backward".
        else if ((prevState_ == 0x2 && currState_ == 0x1) ||
                 (prevState_ == 0x1 && currState_ == 0x2)) {

            pulses_--;

        }

    } else if (encoding_ == X4_ENCODING) {

        //Entered a new valid state.
        if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_)) {
            //2 bit state. Right hand bit of prev XOR left hand bit of current
            //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
            change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);

            if (change == 0) {
                change = -1;
            }

            pulses_ -= change;
        }

    }

    prevState_ = currState_;

}

void QEI::index(void) {
    
    revolutions_++;

}
