#ifndef QEI_H
#define QEI_H

#include "mbed.h"

#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.
class QEI {

public:

    typedef enum Encoding {

        X2_ENCODING,
        X4_ENCODING

    } Encoding;
    QEI(PinName channelA, PinName channelB, PinName index, int pulsesPerRev, Encoding encoding = X2_ENCODING);

    /**
     * Reset the encoder.
     *
     * Sets the pulses and revolutions count to zero.
     */
    void reset(void);

    /**
     * Read the state of the encoder.
     *
     * @return The current state of the encoder as a 2-bit number, where:
     *         bit 1 = The reading from channel B
     *         bit 2 = The reading from channel A
     */
    int getCurrentState(void);

    /**
     * Read the number of pulses recorded by the encoder.
     *
     * @return Number of pulses which have occured.
     */
    int getPulses(void);

    /**
     * Read the number of revolutions recorded by the encoder on the index channel.
     *
     * @return Number of revolutions which have occured on the index channel.
     */
    int getRevolutions(void);

private:

    /**
     * Update the pulse count.
     *
     * Called on every rising/falling edge of channels A/B.
     *
     * Reads the state of the channels and determines whether a pulse forward
     * or backward has occured, updating the count appropriately.
     */
    void encode(void);

    /**
     * Called on every rising edge of channel index to update revolution
     * count by one.
     */
    void index(void);

    Encoding encoding_;

    InterruptIn channelA_;
    InterruptIn channelB_;
    InterruptIn index_;

    int          pulsesPerRev_;
    int          prevState_;
    int          currState_;

    volatile int pulses_;
    volatile int revolutions_;

};

#endif /* QEI_H */
