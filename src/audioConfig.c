/****************************************************************************
 * audioconfig.h
 * openacousticdevices.info
 * May 2020
 *****************************************************************************/

#include "biquad.h"
#include "audioMoth.h"
#include "butterworth.h"
#include "audioConfig.h"

/* Audio configuration constants */

#define CONFIG_SAMPLE_RATE                  48000
#define CONFIG_OVERSAMPLE_RATE              8
#define CONFIG_GAIN                         4
#define CONFIG_ACQUISITION_CYCLES           16
#define CONFIG_CLOCK_DIVIDER                4

#define CONFIG_CARRIER_FREQUENCY            18000

#define SINE_TABLE_LENGTH                   256
#define SINE_QUARTER_CYCLE_INCREMENT        (SINE_TABLE_LENGTH / 4)
#define SINE_CARRIER_PHASE_INCREMENT        (CONFIG_CARRIER_FREQUENCY * SINE_TABLE_LENGTH / CONFIG_SAMPLE_RATE)

#define PULSE_INTERVAL                      4096

#define MAXIMUM_NUMBER_OF_BYTES             16
#define RECEIVE_BUFFER_SIZE_IN_BYTES        16

#define CRC_POLY                            0x1021
#define CRC_SIZE_IN_BYTES                   2

#define ENCODED_BITS_IN_BYTE                14
#define BITS_IN_BYTE                        8

#define SPEED_FACTOR                        1
#define USE_HAMMING_CODE                    true

#define MIN_BIT_PERIOD                      (120 / SPEED_FACTOR)
#define MAX_BIT_PERIOD                      (600 / SPEED_FACTOR)

#define LOW_BIT_PERIOD                      (240 / SPEED_FACTOR)
#define HIGH_BIT_PERIOD                     (480 / SPEED_FACTOR)
#define MID_BIT_PERIOD                      (LOW_BIT_PERIOD / 2 + HIGH_BIT_PERIOD / 2)

#define START_STOP_BIT_PERIOD               (360 / SPEED_FACTOR)

#define PERIOD_TOLERANCE                    (60 / SPEED_FACTOR)

#define MIN_NUMBER_OF_START_STOP_PERIODS    4
#define MAX_NUMBER_OF_START_STOP_PERIODS    24

#define VCO_GAIN                            0.5f

#define AGC_FILTER_CUTOFF_FREQUENCY         20
#define CHANNEL_FILTER_CUTOFF_FREQUENCY     100
#define CHANNEL_FILTER_BANDWIDTH            2.0f
#define CARRIER_FILTER_CUTOFF_FREQUENCY     1000

#define MILLISECONDS_IN_SECOND              1000

#define NUMBER_OF_START_BITS_TO_DETECT      10

#define MAXIMUM_LISTENING_MILLISECONDS      60000

/* In period macro */

#define IN_PERIOD(period, mean, range)      (((period) > ((mean) - (range))) && ((period) < ((mean) + (range))))

/* Useful macros */

#define STATIC_UBUF(x, y)                   static uint8_t x[((y) + 3) & ~3] __attribute__ ((aligned(4)))

#define MIN(a, b)                           ((a) < (b) ? (a) : (b))

/* Sine table */

static const float sineTable[SINE_TABLE_LENGTH] = {0.000000000000f, 0.024541229010f, 0.049067676067f, 0.073564566672f, 0.098017141223f, 0.122410677373f, 0.146730467677f, 0.170961901546f, \
                                                   0.195090323687f, 0.219101235271f, 0.242980197072f, 0.266712784767f, 0.290284663439f, 0.313681751490f, 0.336889863014f, 0.359895050526f, \
                                                   0.382683455944f, 0.405241340399f, 0.427555084229f, 0.449611335993f, 0.471396744251f, 0.492898225784f, 0.514102756977f, 0.534997642040f, \
                                                   0.555570244789f, 0.575808227062f, 0.595699310303f, 0.615231633186f, 0.634393334389f, 0.653172850609f, 0.671558976173f, 0.689540565014f, \
                                                   0.707106769085f, 0.724247097969f, 0.740951180458f, 0.757208883762f, 0.773010432720f, 0.788346409798f, 0.803207516670f, 0.817584812641f, \
                                                   0.831469655037f, 0.844853579998f, 0.857728660107f, 0.870086967945f, 0.881921291351f, 0.893224298954f, 0.903989315033f, 0.914209783077f, \
                                                   0.923879504204f, 0.932992815971f, 0.941544055939f, 0.949528217316f, 0.956940352917f, 0.963776051998f, 0.970031261444f, 0.975702106953f, \
                                                   0.980785310268f, 0.985277652740f, 0.989176511765f, 0.992479562759f, 0.995184719563f, 0.997290432453f, 0.998795449734f, 0.999698817730f, \
                                                   1.000000000000f, 0.999698817730f, 0.998795449734f, 0.997290432453f, 0.995184719563f, 0.992479503155f, 0.989176511765f, 0.985277652740f, \
                                                   0.980785250664f, 0.975702106953f, 0.970031261444f, 0.963776051998f, 0.956940293312f, 0.949528157711f, 0.941544055939f, 0.932992815971f, \
                                                   0.923879504204f, 0.914209723473f, 0.903989315033f, 0.893224298954f, 0.881921231747f, 0.870087027550f, 0.857728600502f, 0.844853520393f, \
                                                   0.831469535828f, 0.817584812641f, 0.803207516670f, 0.788346350193f, 0.773010492325f, 0.757208824158f, 0.740951061249f, 0.724246978760f, \
                                                   0.707106769085f, 0.689540505409f, 0.671558856964f, 0.653172850609f, 0.634393274784f, 0.615231513977f, 0.595699131489f, 0.575808167458f, \
                                                   0.555570185184f, 0.534997463226f, 0.514102756977f, 0.492898136377f, 0.471396625042f, 0.449611365795f, 0.427555054426f, 0.405241221189f, \
                                                   0.382683277130f, 0.359895050526f, 0.336889803410f, 0.313681602478f, 0.290284723043f, 0.266712725163f, 0.242980077863f, 0.219101071358f, \
                                                   0.195090308785f, 0.170961812139f, 0.146730333567f, 0.122410699725f, 0.098017096519f, 0.073564447463f, 0.049067486078f, 0.024541210383f, \

                                                  -0.000000087423f, -0.024541385472f, -0.049067661166f, -0.073564618826f, -0.098017267883f, -0.122410871089f, -0.146730497479f, -0.170961990952f, \
                                                  -0.195090487599f, -0.219101235271f, -0.242980241776f, -0.266712903976f, -0.290284872055f, -0.313681781292f, -0.336889952421f, -0.359895199537f, \
                                                  -0.382683426142f, -0.405241400003f, -0.427555233240f, -0.449611514807f, -0.471396774054f, -0.492898285389f, -0.514102876186f, -0.534997642040f, \
                                                  -0.555570304394f, -0.575808346272f, -0.595699310303f, -0.615231633186f, -0.634393393993f, -0.653172969818f, -0.671558976173f, -0.689540624619f, \
                                                  -0.707106888294f, -0.724247097969f, -0.740951180458f, -0.757208764553f, -0.773010432720f, -0.788346469402f, -0.803207576275f, -0.817584931850f, \
                                                  -0.831469774246f, -0.844853758812f, -0.857728540897f, -0.870086967945f, -0.881921291351f, -0.893224358559f, -0.903989374638f, -0.914209842682f, \
                                                  -0.923879683018f, -0.932992756367f, -0.941544055939f, -0.949528217316f, -0.956940352917f, -0.963776111603f, -0.970031321049f, -0.975702226162f, \
                                                  -0.980785250664f, -0.985277652740f, -0.989176511765f, -0.992479562759f, -0.995184719563f, -0.997290492058f, -0.998795449734f, -0.999698817730f, \
                                                  -1.000000000000f, -0.999698817730f, -0.998795449734f, -0.997290432453f, -0.995184719563f, -0.992479503155f, -0.989176511765f, -0.985277652740f, \
                                                  -0.980785250664f, -0.975702106953f, -0.970031201839f, -0.963775992393f, -0.956940233707f, -0.949528217316f, -0.941544055939f, -0.932992756367f, \
                                                  -0.923879444599f, -0.914209663868f, -0.903989136219f, -0.893224120140f, -0.881921291351f, -0.870086967945f, -0.857728540897f, -0.844853460789f, \
                                                  -0.831469476223f, -0.817584633827f, -0.803207576275f, -0.788346409798f, -0.773010432720f, -0.757208764553f, -0.740951001644f, -0.724246919155f, \
                                                  -0.707106530666f, -0.689540624619f, -0.671558976173f, -0.653172791004f, -0.634393155575f, -0.615231454372f, -0.595699071884f, -0.575807929039f, \
                                                  -0.555570304394f, -0.534997642040f, -0.514102697372f, -0.492898076773f, -0.471396535635f, -0.449611067772f, -0.427554786205f, -0.405241370201f, \
                                                  -0.382683426142f, -0.359894961119f, -0.336889714003f, -0.313681542873f, -0.290284395218f, -0.266712397337f, -0.242980226874f, -0.219101220369f, \
                                                  -0.195090234280f, -0.170961722732f, -0.146730244160f, -0.122410371900f, -0.098016768694f, -0.073564596474f, -0.049067638814f, -0.024541122839f};

/* Hamming decoding table */

static const uint8_t hammingConversion[128] = { 0,  0,  0,  1,  0,  1,  1,  1, \
                                                0,  2,  4,  8,  9,  5,  3,  1, \
                                                0,  2, 10,  6,  7, 11,  3,  1, \
                                                2,  2,  3,  2,  3,  2,  3,  3, \
                                                0, 12,  4,  6,  7,  5, 13,  1, \
                                                4,  5,  4,  4,  5,  5,  4,  5, \
                                                7,  6,  6,  6,  7,  7,  7,  6, \
                                               14,  2,  4,  6,  7,  5,  3, 15, \
                                                0, 12, 10,  8,  9, 11, 13,  1, \
                                                9,  8,  8,  8,  9,  9,  9,  8, \
                                               10, 11, 10, 10, 11, 11, 10, 11, \
                                               14,  2, 10,  8,  9, 11,  3, 15, \
                                               12, 12, 13, 12, 13, 12, 13, 13, \
                                               14, 12,  4,  8,  9,  5, 13, 15, \
                                               14, 12, 10,  6,  7, 11, 13, 15, \
                                               14, 14, 14, 15, 14, 15, 15, 15};

/* Filter variables */

static BW_filter_t agcFilter;

static BW_filter_t carrierFilter;

static BQ_filter_t channel1Filter;

static BQ_filter_t channel2Filter;

/* Filter coefficient variables */

static BW_filterCoefficients_t agcFilterCoefficients;

static BW_filterCoefficients_t carrierFilterCoefficients;

static BQ_filterCoefficients_t channelFilterCoefficients;

/* Carrier generation variable */

float omegaT = 0.0f;

/* Configuration variables */

static volatile bool cancel;

static volatile int16_t configSample;

static volatile bool configSampleReady;

STATIC_UBUF(receivedBytes, RECEIVE_BUFFER_SIZE_IN_BYTES);

/* Configuration states */

typedef enum {IDLE, START_BITS, DATA_BITS, DATA_OR_STOP_BITS} state_t;

typedef enum {NONE, HIGH_BIT, LOW_BIT} receivedBit_t;

/* CRC functions */

static uint16_t updateCRC(uint16_t crc_in, int incr) {

    uint16_t xor = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr) {
        out++;
    }

    if (xor) {
        out ^= CRC_POLY;
    }

    return out;

}

static uint16_t calculateCRC(const uint8_t *data, uint32_t size) {

    uint16_t crc, i;

    for (crc = 0; size > 0; size--, data++) {
        for (i = 0x80; i; i >>= 1) {
            crc = updateCRC(crc, *data & i);
        }
    }

    for (i = 0; i < 16; i++) {
        crc = updateCRC(crc, 0);
    }

    return crc;

}

static bool checkCRC(const uint8_t *data, uint32_t size) {

    uint16_t crc = calculateCRC(data, size - CRC_SIZE_IN_BYTES);

    uint8_t low = crc & 0xFF;
    uint8_t high = crc >> 8;

    return (low == data[size - 2] && high == data[size - 1]);

}

/* Function to perform Costas loop */

float updateCostasLoop(float sample) {

    /* Apply gain control */

    float filteredSample = Butterworth_applyBandPassFilter(sample, &carrierFilter, &carrierFilterCoefficients);

    float agcOutput = Butterworth_applyLowPassFilter(filteredSample > 0 ? filteredSample : -filteredSample, &agcFilter, &agcFilterCoefficients);

    if (agcOutput != 0) filteredSample /= agcOutput;

    /* Demodulate input sound */

    uint8_t index = (uint32_t)omegaT & (SINE_TABLE_LENGTH - 1);

    float channel1 = Biquad_applyFilter(sineTable[index] * filteredSample, &channel1Filter, &channelFilterCoefficients);

    index -= SINE_QUARTER_CYCLE_INCREMENT;

    float channel2 = Biquad_applyFilter(sineTable[index] * filteredSample, &channel2Filter, &channelFilterCoefficients);

    float controlSignal = channel1 * channel2;

    /* Update the VCO */

    omegaT += SINE_CARRIER_PHASE_INCREMENT - VCO_GAIN * controlSignal;

    uint32_t multiple = (uint32_t)omegaT / SINE_TABLE_LENGTH;

    omegaT -= multiple * SINE_TABLE_LENGTH;

    /* Return in phase channel */

    return channel1;

}

/* Handle AudioMoth microphone interrupt */

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) {

    configSample = sample;

    configSampleReady = true;

}

/* Functions to handle audio configuration */

void AudioConfig_enableAudioConfiguration() {

    /* Initialise microphone for configuration */

    AudioMoth_enableMicrophone(CONFIG_GAIN, CONFIG_CLOCK_DIVIDER, CONFIG_ACQUISITION_CYCLES, CONFIG_OVERSAMPLE_RATE);

    AudioMoth_startMicrophoneSamples(CONFIG_SAMPLE_RATE);

    AudioMoth_initialiseMicrophoneInterupts();

    /* Design filters */

    Butterworth_designLowPassFilter(&agcFilterCoefficients, CONFIG_SAMPLE_RATE, SPEED_FACTOR * AGC_FILTER_CUTOFF_FREQUENCY);

    Butterworth_designBandPassFilter(&carrierFilterCoefficients, CONFIG_SAMPLE_RATE, CONFIG_CARRIER_FREQUENCY - CARRIER_FILTER_CUTOFF_FREQUENCY, CONFIG_CARRIER_FREQUENCY + CARRIER_FILTER_CUTOFF_FREQUENCY);

    Biquad_designLowPassFilter(&channelFilterCoefficients, CONFIG_SAMPLE_RATE, SPEED_FACTOR * CHANNEL_FILTER_CUTOFF_FREQUENCY, CHANNEL_FILTER_BANDWIDTH);

    /* Initialise filters */

    Butterworth_initialise(&agcFilter);

    Butterworth_initialise(&carrierFilter);

    Biquad_initialise(&channel1Filter);

    Biquad_initialise(&channel2Filter);

}

void AudioConfig_disableAudioConfiguration() {

    AudioMoth_disableMicrophone();

}

bool AudioConfig_listenForAudioConfigurationTone(uint32_t milliseconds) {

    /* Interrupt flags */

    cancel = false;

    configSampleReady = false;

    /* Zero crossing variables */

    float lastValue = 0.0f;

    uint32_t lastCrossing = 0;

    receivedBit_t lastBit = NONE;

    /* Receive state variables */

    uint32_t bitCount = 0;

    /* Main loop */

    uint32_t counter = 0;

    uint32_t maximumCounter = MIN(milliseconds, MAXIMUM_LISTENING_MILLISECONDS) * CONFIG_SAMPLE_RATE / MILLISECONDS_IN_SECOND;

    while (cancel == false && counter < maximumCounter) {

        if (configSampleReady) {

            /* Update the Costas loop with new sample */

            float costasLoopOutput = updateCostasLoop((float)configSample);

            /* Check thresholds */

            if ((lastValue > 0 && costasLoopOutput < 0) || (lastValue < 0 && costasLoopOutput > 0)) {

                uint32_t period = counter - lastCrossing;

                receivedBit_t currentBit = IN_PERIOD(period, LOW_BIT_PERIOD, PERIOD_TOLERANCE) ? LOW_BIT : IN_PERIOD(period, HIGH_BIT_PERIOD, PERIOD_TOLERANCE) ? HIGH_BIT : NONE;

                if ((currentBit == LOW_BIT && lastBit == HIGH_BIT) || (currentBit == HIGH_BIT && lastBit == LOW_BIT)) {

                    bitCount += 1;

                } else {

                    bitCount = 0;

                }

                if (bitCount == NUMBER_OF_START_BITS_TO_DETECT) return true;

                lastCrossing = counter;

                lastBit = currentBit;

            }

            lastValue = costasLoopOutput;

            counter += 1;

        }

        /* Wait for next sample */

        configSampleReady = false;

        /* Sleep until next interrupt occurs */

        AudioMoth_sleep();

    }

    return false;

}

bool AudioConfig_listenForAudioConfigurationPackets(bool timeout, uint32_t milliseconds) {

    /* Interrupt flags */

    cancel = false;

    configSampleReady = false;

    /* Zero crossing variables */

    float lastValue = 0.0f;

    uint32_t lastCrossing = 0;

    /* Receive state variables */

    uint32_t bitCount = 0;

    uint32_t byteCount = 0;

    uint8_t receivedByte = 0;

    state_t state = IDLE;

    uint8_t receivedHammingCodes[2];

    /* Main loop */

    uint32_t counter = 0;

    uint32_t maximumCounter = MIN(milliseconds, MAXIMUM_LISTENING_MILLISECONDS) * CONFIG_SAMPLE_RATE / MILLISECONDS_IN_SECOND;

    while (cancel == false && (timeout == false || counter < maximumCounter)) {

        if (configSampleReady) {

            /* Call pulse handler */

            if (counter % PULSE_INTERVAL == 0) AudioConfig_handleAudioConfigurationEvent(AC_EVENT_PULSE);

            /* Update the Costas loop with new sample */

            float costasLoopOutput = updateCostasLoop((float)configSample);

            /* Check thresholds */

            if ((lastValue > 0 && costasLoopOutput < 0) || (lastValue < 0 && costasLoopOutput > 0)) {

                uint32_t period = counter - lastCrossing;

                if (state == IDLE) {

                    if (IN_PERIOD(period, START_STOP_BIT_PERIOD, PERIOD_TOLERANCE)) {

                        state = START_BITS;

                        bitCount = 0;

                    }

                } else if (state == START_BITS) {

                    if (IN_PERIOD(period, START_STOP_BIT_PERIOD, PERIOD_TOLERANCE)) {

                        bitCount += 1;

                    } else if (IN_PERIOD(period, LOW_BIT_PERIOD, PERIOD_TOLERANCE) || IN_PERIOD(period, HIGH_BIT_PERIOD, PERIOD_TOLERANCE)) {

                        if (bitCount > MIN_NUMBER_OF_START_STOP_PERIODS && bitCount < MAX_NUMBER_OF_START_STOP_PERIODS) {

                            AudioConfig_handleAudioConfigurationEvent(AC_EVENT_START);

                            receivedHammingCodes[0] = 0;

                            receivedHammingCodes[1] = 0;

                            receivedByte = 0;

                            state = DATA_BITS;

                            byteCount = 0;

                            bitCount = 0;

                        } else {

                            state = IDLE;

                        }

                    } else {

                        state = IDLE;

                    }

                }

                if (state == DATA_BITS || state == DATA_OR_STOP_BITS) {

                    if (period > MIN_BIT_PERIOD && period < MAX_BIT_PERIOD) {

                        /* Determine the received bit */

                        if (period > MID_BIT_PERIOD) {

                            if (USE_HAMMING_CODE) {

                                uint8_t mask = 1 << (bitCount >> 1);

                                receivedHammingCodes[bitCount % 2] |= mask;

                            } else {

                                uint8_t mask = 1 << bitCount;

                                receivedByte |= mask;

                            }

                        }

                        bitCount += 1;

                        /* Check if this could still be a stop bit */

                        if (!IN_PERIOD(period, START_STOP_BIT_PERIOD, PERIOD_TOLERANCE)) {

                            state = DATA_BITS;

                        }

                        /* Check if stop condition met */

                        if (bitCount == MIN_NUMBER_OF_START_STOP_PERIODS && state == DATA_OR_STOP_BITS && byteCount > CRC_SIZE_IN_BYTES) {

                            if (checkCRC(receivedBytes, byteCount)) {

                                AudioConfig_handleAudioConfigurationPacket(receivedBytes, byteCount - CRC_SIZE_IN_BYTES);

                            } else {

                                AudioConfig_handleAudioConfigurationEvent(AC_EVENT_CRC_ERROR);

                            }

                            state = IDLE;

                        }

                        /* Check if full byte has been received */

                        uint32_t requiredNumberOfBits = USE_HAMMING_CODE ? ENCODED_BITS_IN_BYTE : BITS_IN_BYTE;

                        if (bitCount == requiredNumberOfBits) {

                            if (USE_HAMMING_CODE) {

                                receivedBytes[byteCount] = hammingConversion[receivedHammingCodes[1]] << 4;

                                receivedBytes[byteCount] |= hammingConversion[receivedHammingCodes[0]];

                            } else {

                                receivedBytes[byteCount] = receivedByte;

                            }

                            byteCount += 1;

                            /* Check the CRC if all bytes have been received */

                            if (byteCount == MAXIMUM_NUMBER_OF_BYTES) {

                                if (checkCRC(receivedBytes, MAXIMUM_NUMBER_OF_BYTES)) {

                                    AudioConfig_handleAudioConfigurationPacket(receivedBytes, MAXIMUM_NUMBER_OF_BYTES - CRC_SIZE_IN_BYTES);

                                } else {

                                     AudioConfig_handleAudioConfigurationEvent(AC_EVENT_CRC_ERROR);

                                }

                                state = IDLE;

                            } else {

                                AudioConfig_handleAudioConfigurationEvent(AC_EVENT_BYTE);

                                receivedHammingCodes[0] = 0;

                                receivedHammingCodes[1] = 0;

                                receivedByte = 0;

                                state = DATA_OR_STOP_BITS;

                                bitCount = 0;

                            }

                        }

                    } else {

                        AudioConfig_handleAudioConfigurationEvent(AC_EVENT_BIT_ERROR);

                        state = IDLE;

                    }

                }

                lastCrossing = counter;

            }

            lastValue = costasLoopOutput;

            counter += 1;

        }

        /* Wait for next sample */

        configSampleReady = false;

        /* Sleep until next interrupt occurs */

        AudioMoth_sleep();

    }

    return cancel == false;

}

/* Cancel audio configuration */

void AudioConfig_cancelAudioConfiguration() {

    cancel = true;

}
