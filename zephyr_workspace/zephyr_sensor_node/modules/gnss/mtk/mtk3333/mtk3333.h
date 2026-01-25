#ifndef ZEPHYR_DRIVERS_GNSS_MTK3333_H_
#define ZEPHYR_DRIVERS_GNSS_MTK3333_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif
#define GPS_MAX_I2C_TRANSFER 32  ///< The max number of bytes we'll try to read at once
#define MAXLINELENGTH 120        ///< how long are max NMEA lines to parse?
#define NMEA_MAX_SENTENCE_ID 20  ///< maximum length of a sentence ID name, including terminating 0
#define NMEA_MAX_SOURCE_ID 3     ///< maximum length of a source ID name, including terminating 0

struct mtk3333_data {
    struct k_work_delayable poll_work;
    const struct device* dev;

    int8_t read_buffer_index;

    char i2c_buffer[GPS_MAX_I2C_TRANSFER];
    int8_t buff_max;
    char last_char;
    volatile char buffer_line1[MAXLINELENGTH];
    volatile char buffer_line2[MAXLINELENGTH];
    volatile uint8_t line_index;
    volatile char* current_line;
    volatile char* last_line;
    volatile bool rcvd_flag;

    uint32_t last_update;
    uint32_t last_fix;
    uint32_t last_time;
    uint32_t last_date;
    uint32_t rcvd_time;
    uint32_t sent_time;
    int thisCheck;                            ///< the results of the check on the current sentence
    char thisSource[NMEA_MAX_SOURCE_ID];      ///< the first two letters of the current sentence, e.g. WI, GP
    char thisSentence[NMEA_MAX_SENTENCE_ID];  ///< the next three letters of the current sentence, e.g. GLL, RMC
    char lastSource[NMEA_MAX_SOURCE_ID];      ///< the results of the check on the most recent successfully
                                              ///< parsed sentence
    char lastSentence[NMEA_MAX_SENTENCE_ID];  ///< the next three letters of the most recent successfully parsed
                                              ///< sentence, e.g. GLL, RMC

    /* Simulated GNSS data */
    struct gnss_data data;
    int fix_rate_ms;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_GNSS_MTK3333_H_ */
