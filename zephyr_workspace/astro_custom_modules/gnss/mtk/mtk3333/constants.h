
#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

// used by check() for validity tests, room for future expansion
///< valid source ids
const char* sources[7] = {"II", "WI", "GP", "PG", "GN", "P", "ZZZ"};
#ifdef NMEA_EXTENSIONS
///< parseable sentence ids
const char* sentences_parsed[21] = {"GGA", "GLL", "GSA", "RMC", "DBT", "HDM", "HDT", "MDA", "MTW", "MWV",
                                    "RMB", "TOP", "TXT", "VHW", "VLW", "VPW", "VWR", "WCV", "XTE", "ZZZ"};
///< known, but not parseable
const char* sentences_known[15] = {"APB", "DPT", "GSV", "HDG", "MWD", "ROT", "RPM", "RSA", "VDR", "VTG", "ZDA", "ZZZ"};
#else  // make the lists short to save memory
///< parseable sentence ids
const char* sentences_parsed[6] = {"GGA", "GLL", "GSA", "RMC", "TOP", "ZZZ"};
///< known, but not parseable
const char* sentences_known[4] = {"DBT", "HDM", "HDT", "ZZZ"};
#endif

#endif