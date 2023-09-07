// Copyright [year] <Copyright Owner>
#ifndef NMEA_READER_H_
#define NMEA_READER_H_


#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

// using namespace std;

class NMEA_Reader {
 private:
    bool status_;
    double Long_;
    double Lat_;
    double Alt_;
    int dataValidation_;
    double geoid_;  // meter
    int satNumber_;
    char Lat_dir_;
    char Long_dir_;
    double UTC_;
    // std::string NMEA_GPRMC_ ;
    std::string NMEA_GPGGA_;

 public:
    NMEA_Reader();
    ~NMEA_Reader();
    double getLat();
    double getLong();
    double getAlt();
    double getUTC();
    double getGeoid();
    char getLongDir();
    char getLatDir();
    int getDataValidation();
    int getSatNumber();
    bool getStatus();

    // bool setNMEA_GPRMC(char* str);
    // bool setNMEA_GPRMC(std::string str);
    bool setNMEA_GPGGA(char* str);
    bool setNMEA_GPGGA(std::string str);
    std::vector<std::string> NMEA_split(std::string l, std::string k);
};

NMEA_Reader::NMEA_Reader() {
    status_ = false;
    Alt_ = 0;
    Lat_ = 0;
    Long_ = 0;
    dataValidation_ = 0;
    geoid_ = 0;
    Lat_dir_ = 'N';
    Long_dir_ = 'E';
    UTC_ = 0;
    satNumber_ = 0;

    NMEA_GPGGA_ = "";
    // NMEA_GPRMC = "";
}

NMEA_Reader::~NMEA_Reader() {}

// bool NMEA_Reader::setNMEA_GPRMC(char* str) {
//     status_ = false;
//     //TODO:
//     return false;
// }

// bool NMEA_Reader::setNMEA_GPRMC(std::string str) {
//     status_ = false;
//     NMEA_GPRMC_ = str;
//     if (str.compare(0, 6, "$GPRMC") == 0)
//     {
//         // valid input
//     }
//     return false;
// }

bool NMEA_Reader::setNMEA_GPGGA(char * str) {
    std::string s = str;
    return setNMEA_GPGGA(s);
}

bool NMEA_Reader::setNMEA_GPGGA(std::string str) {
    status_ = false;
    NMEA_GPGGA_ = str;
    if (str.compare(0, 6, "$GPGGA") == 0) {
        // valid input
        double gps_lat, gps_long, gps_alt, gps_utc, geoid;
        char gps_lat_dir, gps_long_dir;  // N , E
        int validation, satNumber;
        std::vector<std::string> ggaData = NMEA_split(str , ",");
        try {
            gps_utc = std::stod(ggaData.at(1));
            gps_lat = std::stod(ggaData.at(2));
            gps_lat_dir = (ggaData.at(3)).at(0);
            gps_long = std::stod(ggaData.at(4));
            gps_long_dir = (ggaData.at(5)).at(0);
            validation = static_cast<int>(std::stod(ggaData.at(6)));
            satNumber = static_cast<int>(std::stod(ggaData.at(7)));
            gps_alt = std::stod(ggaData.at(9));
            geoid = std::stod(ggaData.at(11));
            status_ = true;
        } catch(const std::exception& e) {
            gps_utc = 0;
            gps_lat = 0;
            gps_lat_dir = 'N';
            gps_long = 0;
            gps_long_dir = 'E';
            validation = 0;
            satNumber = 0;
            gps_alt = 0;
            geoid = 0;
            status_ = false;
        }
        Alt_ = gps_alt;
        Lat_ = gps_lat;
        Long_ = gps_long;
        dataValidation_ = validation;
        geoid_ = geoid;
        Lat_dir_ = gps_lat_dir;
        Long_dir_ = gps_long_dir;
        UTC_ = gps_utc;
        satNumber_ = satNumber;
        return true;
    }
    return false;
}

double NMEA_Reader::getLat() {
    return Lat_;
}

double NMEA_Reader::getLong() {
    return Long_;
}

double NMEA_Reader::getAlt() {
    return Alt_;
}

double NMEA_Reader::getGeoid() {
    return geoid_;
}

char NMEA_Reader::getLongDir() {
    return Long_dir_;
}

char NMEA_Reader::getLatDir() {
    return Lat_dir_;
}

int NMEA_Reader::getSatNumber() {
    return satNumber_;
}

int NMEA_Reader::getDataValidation() {
    return dataValidation_;
}
double NMEA_Reader::getUTC() {
    return UTC_;
}
bool NMEA_Reader::getStatus() {
    return status_;
}

// for string delimiter
std::vector<std::string> NMEA_Reader::NMEA_split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.emplace_back(s.substr(pos_start));
    return res;
}

#endif  // NMEA_READER_H_
