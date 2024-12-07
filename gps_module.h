#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <string>
#include <mutex>
#include <atomic>

// GPSデータ構造体
struct GPSData {
    uint64_t time_usec;
    uint8_t fix_type;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    bool location_valid;  // is location data valid?
};

// 関数プロトタイプ
bool verifyChecksum(const std::string& sentence);
GPSData parseNMEA(const std::string& sentence);
void printGPSData(const GPSData& data);
int setupSerialPort(const char* port_name, int baudrate);
void start_gps_thread(const char* port_name, int baudrate);
void stop_gps_thread();

// GPSデータ共有用
extern std::atomic<bool> gps_thread_running;
extern std::mutex gps_data_mutex;
extern GPSData current_gps_data;
extern bool debug_mode;

#endif // GPS_MODULE_H
