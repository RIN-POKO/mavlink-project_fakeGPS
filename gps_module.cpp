#include "gps_module.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <ctime>
#include <cmath>
#include <vector>
#include <thread>

std::atomic<bool> gps_thread_running(false);
std::mutex gps_data_mutex;
GPSData current_gps_data;
bool debug_mode = false;

// チェックサム検証
bool verifyChecksum(const std::string& sentence) {
    size_t starPos = sentence.find('*');
    if (starPos == std::string::npos) return false;

    unsigned char checksum = 0;
    for (size_t i = 1; i < starPos; ++i) {
        checksum ^= sentence[i];
    }

    unsigned int providedChecksum;
    std::stringstream ss;
    ss << std::hex << sentence.substr(starPos + 1);
    ss >> providedChecksum;

    return checksum == providedChecksum;
}

// NMEA文解析
GPSData parseNMEA(const std::string& sentence) {
    GPSData gps_data = {};
    gps_data.time_usec = std::time(nullptr) * 1e6;
    gps_data.location_valid = false;

    if (sentence.rfind("$GNGGA", 0) == 0) {
        std::istringstream stream(sentence);
        std::string token;
        std::vector<std::string> fields;
        while (std::getline(stream, token, ',')) {
            fields.push_back(token);
        }

        if (fields.size() >= 2 && fields[6] != "0") {
            gps_data.location_valid = true;
        }
        if (fields.size() >= 6 && !fields[2].empty() && !fields[4].empty()) {
            gps_data.time_usec = std::stod(fields[1]) * 1e6;
            gps_data.fix_type = fields[6] == "0" ? 0 : (fields[6] == "1" ? 2 : 3);
            gps_data.lat = static_cast<int32_t>(std::stod(fields[2]) * 1e7);
            gps_data.lon = static_cast<int32_t>(std::stod(fields[4]) * 1e7);
            gps_data.alt = static_cast<int32_t>(std::stod(fields[9]) * 1000);
        }
    }

    return gps_data;
}

// GPSデータ表示
void printGPSData(const GPSData& data) {
    if (!data.location_valid) {
        std::cout << "[Waiting for location data...]\n";
        return;
    }

    std::cout << "UTC Time(usec): " << data.time_usec << "\n"
              << "Fix type: " << static_cast<int>(data.fix_type) << "\n"
              << "Latitude: " << data.lat << "\n"
              << "Longitude: " << data.lon << "\n"
              << "Altitude: " << data.alt << "\n"
              << "--------------------------\n";
}

// シリアルポート設定
int setupSerialPort(const char* port_name, int baudrate) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port: " << port_name << "\n";
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// GPS受信スレッド
void gps_receive_thread(int fd) {
    char buffer[1024];
    std::string dataBuffer;

    while (gps_thread_running) {
        int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            dataBuffer += buffer;

            size_t pos;
            while ((pos = dataBuffer.find('\n')) != std::string::npos) {
                std::string sentence = dataBuffer.substr(0, pos);
                dataBuffer.erase(0, pos + 1);

                if (sentence[0] == '$' && verifyChecksum(sentence)) {
                    GPSData gps_data = parseNMEA(sentence);
                    std::lock_guard<std::mutex> lock(gps_data_mutex);
                    current_gps_data = gps_data;

                    if (debug_mode) {
                        printGPSData(gps_data);
                    }
                }
            }
        }
        usleep(10000); // 10ms
    }
}

// GPSスレッド開始
void start_gps_thread(const char* port_name, int baudrate) {
    int fd = setupSerialPort(port_name, baudrate);
    if (fd == -1) {
        return;
    }

    gps_thread_running = true;
    std::thread(gps_receive_thread, fd).detach();
}

// GPSスレッド停止
void stop_gps_thread() {
    gps_thread_running = false;
}