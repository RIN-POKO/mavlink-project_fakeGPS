/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 */

#include <csignal>
#include "mavlink_control.h"
#include "gps_module.h" // GPSモジュールのインクルード

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(int argc, char** argv) {
    // コマンドライン引数のデフォルト値
    char* uart_name = (char*)"/dev/ttyUSB0";
    int baudrate = 57600;
    char* gps_port_name = (char*)"/dev/ttyUSB1"; // GPS用ポート
    int gps_baudrate = B38400;
    bool use_udp = false;
    char* udp_ip = (char*)"127.0.0.1";
    int udp_port = 14540;
    bool autotakeoff = false;

    // コマンドライン引数の解析
    parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff, gps_port_name, gps_baudrate);

    // ポートとスレッドの設定
    Generic_Port* port;
    if (use_udp) {
        port = new UDP_Port(udp_ip, udp_port);
    } else {
        port = new Serial_Port(uart_name, baudrate);
    }

    Autopilot_Interface autopilot_interface(port);

    // 割り込みシグナルの設定
    port_quit = port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT, quit_handler);

    // ポートとインターフェースの起動
    port->start();
    autopilot_interface.start();

    // GPSモジュールの起動
    start_gps_thread(gps_port_name, gps_baudrate);

    // コマンドの実行
    commands(autopilot_interface, autotakeoff);

    // スレッドとポートの終了
    stop_gps_thread();
    autopilot_interface.stop();
    port->stop();

    delete port;
    return 0;
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------
void commands(Autopilot_Interface& api, bool autotakeoff) {
    printf("Starting commands...\n");

    if (autotakeoff) {
        api.arm_disarm(true);
        usleep(100000); // 100ms
    }

    while (true) {
        {
            // GPSデータを取得して処理
            std::lock_guard<std::mutex> lock(gps_data_mutex);
            if (current_gps_data.location_valid) {
                printf("Sending GPS data via MAVLink...\n");
                api.send_input_gps_message(current_gps_data.time_usec);
            } else {
                printf("Waiting for valid GPS data...\n");
            }
        }
        usleep(50000); // 50ms
    }
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
void parse_commandline(int argc, char** argv, char*& uart_name, int& baudrate,
                       bool& use_udp, char*& udp_ip, int& udp_port,
                       bool& autotakeoff, char*& gps_port_name, int& gps_baudrate) {
    const char* usage = "usage: mavlink_control [-d <device> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a] [--gps-port <port> --gps-baud <baudrate>]";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[++i];
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[++i]);
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
            if (argc > i + 1) {
                udp_ip = argv[++i];
                use_udp = true;
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
            if (argc > i + 1) {
                udp_port = atoi(argv[++i]);
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
            autotakeoff = true;
        } else if (strcmp(argv[i], "--gps-port") == 0) {
            if (argc > i + 1) {
                gps_port_name = argv[++i];
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "--gps-baud") == 0) {
            if (argc > i + 1) {
                gps_baudrate = atoi(argv[++i]);
            } else {
                printf("%s\n", usage);
                throw EXIT_FAILURE;
            }
        } else {
            printf("%s\n", usage);
            throw EXIT_FAILURE;
        }
    }
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
void quit_handler(int sig) {
    printf("\nTerminating at user request\n");

    try {
        autopilot_interface_quit->handle_quit(sig);
    } catch (...) {}

    try {
        port_quit->stop();
    } catch (...) {}

    stop_gps_thread();

    exit(0);
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char** argv) {
    try {
        return top(argc, argv);
    } catch (int error) {
        fprintf(stderr, "mavlink_control threw exception %i\n", error);
        return error;
    }
}
