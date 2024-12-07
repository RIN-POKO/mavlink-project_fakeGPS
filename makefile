# ターゲット定義
all: git_submodule mavlink_control

# メインターゲット
mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp gps_module.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 \
	    mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp gps_module.cpp \
	    -o mavlink_control -lpthread

# サブモジュールの初期化
git_submodule:
	git submodule update --init --recursive

# クリーンアップターゲット
clean:
	rm -rf *o mavlink_control
