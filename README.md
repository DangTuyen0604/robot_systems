NGHIÊN CỨU, THIẾT KẾ VÀ CHẾ TẠO ROBOT DI ĐỘNG NHẬN DIỆN VÀ ĐIỀU HƯỚNG THEO BIỂN BÁO GIAO THÔNG
Tổng quan về hệ thống
    Hệ thống được chia thành 2 workspace chính:
    1.Laptop_ws(chạy trên máy tính cá nhân)
        DATN chứa URDF mô hình robot, file launch mô phỏng trên Gazebo và môi trường mô phỏng datn_track_world

        perception_pkg:  chứa 2 node là nhận diện làn đường lane_detection_node, nhận diện biển báo traffic_sign_node. Dữ liệu sau khi đi qua 2 mô hình AI sẽ được truyền xuống decision_pkg thông qua topic

        decision_node:  có nhiệm vụ là ra quyết định dựa vào các dữ liệu đầu vào là dữ liệu sau khi đi qua 2 mô hình AI của 2 node là: lane_detection_node, traffic_sign_node.

        interfaces_pkg: khai báo các custom message/service
        bringup_pkg: khởi chạy hệ thống
    2.Raspberry_ws(chạy trên máy tính nhúng raspberry pi4)
        camera_pkg: Node đọc dữ liệu từ camera 
        control_pkg: Node nhận lệnh điều khiển và phát xung PWM ra động cơ thực tế
        interfaces_pkg: Đồng bộ định dạng message giao tiếp với Laptop.
Yêu cầu môi trường: 
    Hệ điều hành: Ubuntu 22.04 LTS
    Môi trường ROS: ROS2 humble
Hướng dẫn chạy: 
    Trên laptop: 
        Truy cập vào thư mục: cd robot_system/laptop_ws
        Source môi trường: source install/setup.bash
        Khởi chạy mô hình: ros2 launch bringup_pkg system_launch.py
    Trên raspberry:
        Truy cập vào thư mục: cd robot_system/raspberry_ws
        Source môi trường: source install/setup.bash
        Chạy node điều khiển động cơ: ros2 launch control_pkg motor_node
    chạy mô phỏng:
        Truy cập vào thư mục: cd robot_system/laptop_ws
        Source môi trường: source install/setup.bash
        Khởi chạy mô hình: ros2 launch bringup_pkg system_launch.py

