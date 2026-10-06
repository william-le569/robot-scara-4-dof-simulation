# SCARA Robot Simulation & Kinematics Control (MATLAB)

https://www.youtube.com/watch?v=l_5V8tedO94

Chương trình mô phỏng động học (Kinematics) và quy hoạch quỹ đạo chuyển động cho **Robot SCARA 4 bậc tự do** (4-DOF SCARA Robot) sử dụng ngôn ngữ MATLAB và giao diện đồ họa người dùng (MATLAB GUI).

## 📋 Mục lục

1. [Giới thiệu tổng quan](#1-giới-thiệu-tổng-quan)

2. [Cấu trúc thư mục & Các tệp nguồn](#2-cấu-trúc-thư-mục--các-tệp-nguồn)

3. [Mô hình Động học Robot SCARA](#3-mô-hình-động-học-robot-scara)

4. [Hướng dẫn Cài đặt & Chạy chương trình](#4-hướng-dẫn-cài-đặt--chạy-chương-trình)

5. [Mô tả chi tiết các hàm chức năng](#5-mô-tả-chi-tiết-các-hàm-chức-năng)

## 1. Giới thiệu tổng quan

Dự án này cung cấp công cụ tính toán và trực quan hóa 3D cho dòng Robot SCARA gồm:

* **3 khớp quay** ($\theta_1, \theta_2, \theta_4$)

* **1 khớp trượt** ($d_3$)

### Chức năng chính:

* **Động học thuận (Forward Kinematics):** Tính toán vị trí/hướng điểm cuối (End-Effector) từ thông số các khớp.

* **Động học ngược (Inverse Kinematics):** Tính toán cấu hình khớp $(\theta_1, \theta_2, d_3, \theta_4)$ tương ứng với vị trí điểm cuối mong muốn.

* **Quy hoạch quỹ đạo (Trajectory Planning):** Chia khoảng thời gian ($t_1, t_2, t_3$) phục vụ tính toán quỹ đạo nội suy chuyển động mượt mà.

* **Mô phỏng Đồ họa 3D (3D Visualization):** Biểu diễn hình học không gian robot và hệ trục tọa độ khớp bằng trục mũi tên 3D (`arrow3d`).

## 2. Cấu trúc thư mục & Các tệp nguồn

```
├── HomoTransform.m    # Tính ma trận biến đổi đồng nhất DH (Denavit-Hartenberg)
├── InvKinematic.m     # Giải bài toán động học ngược cho Robot SCARA
├── calJointVar.m      # Tính toán vị trí tọa độ các gốc khớp trong không gian 3D
├── arrow3d.m          # Biểu diễn trực quan mũi tên 3D cho các trục tọa độ
├── getInit.m          # Đọc dữ liệu đầu vào từ giao diện MATLAB GUI
├── gettime.m          # Chia khung thời gian quy hoạch quỹ đạo chuyển động
└── workspace.mat      # Tệp lưu trữ biến môi trường làm việc MATLAB


```

## 3. Mô hình Động học Robot SCARA

### 3.1 Bảng tham số Denavit-Hartenberg (D-H)

Mô hình toán học của robot dựa trên phương pháp biểu diễn D-H tiêu chuẩn với các biến khớp $(\theta_1, \theta_2, d_3, \theta_4)$:

$$
T_i^{i-1} = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)
$$

Ma trận biến đổi tổng quát từ gốc đến điểm cuối:

$$
T_0^4 = T_0^1(\theta_1) \cdot T_1^2(\theta_2) \cdot T_2^3(d_3) \cdot T_3^4(\theta_4)
$$

### 3.2 Động học ngược (Inverse Kinematics)

Hàm `InvKinematic.m` tính toán nghịch đảo từ vị trí $(X, Y, Z)$ và góc hướng điểm cuối $\phi$:

* **Khớp trượt** $d_3$**:** Tính trực tiếp dựa trên cao độ $Z$.

* **Khớp quay** $\theta_1, \theta_2$**:** Giải hệ phương trình lượng giác mặt phẳng $(X, Y)$ sử dụng quy tắc hình học hoặc định lý cosin ($c_2 = \cos(\theta_2)$).

* **Khớp quay** $\theta_4$**:** Xác định dựa trên góc quay tổng thể $\phi = \theta_1 + \theta_2 - \theta_4$.

## 4. Hướng dẫn Cài đặt & Chạy chương trình

### Yêu cầu hệ thống:

* **Phần mềm:** MATLAB R2018b hoặc các phiên bản mới hơn.

* **Toolbox bắt buộc:**

  * *Symbolic Math Toolbox* (Nếu có thực hiện biến đổi biểu thức đại số).

  * *MATLAB Graphics / GUI Tools*.

### Các bước khởi chạy:

1. Mở phần mềm MATLAB.

2. Điều hướng thư mục làm việc (*Current Folder*) về thư mục chứa các tệp `.m` của dự án.

3. Nhập lệnh sau vào cửa sổ lệnh (*Command Window*) để nạp dữ liệu môi trường (nếu cần):

   ```
   load('workspace.mat')
   
   
   ```

4. Nếu có giao diện `.fig` đi kèm, mở bằng lệnh:

   ```
   guide
   
   
   ```

   Hoặc chạy trực tiếp tệp điều khiển GUI chính.

## 5. Mô tả chi tiết các hàm chức năng

### 🔹 `HomoTransform(theta, d, alpha, a)`

* **Đầu vào:** Tham số D-H của một khâu $(\theta, d, \alpha, a)$.

* **Đầu ra:** Ma trận $4 \times 4$ biểu diễn ma trận biến đổi đồng nhất $T$.

### 🔹 `InvKinematic(J, a1, a2, d1, d4)`

* **Đầu vào:** Vector điểm cuối $J = [X, Y, Z, \phi]$ và kích thước hình học các khâu ($a_1, a_2, d_1, d_4$).

* **Đầu ra:** Các giá trị khớp $[\theta_1, \theta_2, d_3, \theta_4]$.

### 🔹 `calJointVar(T)`

* **Đầu vào:** Chuỗi ma trận biến đổi đồng nhất $T$ của các khâu.

* **Đầu ra:** Tọa độ tâm các khớp trong không gian 3D dùng để vẽ khung xương robot.

### 🔹 `getInit(handles)`

* **Đầu vào:** Cấu trúc `handles` của MATLAB GUI.

* **Đầu ra:** Các giá trị thông số ban đầu được lấy từ các ô nhập liệu (`edittext`).

### 🔹 `gettime(t_total, ...)`

* **Đầu vào:** Tổng thời gian thực thi chuyển động.

* **Đầu ra:** Các phân đoạn thời gian $t_1, t_2, t_3$ cho quá trình gia tốc, vận tốc ổn định và giảm tốc.

### 🔹 `arrow3d(...)`

* **Chức năng:** Hàm bổ trợ đồ họa giúp vẽ mũi tên dạng bề mặt 3D mô phỏng hệ trục tọa độ tại từng vị trí khớp.

*Dự án phục vụ cho mục đích nghiên cứu, học tập môn Động lực học & Điều khiển Robot.*
