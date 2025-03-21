## BMS Monitoring & Control
  1. Download JK BMS on Playstore / Appstore.
  2. Connect BMS bluetooth with your smartphone
      - Application Password : 1234
      - Control BMS Password : 123456
    
## Setting PID Motor (ZLTech)
  1. Connect PC with Motor Driver* with USB to RS485 Communication
      - Motor Single (motor_center) : ZLAC8015
      - Motor Double (motor_left, motor_right) : ZLAC8015D

## Microcontroller Communication
  1. Connect PC with STM32 Microcontroller with USB to Ethernet Cable.
  2. Change your local ethernet with :<br/>
     ![image](https://github.com/user-attachments/assets/6373e2c8-56ed-4383-9084-6d2cfddca594)<br/>
     - IP : 192.168.50.254 (PC)
     - Subnet Mask : 255.255.255.0
     - Default Gateway : 8.8.8.8
     
  - UDP Ethernet Protocol
    - Port : 9798
    - IP 192.168.50.2 (STM32)
      
  3. Try to do "ping 192.168.50.2 -t" with Command Prompt to make sure that UDP Ethernet communication with STM32 is connected.
  4. Open VSCode to run the udp_io.cpp
      - Transmit Protocol<br/>
        ![image](https://github.com/user-attachments/assets/8608d970-2885-4933-8bb7-f82a90bfae67)<br/>
        - Kinematics Mode<br/>
            uint8_t PC_Mode = 1;<br/>
            int16_t PC_Vel[3] = {0, 0, 0}; (linear_x, linear_y, angular_z)<br/>
        - Manual Mode<br/>
            uint8_t PC_Mode = 2;<br/>
            int16_t PC_Vel[3] = {0, 0, 0}; (motor_left, motor_right, motor_center)<br/>
      - Receive Protocol<br/>
        ![image](https://github.com/user-attachments/assets/0e7f550e-c97a-43a0-b01d-7ed6ce70d637)<br/>
