Introduction
This project demonstrates a smart car system controlled via a mobile application using Bluetooth communication. The car integrates an automatic braking system based on an ultrasonic sensor for obstacle detection. The project aims to enhance safety by stopping the car when objects are detected within a predefined distance, simulating real-world automotive safety systems.

 

________________________________________






Objective
1.	To develop a smart vehicle system that incorporates an automatic braking mechanism for collision prevention.
2.	To implement a Bluetooth-based mobile application for wireless control of the car’s speed, direction, lights, and horn.
3.	To demonstrate real-time obstacle detection using an ultrasonic sensor.
4.	To showcase embedded systems technology for controlling motor speed and direction using Pulse Width Modulation (PWM).
5.	To simulate foundational concepts in autonomous vehicle technology.
________________________________________
Components and Circuit Diagram
Components Used
1.	ESP32 Microcontroller: Provides Bluetooth functionality and motor control.
2.	HC-SR04 Ultrasonic Sensor: Measures distance for automatic braking.
3.	L298N Motor Driver: Controls the direction and speed of the motors.
4.	Motors: Used for forward and backward movement.
5.	Bluetooth Module (integrated in ESP32): Communicates with the mobile application.
6.	Buzzer: Provides sound output for the horn.
7.	LEDs: Used as front and backlights for signaling.
8.	Battery: Powers the car.
9.	Wires and Connectors: For connecting components.









Circuit Diagram
In the circuit:
•	The motor driver is connected to the ESP32 pins for PWM control.
•	The ultrasonic sensor’s trig and echo pins are connected to digital pins for distance measurement.
•	The buzzer and LEDs are connected to output pins for horn and light control.



 
________________________________________






Code Explanation
The code is written in C++ for the Arduino IDE and utilizes the BluetoothSerial library for Bluetooth communication.
Key Components of the Code
1.	Bluetooth Initialization
2.	BluetoothSerial serialBT;
serialBT.begin("Mahmoud Tarek");
Initializes Bluetooth communication with the name "Mahmoud Tarek".
3.	PWM Setup for Motor Speed Control
4.	ledcSetup(R, 5000, 8);
ledcAttachPin(enA, R);
Configures PWM for motor speed control on specific pins.
5.	Distance Measurement Function
6.	long getDistance() {
7.	    digitalWrite(trigPin, LOW);
8.	    delayMicroseconds(2);
9.	    digitalWrite(trigPin, HIGH);
10.	    delayMicroseconds(10);
11.	    digitalWrite(trigPin, LOW);
12.	    long duration = pulseIn(echoPin, HIGH);
13.	    return duration * 0.034 / 2;
}
This function measures the distance to an obstacle using the ultrasonic sensor.






14.	Bluetooth Command Processing
15.	while (serialBT.available()) {
16.	    btSignal = serialBT.read();
17.	    if (btSignal == 'F') forward();
18.	    else if (btSignal == 'B') backward();
19.	    // Additional cases for left, right, lights, and horn
}
Commands from the mobile app are processed to control the car’s movement and other features.
20.	Automatic Braking
21.	if (distance < distanceThreshold && btSignal == 'F') {
22.	    stop();
}
Stops the car if an obstacle is detected within the threshold.
23.	Light and Horn Control
24.	case 'W':
25.	    digitalWrite(frontLight1, HIGH);
26.	    digitalWrite(frontLight2, HIGH);
27.	    break;
28.	case 'V':
29.	    hornOn();
    break;
Controls the front lights and horn.
________________________________________

Flow chart and block diagram  

 












Results and Observations
•	The car successfully stops when obstacles are detected within the set distance.
•	Bluetooth communication allows smooth control of movement, speed, lights, and horn.
•	The system effectively integrates obstacle detection and motor control.
________________________________________
Conclusion
This project demonstrates a practical implementation of a Bluetooth-controlled car with an automatic braking system. The integration of distance measurement and real-time response enhances safety and simulates basic autonomous vehicle functionality. Future enhancements could include advanced sensors, AI-based decision-making, and improved control algorithms.
