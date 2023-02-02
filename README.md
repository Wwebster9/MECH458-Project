# MECH458-Project
Uvic Mechatronics (MECH458) Project 

The project was to program a sorting algorithm to be deployed on a small sorting apparatus. The sorting apparatus consists of a conveyor belt and several sensors. The goal of this project was to sort items of various materials and colours into the appropriate bucket with the greatest possible accuracy. During a final demonstration, the sorting system was given four items of material unknown to the sorting algorithm. These items were identified, sorted into the correct buckets, and recorded by the system.

System Description:
The system could be paused with a pushbutton, which causes the DC motor to stop, thereby stopping the conveyor belt. Any sorted material information will then be displayed on the LCD. The sorted material information consists of the number of items sorted into each bucket and the number of partials. The partials are the components that have been classified by the sorting algorithm, but have not yet been sorted into a bucket. The system resumes when the pushbutton is pressed again.
The system also includes a ramp down function that is initiated when a second pushbutton is pressed. Thereafter, the system continues sorting objects placed on the conveyor belt for a duration of time and then proceeds to power down. Afterwards, all sorted materials are displayed on the LCD screen. The system is now in a safe mode requiring the microcontroller unit (MCU) to be reset to proceed with additional sorting.
To complete the project, various sensors and mechanical components have been utilized. The development of the project was completed on a prototype board and an MCU before progressing to the full test apparatus for validation.


![image](https://user-images.githubusercontent.com/66701943/216217188-e3bf9bdb-c218-4e3b-9d1e-6e0ee6f78611.png)

![image](https://user-images.githubusercontent.com/66701943/216217336-4e3322df-e2dd-4f10-acf7-c37b58a0b200.png)

