
# VERY SLOW SOFT UART EXAMPLE FOR ESP32

Very Very simple Soft serial for test purposes with very slow from 1baud up to 100baud
You can type at 115200bps will output at from 1bps upto 100bps, you can define


 //SOFT UART COMMUNICATION - ONLY TX 
 
 #define TX_BAUD_RATE 10       //from 1 up to 100 bps
 
 #define TX_PIN GPIO_NUM_15    // CHOOSE THE PIN 
 
 #define PARITY_ON OFF         //ODD PARITY ON|OFF   
 
 #define LEVEL_HIGH OFF        // ON|OFF SERIAL SIGNAL INVERSION - HERE YOU CAN INVERT THE SERIAL LOGIC 1 TO 0 



## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
