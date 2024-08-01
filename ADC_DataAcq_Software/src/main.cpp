#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <stddef.h>

#define SPI_DATAMODE SPI_MODE1
#define BUSY_PIN GPIO_NUM_9
#define CNV_PIN GPIO_NUM_10
#define SCLK_PIN GPIO_NUM_12
#define MISO_PIN GPIO_NUM_13
#define MOSI_PIN GPIO_NUM_11
#define SS_PIN GPIO_NUM_18  
#define SPI_CLK 4000000
#define NUM_SAMPLES_MEM 20000

void spiCom(SPIClass *spi, uint16_t data, uint8_t mode);
int sendtoOctave(uint16_t fields[], int n_fields);

// Variables
uint16_t meas_samples[NUM_SAMPLES_MEM] = {0};
volatile uint8_t myISR_Flags = 0x00; // Interrupt flags
uint16_t ecgData = 0; // Pre-processed ECG data
uint16_t receivedValue = 0; // Data container for spiCom()
SPIClass *fspi = NULL; // Instance of SPI class object

int cnt_samples = 0; // To count number of samples
bool DAQ_CMD = false; // Flag to track if DAQ command is received

// Hardware timer definition
hw_timer_t *Timer0_Cfg = NULL; // Timer0 configuration

// ISR for Timer0
void IRAM_ATTR Timer0_ISR() {
    digitalWrite(CNV_PIN, HIGH);
    myISR_Flags |= 0x01; // Set Timer0 flag
    digitalWrite(CNV_PIN, LOW);
}

// ISR for Busy pin
void IRAM_ATTR BusyPin_ISR() {
    myISR_Flags |= 0x02; // Set Busy pin flag
}

// SPI communication function
void spiCom(SPIClass *spi, uint16_t data, uint8_t spi_mode) {
    spi->beginTransaction(SPISettings(SPI_CLK, MSBFIRST, spi_mode));
    digitalWrite(spi->pinSS(), LOW);
    receivedValue = spi->transfer16(data);
    digitalWrite(spi->pinSS(), HIGH);
    spi->endTransaction();
} 

void setup() {
    Serial.begin(115200);

    // Initialize SPI
    fspi = new SPIClass(FSPI);
    fspi->begin(SCLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    pinMode(fspi->pinSS(), OUTPUT);
    digitalWrite(fspi->pinSS(), HIGH);
    pinMode(CNV_PIN, OUTPUT);
    pinMode(BUSY_PIN, INPUT);

    // Attach ISR for the busy pin
    attachInterrupt(digitalPinToInterrupt(BUSY_PIN), BusyPin_ISR, FALLING);

    // Timer0 configuration
    Timer0_Cfg = timerBegin(0, 40, true); // Prescaler of 40 for 1us ticks
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true); // 1ms interval
    timerAlarmEnable(Timer0_Cfg);
    cnt_samples = 0;
}


void loop() {
    // Check for serial input for DAQ_CMD
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        if (input == "DAQ_CMD") {
            DAQ_CMD = true;
            cnt_samples = 0; // Reset sample counter
        }
    }

    // Continue only if DAQ_CMD is received
    if (!DAQ_CMD) return;

    // Marking (2)
    if (myISR_Flags & 0x01) { // Check if Timer0_ISR set the flag
        myISR_Flags &= ~0x01; // Clear Timer0 flag
        // Start ADC conversion
        digitalWrite(CNV_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(CNV_PIN, LOW);
    }

    if (myISR_Flags & 0x02) { // Check if BusyPin_ISR set the flag
        // Read ADC result
        spiCom(fspi, 0, SPI_DATAMODE);
        meas_samples[cnt_samples++] = receivedValue;
        myISR_Flags &= ~0x02; // Clear Busy pin flag

        if (cnt_samples >= NUM_SAMPLES_MEM) {
            cnt_samples = 0; // Reset sample counter
            // Disable interrupts
            noInterrupts();
            timerAlarmDisable(Timer0_Cfg);
            detachInterrupt(digitalPinToInterrupt(BUSY_PIN));
            // Send data to Octave or process it further
            sendtoOctave(meas_samples, NUM_SAMPLES_MEM);
            // Clear array fields
            memset(meas_samples, 0, sizeof(meas_samples));
            // Enable interrupts  
            timerAlarmEnable(Timer0_Cfg);
            attachInterrupt(digitalPinToInterrupt(BUSY_PIN), BusyPin_ISR, FALLING);
            interrupts();
        }
    }
}



int sendtoOctave(uint16_t fields[], int n_fields) {
    for (int i = 0; i < n_fields; i++) {
        float ecgData = fields[i] * (3.3 / 65536); // Scale the ECG data to voltage
        Serial.write((uint8_t*)&ecgData, sizeof(float));
    }
    return 0;
}
