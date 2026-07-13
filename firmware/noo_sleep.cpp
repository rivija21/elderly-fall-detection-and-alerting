#include <Arduino.h>
#include "driver/gpio.h"
#include <DFRobot_HumanDetection.h>
#include <LittleFS.h>
#include "AudioFileSourceLittleFS.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#include <WiFi.h>
#include <Wire.h> //tof
#include "SparkFun_VL53L1X.h"

// --- EDGE IMPULSE & NEOPIXEL INCLUDES ---
#define EIDSP_QUANTIZE_FILTERBANK   0
#include <KalharaHWP-project-1_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include <NeoPixelBus.h>

#/ Wake-up Pins
#include "driver/rtc_io.h"

#define PIN_PIR        GPIO_NUM_34 // Active HIGH
#define PIN_SIM_RING   GPIO_NUM_35 // Active LOW
// !! -- update the code: sleep/ wake up based on PIR sensor and SIM800L ring detection -- !!
// !! -- use SEN0623 MOSFET gate + otther EN pins and sleep signals for efficient power handling -- !!


// --- PIN DEFS ---
#define PIN_SIM_RING   GPIO_NUM_35 // Active LOW
#define PIN_SIM_EN     GPIO_NUM_21 
#define PIN_RADAR_EN   GPIO_NUM_18

#define SIM_RX_PIN     16 // ESP32 RX2 <- SIM800L TX
#define SIM_TX_PIN     17 // ESP32 TX2 -> SIM800L RX
#define RADAR_RX_PIN   26 // ESP32 RX <- SEN0623 TX
#define RADAR_TX_PIN   25 // ESP32 TX -> SEN0623 RX

// I2S Speaker Output (MAX98357A)
#define I2S_BCLK 5
#define I2S_LRC  2
#define I2S_DIN  33
#define SD_MODE  4

// NeoPixel Configuration
#define NUM_PIXELS 4 
#define DATA_PIN 19 

// --- GLOBAL VARIABLES ---
DFRobot_HumanDetection hu(&Serial1);
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod> strip(NUM_PIXELS, DATA_PIN);
SFEVL53L1X distanceSensor;

char phone1[15] = "";
char phone2[15] = ""; 
int height = 270;
char location[20] = "bathroom";
volatile bool smsPending = false; 

// --- EDGE IMPULSE GLOBALS ---
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;

// --- FUNCTION PROTOTYPES ---
void IRAM_ATTR simRingISR();
void initializesystem();
void handleBathroomOccupancy();
void handleIncomingSMS();
bool verifyFallWithAudio();
void sendSMS(String phoneNumber, String message);
void playAudioFile(const char* filepath);
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static void microphone_inference_end(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static int i2s_init_mic(uint32_t sampling_rate);
static int i2s_deinit_mic(void);
static void capture_samples(void* arg);
static void audio_inference_callback(uint32_t n_bytes);


// --- SETUP & MAIN LOOP ---
void IRAM_ATTR simRingISR() {
    smsPending = true;
}

void setup() {
    Serial.begin(115200);
    delay(100);

    WiFi.mode(WIFI_OFF);
    btStop();

    pinMode(PIN_SIM_RING, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_SIM_RING), simRingISR, FALLING);

    strip.Begin();
    strip.ClearTo(RgbColor(0, 0, 0)); 
    strip.SetPixelColor(0, RgbColor(255, 0, 0)); // 4th LED ON (White/Power)
    strip.Show();

    run_classifier_init(); 

    initializesystem();
}

void loop() {
    if (smsPending) {
        smsPending = false;
        handleIncomingSMS();
    }

    if (hu.smHumanData(hu.eHumanPresence) == 1) {
        handleBathroomOccupancy();
    }

    delay(100); 
}

// --- CORE HANDLERS ---
void initializesystem() {
    Wire.begin(); 
    Serial1.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    delay(100);

    // Initial ToF Measurement for Height
    if (distanceSensor.begin() != 0) {
    } else {
        distanceSensor.startRanging();
        while (!distanceSensor.checkForDataReady()) {
            delay(1);
        }
        int initialDistMm = distanceSensor.getDistance();
        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();
        
        height = initialDistMm / 10; // Convert mm to cm for SEN0623
    }

    hu.begin(); 
    delay(100);
    delay(1500); 

    while (hu.begin() != 0) {
        delay(1000);
    }

    while (hu.configWorkMode(hu.eFallingMode) != 0) {
        delay(1000);
    }
    delay(300);

    hu.configLEDLight(hu.eFALLLed, 0); delay(300);
    hu.configLEDLight(hu.eHPLed, 0); delay(300);
    hu.dmInstallHeight(height); delay(300); // Utilizes dynamically measured height
    hu.dmFallTime(5); delay(300);
    hu.dmUnmannedTime(5); delay(300);
    hu.dmFallConfig(hu.eResidenceTime, 200); delay(500);
    hu.dmFallConfig(hu.eFallSensitivityC, 3); delay(300);
    hu.sensorRet(); delay(500); 

    Serial2.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
    Serial.println("Waiting 1s for SIM network registration...");
    delay(1000); 

    Serial2.println("AT+GSMBUSY=1"); delay(500);
    Serial2.println("AT+CMGF=1"); delay(500);
    Serial2.println("AT+CMGD=1,4"); delay(2000);

    Serial2.println("AT+CREG?");
    long cregTimeout = millis() + 2000;
    String cregResponse = "";
    while (millis() < cregTimeout) {
        while (Serial2.available()) {
            cregResponse += (char)Serial2.read();
        }
    }
    if (cregResponse.indexOf(",1") != -1 || cregResponse.indexOf(",5") != -1) {
        strip.SetPixelColor(1, RgbColor(0, 255, 0)); // 3rd LED ON (Green/Cellular OK)
        strip.Show();
    }

    
    if (!LittleFS.begin()) {
    } else {
        if (LittleFS.exists("/setup.wav")) {
            playAudioFile("/setup.wav");
        }
    }
}

void handleBathroomOccupancy() {

    bool isOccupied = true;
    bool fallDetected = false;

    while (isOccupied) {
        int presence = hu.smHumanData(hu.eHumanPresence);
        int fallState = hu.getFallData(hu.eFallState);

        if (presence == 1) {
            strip.SetPixelColor(2, RgbColor(0, 255, 0));
        } else {
            strip.SetPixelColor(2, RgbColor(0, 0, 0));
        }
        strip.Show();

        if (fallState == 1) { 
            fallDetected = true;
            strip.SetPixelColor(3, RgbColor(0, 255, 0)); 
            strip.Show();
            break; 
        }

        if (presence == 0) {
            delay(2000); 
            if (hu.smHumanData(hu.eHumanPresence) == 0) {
                isOccupied = false; 
                fallDetected = false;
            } else {
                Serial.println("False negative mitigated. Occupant still present.");
            }
        }

        if (smsPending) {
            smsPending = false;
            handleIncomingSMS();
            delay(500); 
        }

        delay(1000); 
    }

    if (fallDetected) {
        playAudioFile("/message.wav");

        bool falsePositive = verifyFallWithAudio();
        
        if (!falsePositive) {
            String alertMsg = "ALERT: Fall detected in the " + String(location) + "!";
            
            if (strlen(phone1) > 4) sendSMS(String(phone1), alertMsg);
            if (strlen(phone2) > 4) sendSMS(String(phone2), alertMsg);
            
            if (strlen(phone1) < 5 && strlen(phone2) < 5) {
            }
            delay(5000); 
            
        } else { 
            playAudioFile("/cancel.wav");
        }
        strip.SetPixelColor(3, RgbColor(0, 0, 0)); 
        strip.SetPixelColor(2, RgbColor(0, 0, 0)); 
        strip.Show();
    }
}

bool verifyFallWithAudio() {
    
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        Serial.println("ERR: Could not allocate audio buffer. Defaulting to confirmed fall.");
        return false; 
    }

    bool cancellationCommandDetected = false;
    unsigned long verificationStartTime = millis();
    const unsigned long VERIFICATION_TIMEOUT = 10000; 

    while (millis() - verificationStartTime < VERIFICATION_TIMEOUT) {
        if (!microphone_inference_record()) continue;

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = {0};

        EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
            continue;
        }

        if (result.classification[0].value > 0.75) {
            cancellationCommandDetected = true;
            break;
        }
    }

    microphone_inference_end(); 
    return cancellationCommandDetected;
}

void playAudioFile(const char* filepath) {
    if (!LittleFS.exists(filepath)) {
        return;
    }

    pinMode(SD_MODE, OUTPUT);
    digitalWrite(SD_MODE, HIGH); 
    delay(100);

    AudioFileSourceLittleFS *file = new AudioFileSourceLittleFS(filepath); 
    AudioOutputI2S *out = new AudioOutputI2S();
    out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DIN);
    AudioGeneratorWAV *wav = new AudioGeneratorWAV();
    
    if (wav->begin(file, out)) {
        while (wav->isRunning()) {
            if (!wav->loop()) {
                wav->stop();
            }
        }
    } else {
        Serial.println("Failed to decode audio file.");
    }
    
    delete file;
    delete wav;
    delete out;
    
    digitalWrite(SD_MODE, LOW); 
}

void sendSMS(String phoneNumber, String message) {
    phoneNumber.trim(); 
    phoneNumber.replace("\"", ""); 

    while (Serial2.available()) { Serial2.read(); }

    Serial2.print("AT+CMGS=\"");
    Serial2.print(phoneNumber);
    Serial2.print("\"\r"); 
    
    long waitPrompt = millis();
    bool promptReceived = false;
    
    while (millis() - waitPrompt < 3000) {
        while (Serial2.available()) {
            char c = Serial2.read();
            if (c == '>') promptReceived = true;
        }
        if (promptReceived) break;
        delay(10);
    }
    
    if (promptReceived) {
        Serial2.print(message);
        Serial2.write(26); 
        Serial.println("Payload pushed. Waiting 6s...");
        delay(6000); 
    } else {
        Serial.println("Error: SIM timeout.");
        Serial2.write(27); 
        delay(1000);
    }
}

void handleIncomingSMS() {
    delay(1000); 
    Serial2.println("AT+CMGR=1");
    delay(500);
    
    String response = "";
    long timeout = millis() + 3000;
    while (millis() < timeout) {
        while (Serial2.available()) response += (char)Serial2.read();
    }

    String senderNumber = "";
    int cmgrIndex = response.indexOf("+CMGR:");
    if (cmgrIndex != -1) {
        int firstQuote = response.indexOf("\"", cmgrIndex);
        int secondQuote = response.indexOf("\"", firstQuote + 1);
        int thirdQuote = response.indexOf("\"", secondQuote + 1);
        int fourthQuote = response.indexOf("\"", thirdQuote + 1);
        
        if (thirdQuote != -1 && fourthQuote != -1) {
            senderNumber = response.substring(thirdQuote + 1, fourthQuote);
        }
    }

    response.toLowerCase(); 
    bool commandFound = false;
    String replyMessage = ""; 

    // Must evaluate "height=?" prior to "height=" to prevent premature substring matching.
    if (response.indexOf("height=?") != -1) {
        distanceSensor.startRanging(); 
        unsigned long t = millis();
        while (!distanceSensor.checkForDataReady()) {
        if (millis() - t > 1000) {
            replyMessage = "Error: ToF sensor timeout.";
             commandFound = true; 
        } else {
            int currentDistMm = distanceSensor.getDistance(); 
            int currentHeightCm = currentDistMm / 10;
            replyMessage = "Current ToF measured height: " + String(currentHeightCm) + "cm. Active system height: " + String(height) + "cm.";
            commandFound = true;
        }
        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();
    }   
        int currentDistMm = distanceSensor.getDistance(); 
        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();
        
        int currentHeightCm = currentDistMm / 10;
        replyMessage = "Current ToF measured height: " + String(currentHeightCm) + "cm. Active system height: " + String(height) + "cm.";
        commandFound = true;
    }
    else if (response.indexOf("height=") != -1) { 
        int valIndex = response.indexOf("height=") + 7;
        int endIndex = response.indexOf('\r', valIndex);
        if (endIndex == -1) endIndex = response.indexOf('\n', valIndex);
        if (endIndex == -1) endIndex = response.length();
        
        height = response.substring(valIndex, endIndex).toInt();
        hu.dmInstallHeight(height); // Pushes the new manual height logic to the mmWave sensor.
        
        replyMessage = "Success: Height manually set to " + String(height) + "cm";
        commandFound = true;
    } 
    else if (response.indexOf("phone1=") != -1) {
        int valIndex = response.indexOf("phone1=") + 7;
        int endIndex = response.indexOf('\r', valIndex);
        if (endIndex == -1) endIndex = response.indexOf('\n', valIndex);
        if (endIndex == -1) endIndex = response.length();
        
        String newPhone = response.substring(valIndex, endIndex);
        newPhone.trim(); newPhone.replace("\"", ""); 
        strncpy(phone1, newPhone.c_str(), sizeof(phone1) - 1);
        phone1[sizeof(phone1) - 1] = '\0'; 
        replyMessage = "Success: Phone1 set to " + String(phone1);
        commandFound = true;
    }
    else if (response.indexOf("phone2=") != -1) {
        int valIndex = response.indexOf("phone2=") + 7;
        int endIndex = response.indexOf('\r', valIndex);
        if (endIndex == -1) endIndex = response.indexOf('\n', valIndex);
        if (endIndex == -1) endIndex = response.length();
        
        String newPhone = response.substring(valIndex, endIndex);
        newPhone.trim(); newPhone.replace("\"", ""); 
        strncpy(phone2, newPhone.c_str(), sizeof(phone2) - 1);
        phone2[sizeof(phone2) - 1] = '\0'; 
        replyMessage = "Success: Phone2 set to " + String(phone2);
        commandFound = true;
    }
    else if (response.indexOf("location=") != -1) {
        int valIndex = response.indexOf("location=") + 9;
        int endIndex = response.indexOf('\r', valIndex);
        if (endIndex == -1) endIndex = response.indexOf('\n', valIndex);
        if (endIndex == -1) endIndex = response.length();
        
        String newLoc = response.substring(valIndex, endIndex);
        newLoc.trim(); newLoc.replace("\"", ""); 
        strncpy(location, newLoc.c_str(), sizeof(location) - 1);
        location[sizeof(location) - 1] = '\0'; 
        replyMessage = "Success: Location set to " + String(location);
        commandFound = true;
    }
    else if (response.indexOf("help") != -1) {
        replyMessage = "CMDs: height=?, height=, phone1=, phone2=, location=, help";
        commandFound = true;
    }
    
    delay(1000);
    if (commandFound) {
        sendSMS(senderNumber, replyMessage);
    } else if (!commandFound && response.length() > 0 && cmgrIndex != -1) {
        sendSMS(senderNumber, "Error: CMD NOT FOUND!");
    }

    while (Serial2.available()) { Serial2.read(); }
    Serial2.println("AT+CMGD=1,4"); 
    delay(3000); 
}

// --- EDGE IMPULSE I2S MICROPHONE HANDLERS ---
static void audio_inference_callback(uint32_t n_bytes) {
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    size_t bytes_read = i2s_bytes_to_read;
    while (record_status) {
        i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
        if (bytes_read > 0) {
            for (int x = 0; x < i2s_bytes_to_read/2; x++) {
                sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
            }
            if (record_status) {
                audio_inference_callback(i2s_bytes_to_read);
            } else {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[0] == NULL) return false;

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    if (i2s_init_mic(EI_CLASSIFIER_FREQUENCY)) {
        Serial.println("Failed to start I2S!");
    }
    ei_sleep(100);
    record_status = true;
    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

static bool microphone_inference_record(void) {
    bool ret = true;
    if (inference.buf_ready == 1) {
        ret = false;
    }
    while (inference.buf_ready == 0) {
        delay(1);
    }
    inference.buf_ready = 0;
    return ret;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void) {
    record_status = false;
    delay(100); 
    i2s_deinit_mic();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
}

static int i2s_init_mic(uint32_t sampling_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = EI_CLASSIFIER_FREQUENCY,
        .bits_per_sample = (i2s_bits_per_sample_t)16,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = -1,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 14,
        .ws_io_num = 27,
        .data_out_num = -1,
        .data_in_num = 32,
    };
    esp_err_t ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
    ret = i2s_set_pin((i2s_port_t)1, &pin_config);
    ret = i2s_zero_dma_buffer((i2s_port_t)1);
    return int(ret);
}

static int i2s_deinit_mic(void) {
    i2s_driver_uninstall((i2s_port_t)1); 
    return 0;
}