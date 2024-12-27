/*
 *  MCE 202 Project: MIDI, Media and Volume Controller USB Device
 *  
 *  Abdullah Erdem Özbaykuş - 20220613033
 *  Ahmet Kıvanç Öztin - 20230613029
 *  Berkay Eren Konuk - 20230613026
 *  Deniz Emre Kapkap - 20230613024
 *  Ece Yörükoğlu - 20230613042
 *  Tolga Tuna Akpınar - 20230613005
 *  Zeynep Demirbaş - 20230613016
 *
 *  
 *  
 *  Last updated: 27/12/2024 18:41
 *
 *  Note: During the compilation process, HID-Project library prompts a pragma warning: "Using default ASCII layout for keyboard modules"
 *
 *  It is just an information message and can be avoided but it can be misleading because Arduino IDE shows it in compilation terminal in red color like error messages
 *  For more information: https://github.com/NicoHood/HID/wiki/Keyboard-API#keyboard-layouts
 *
 *  Note 2: Although the project code is huge enough for it that it needs to be seperated into some .h and .cpp files, the project is written on a single .ino file on purpose.
 *  The reason behind that is, using header files are not in the boundaries of our MCE 202 lecture syllabus and not everyone participated in this project has that much C++ experience and knowledge.
 */



/* 
 *  A brief information about the code structure
 *  We have divided the code into below parts
 *    -> Import libraries
 *    -> Define macros
 *    -> Define constants
 *    -> Define variables
 *    -> Function definitions
 *    -> setup & loop functions
 */



/*
 *  Import libraries
 *
 */
#include "FastLED.h"
#include "MIDIUSB.h"
#include "HID-Project.h"
#include "HID-Settings.h"



/*
 *  Define macros
 *
 */
// These macros are for the device to control its mode by a switch
#define MODE_MEDIA 0
#define MODE_MIDI 1

// These are the boundaries for the encoder counter
#define ENCODER_MAX 127
#define ENCODER_MIN 0

// These macros define encoder direction
#define ENCODER_CLOCKWISE true
#define ENCODER_COUNTERCLOCKWISE false

// These are the boundaries for the encoder increment
#define INCREMENT_MAX 5
#define INCREMENT_MIN 1

// These are the boundaries for the LED brightness
#define LED_MAX 200 // Normally 255 but we prefer 220 for safety
#define LED_MIN 0

// These interval macro is for the interval of animation timer
#define LED_INTERVAL 25

// These macros are for the encoder double click event
#define ENCODER_CLICK_DOUBLE 2
#define ENCODER_CLICK_SINGLE 1
#define ENCODER_CLICK_NONE 0
#define ENCODER_CLICK_DEBOUNCE_TIME 50 // Debounce time in milliseconds
#define ENCODER_CLICK_DOUBLE_INTERVAL 150 // Max time between clicks for a double-click

// Cooldown for brightness mode
#define BRIGHTNESS_MODE_COOLDOWN 500



/*
 *  Define constants
 *
 */
const int pinEncoderCLK = 2; // Must be an interrupt-capable pin
const int pinEncoderDT = 3;
const int pinEncoderSW = 4;

const int buttonCount = 6;
const int pinButtons[buttonCount] = {5, 6, 7, 8, 9, 14}; // The pins for the buttons

const int pinSwitch = 16;  

const int ledCount = 48;
const int pinLED = 10;
CRGB leds[ledCount];

// Animation count for LED
const int animationCount = 7;



/*
 *  Define variables
 *
 */
int DEVICE_MODE = MODE_MEDIA; // Initial mode: MEDIA

// Button states is used to keep track of if a button is pressed or not, to prevent continuous data spam
bool buttonStates[buttonCount] = {false, false, false, false, false, false};

int encoderIncrement = 3;

// The keywork 'volatile' informs the compiler that the values of these variables can be changed unexpectedly by external factors, in our case due to an interrupt
volatile int midiControlValue = 0; // Encoder position variable, kept between 0 and 127 inclusive
volatile int lastStateCLK; // Necessary to keep the last state of the encoder

// Generally, we should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // Stores the last update time for LED animation
int ledIndex = 0; // Current LED index for the animation

// To handle encoder double click, it will change the animation
unsigned long encoderLastPressTime = 0;
unsigned long encoderLastReleaseTime = 0;
unsigned long encoderCurrentMillis = 0;
bool encoderButtonState = HIGH;  // Assume button is not pressed initially
bool encoderLastButtonState = HIGH;
int encoderClickState = ENCODER_CLICK_NONE;

// Handle animation change
int animationState = 0;

// LED brightness
int brightnessLED = 125;

// To handle brightness mode state
bool isModeBrightness = false;



/*
 *  Function definitions
 *
 */
// Send data to press on a note
void noteOn(byte channel, byte pitch, byte velocity) {
  // First parameter is the event type (0x09 = note on, 0x08 = note off)
  // Second parameter is note-on/note-off, combined with the channel
  // Channel can be anything between 0-15. Typically reported to the user as 1-16
  // Third parameter is the note number (48 = middle C)
  // Fourth parameter is the velocity (64 = normal, 127 = fastest)
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

// Send data to cancel pressed note
void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

// Send data to change the value of specified controller on specified channel in a DAW (Digital Audio Workspace) program. (ex. FL Studio, Ableton Live, Bitwig Studio, Reaper, Cubase etc...)
void controlChange(byte channel, byte control, byte value) {
  // First parameter is the event type (0x0B = control change)
  // Second parameter is the event type, combined with the channel
  // Third parameter is the control number number (0-119)
  // Fourth parameter is the control value (0-127)
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

// Handle the state of switch component in order to determine device mode (MIDI or MEDIA)
void handleSwitch() {
  int valueSwitch = digitalRead(pinSwitch); 

  if (valueSwitch == HIGH) {  
    if(DEVICE_MODE == MODE_MEDIA) {
      Serial.println("Switch mode to MIDI");
      DEVICE_MODE = MODE_MIDI;
    }
  } else {  
    if(DEVICE_MODE == MODE_MIDI) {
      Serial.println("Switch mode to MEDIA");
      DEVICE_MODE = MODE_MEDIA;
    }
  }  
}

// Handle button press behaviour due to buttonIndex regarding to the buttons inside pinButtons array
void handleButtonPress(int buttonIndex) {
  if(DEVICE_MODE == MODE_MEDIA) {
    /* Let's define some behaviour here for buttons due to their array index
     *
     * For the case of MEDIA mode:
     * - The button at index 0 will be {MEDIA_PREVIOUS} key
     * - The button at index 1 will be {MEDIA_PLAY_PAUSE} key
     * - The button at index 2 will be {MEDIA_NEXT} key
     * - The button at index 3 will be {MEDIA_REWIND} key
     * - The button at index 4 will be {MEDIA_VOLUME_MUTE} key
     * - The button at index 5 will be {MEDIA_FAST_FORWARD} key
     *
     */

    switch(buttonIndex) {
      case 0: 
        Consumer.write(MEDIA_PREVIOUS);
        Serial.println("[MEDIA] Play previous media");
        break;
      case 1: 
        Consumer.write(MEDIA_PLAY_PAUSE);
        Serial.println("[MEDIA] Play/pause media");
        break;
      case 2: 
        Consumer.write(MEDIA_NEXT);
        Serial.println("[MEDIA] Play next media");
        break;
      case 3:
        Consumer.write(MEDIA_REWIND);
        Serial.println("[MEDIA] Rewind media");
        break;
      case 4: 
        Consumer.write(MEDIA_VOLUME_MUTE);
        Serial.println("[MEDIA] Mute sound");
        break;
      case 5: 
        Consumer.write(MEDIA_FAST_FORWARD);
        Serial.println("[MEDIA] Fast forward media");
        break;
    }
  } else /*MODE_MIDI*/ {
    /* Let's define some behaviour here for buttons due to their array index
     *
     * For the case of MIDI mode, the first button will be C note (48 = middle C) and the rest of the buttons will be the following buttons
     * So, we will sum their indexes with middle C value which is 48
     */
    int buttonNote = 48 + buttonIndex;
    noteOn(0, buttonNote, 64);   // Channel 0, middle C + index as offset, normal velocity (64 = normal, 127 = fastest)
    Serial.println("[MIDI] Send note ON: " + String(buttonNote));
  }
}

// Handle button release behaviour due to buttonIndex regarding to the buttons inside pinButtons array
void handleButtonRelease(int buttonIndex) {
  if(DEVICE_MODE == MODE_MEDIA) {
    // No need to do anything in button release scenerio for MEDIA mode
  } else /*MODE_MIDI*/ {
    int buttonNote = 48 + buttonIndex;
    noteOff(0, buttonNote, 64);   // Channel 0, middle C + index as offset, normal velocity (64 = normal, 127 = fastest)
    Serial.println("[MIDI] Send note OFF: " + String(buttonNote));
  }
}

// Handle buttons whose pins are defined in pinButtons array in "void setup()"
void handleButtons() {
  // We use a for loop to iterate through all button pins and write one button code which will work for all buttons, which is pretty efficient I might say
  for (int i = 0; i < buttonCount; i++) {
    if(digitalRead(pinButtons[i]) == LOW) { // digitalRead returns LOW if our buttons are pressed, becuase we used INPUT_PULLUP
      if(!buttonStates[i]) {
        buttonStates[i] = true;
        handleButtonPress(i);
      }
    } else {
      if(buttonStates[i]) {
        buttonStates[i] = false;
        handleButtonRelease(i);
      }
    }
  }
}

// Handle value increment operation
int handleIncrement(int val, int inc, int threshold, bool isIncreasing) {
  int newVal = val + inc;
  if(isIncreasing ? (newVal < threshold) : (newVal > threshold)) {
    return newVal;
  } else {
    return threshold;
  }
}

// Handle the case of encoder turning either clockwise or counterclockwise
void encoderRotation(bool isClockwise) {
  bool encoderButtonState = digitalRead(pinEncoderSW) == HIGH; // encoderButtonState returns LOW if encoder button is pressed, becuase we used INPUT_PULLUP
  /*  The main logic here is that: 
    *      In MEDIA mode,
    *        -> If encoder button is pressed while rotating it, change the brightness of LED
    *        -> If encoder button is not pressed while rotating it, change the volume of operating system
    *
    *      In MIDI mode,
    *        -> If encoder button is pressed while rotating it, change how much value it will increment
    *        -> If encoder button is not pressed while rotating it, change the value that will be sent to DAW (Digital Audio Workstation) through MIDIUSB interface
    *  
    *  The below code might look confusing but it's actually simple.
    *  We have created an increment handler function and we've used ternary operator (condition ? true : false) structure in its arguments in order to reduce the number of if block that will look messy.
    */
  if(DEVICE_MODE == MODE_MEDIA) {
    if (encoderButtonState) {
      Consumer.write(isClockwise ? MEDIA_VOLUME_UP : MEDIA_VOLUME_DOWN);
      Serial.println("[MEDIA] Turn volume " + String(isClockwise ? "up" : "down"));
    } else {
      brightnessLED = handleIncrement(
        brightnessLED, 
        isClockwise ? encoderIncrement : -encoderIncrement, 
        isClockwise ? LED_MAX : LED_MIN, 
        isClockwise
      );

      Serial.println("[MEDIA] Brightness: " + String(brightnessLED));
    }
  } else /*MODE_MIDI*/ {
    if (encoderButtonState) {
      // Change midiControlValue value to change MIDI control
      midiControlValue = handleIncrement(
        midiControlValue, 
        isClockwise ? encoderIncrement : -encoderIncrement, 
        isClockwise ? ENCODER_MAX : ENCODER_MIN, 
        isClockwise
      );
      Serial.println("[MIDI] Send controller: " + String(midiControlValue));
    } else {
      // Change knob sensitivity
      encoderIncrement = handleIncrement(
        encoderIncrement, 
        isClockwise ? 1 : -1, 
        isClockwise ? INCREMENT_MAX : INCREMENT_MIN, 
        isClockwise
      );
      Serial.println("[MIDI] Increment: " + String(encoderIncrement));
    }

    controlChange(0, 10, midiControlValue); // Send MIDI information to change 10th control on channel 0 to the value midiControlValue (0-127)
  }
}

// Encoder ISR (Interrupt Service Routine)
void updateEncoder() {
  // Read the current state of CLK
  int currentStateCLK = digitalRead(pinEncoderCLK);

  // If the state has changed
  if (currentStateCLK != lastStateCLK) {
    // Determine direction by comparing DT and CLK
    if (digitalRead(pinEncoderDT) != currentStateCLK) {
      encoderRotation(ENCODER_CLOCKWISE);
    } else {
      encoderRotation(ENCODER_COUNTERCLOCKWISE);
    }
  }

  // Save the last state of CLK
  lastStateCLK = currentStateCLK;
}

// Change the number of LEDs lighting up due to midiControlValue in color green in MIDI mode
void midiControlValueAnimationLED() {
  // Map the volume to the LED count
  int activeLEDs = map(midiControlValue, ENCODER_MIN, ENCODER_MAX, 0, ledCount);

  // Clear all LEDs
  fill_solid(leds, ledCount, CRGB::Black);

  // Light up LEDs to indicate the volume level
  for (int i = 0; i < activeLEDs; i++) {
    leds[i] = CRGB::Green;
  }

  FastLED.show();
}

// Change the number of LEDs lighting up due to encoderIncrement while changing the sensitivity of encoder knob in color red in MIDI mode
void incrementValueAnimationLED() {
  // Map the volume to the LED count
  int activeLEDs = map(encoderIncrement, 0, INCREMENT_MAX, 0, ledCount);

  // Clear all LEDs
  fill_solid(leds, ledCount, CRGB::Black);

  // Light up LEDs to indicate the volume level
  for (int i = 0; i < activeLEDs; i++) {
    leds[i] = CRGB::Red;
  }

  FastLED.show();
}

// Change the number of LEDs lighting up due to brightnessLED while changing the brightness of LEDs in color blue in MEDIA mode
void brightnessValueAnimationLED() {
  // Map the volume to the LED count
  int activeLEDs = map(brightnessLED, LED_MIN, LED_MAX, 0, ledCount);

  // Clear all LEDs
  fill_solid(leds, ledCount, CRGB::Black);

  // Light up LEDs to indicate the volume level
  for (int i = 0; i < activeLEDs; i++) {
    leds[i] = CRGB::Blue;
  }

  FastLED.show();
}

// Animation 0
void rainbowAnimationLED() {
    // Update the rainbow animation
    static uint8_t hue = 0; // Define it as static so it can remain its changed value between function calls
    fill_rainbow(leds, ledCount, hue, 7); // 7 is the delta hue

    FastLED.show();
    hue++; // Increment the base hue for the next frame
}

// Animation 1
void redAnimationLED() {
  fill_solid(leds, ledCount, CRGB::Red);

  // Show the LEDs
  FastLED.show();
}

// Animation 2
void greenAnimationLED() {
  fill_solid(leds, ledCount, CRGB::Green);

  // Show the LEDs
  FastLED.show();
}

// Animation 3
void cyanAnimationLED() {
  fill_solid(leds, ledCount, CRGB::Cyan);

  // Show the LEDs
  FastLED.show();
}

// Animation 4
void magentaAnimationLED() {
  fill_solid(leds, ledCount, CRGB::Magenta);

  // Show the LEDs
  FastLED.show();
}

// Animation 5
void slideAnimationLED() {
  static int position = 0; // Current position of the block
  static int direction = 1; // 1 = forward, -1 = backward
  int colorsCount = 5;
  static CRGB colors[] = {CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Cyan, CRGB::Magenta};
  static int colorIndex = 0;
  int blockSize = 5; // Number of LEDs in the sliding block

  // Clear all LEDs
  fill_solid(leds, ledCount, CRGB::Black);

  // Light up the sliding block
  for (int i = 0; i < blockSize; i++) {
    int index = (position + i) % ledCount; // Wrap around if it exceeds the strip length
    leds[index] = colors[colorIndex]; // Block color
  }

  // Move the block
  position += direction;

  // Reverse direction at the ends
  if (position == 0 || position + blockSize >= ledCount) {
    direction = -direction;
    colorIndex = (colorIndex + 1) % colorsCount;
  }

  FastLED.show();
}

// Animation 6
void closeAnimationLED() {
  // Clear all LEDs
  fill_solid(leds, ledCount, CRGB::Black);

  // Show the LEDs
  FastLED.show();
}

// Handle LED animations without using the delay function
void handleLED() {
  /* We will be avoiding using delay function since it will affect all processes running inside the loop
   * We want the animation run parallel to all the other commands, so we will be counting milliseconds to check if necessary amount of time is passed
   */ 
  unsigned long currentMillis = millis();

  bool encoderButtonState = digitalRead(pinEncoderSW) == HIGH; // encoderButtonState returns LOW if encoder button is pressed, becuase we used INPUT_PULLUP

  if (DEVICE_MODE == MODE_MEDIA) {
    if (encoderButtonState) {
      isModeBrightness = false; // Turn OFF brightness change mode
      if (currentMillis - previousMillis >= LED_INTERVAL) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // Update the animation
        switch(animationState) {
          case 0:
            rainbowAnimationLED();
            break;
          case 1:
            redAnimationLED();
            break;
          case 2:
            greenAnimationLED();
            break;
          case 3:
            cyanAnimationLED();
            break;
          case 4:
            magentaAnimationLED();
            break;
          case 5:
            slideAnimationLED();
            break;
          case 6:
            closeAnimationLED();
            break;
        } 
      }
    } else {
      // Cooldown to go brightness change state for a more pleasing experience
      if (currentMillis - previousMillis >= BRIGHTNESS_MODE_COOLDOWN) {
        isModeBrightness = true; // Turn ON brightness change mode
      }
      // If brightness change mode is ON
      if (isModeBrightness) {
        brightnessValueAnimationLED();

        // We are changing the brightnessLED variable inside updateEncoder (ISR) and we set brightness inside handleLED (right here),
        // because we don't want to handle any FastLED operations in an interrupt service routine since we handle all FastLED releated thing here together
        FastLED.setBrightness(brightnessLED);
      }
    }
  } else /*MODE_MIDI*/ {
    if (encoderButtonState) {
      midiControlValueAnimationLED();
    } else {
      incrementValueAnimationLED();
    }
  }
}

// Change animationState
void handleAnimationChange() {
  animationState = (animationState + 1) % animationCount; // animationState will go back to 0 once the divison of the animationState with animationCount results in 1, pretty clever
}

// We want to change the LED animation if encoder button is pressed twice, seems quite complicated though but we have to use millis in order to avoid using the delay function
void handleEncoderDoubleClick() {
  encoderCurrentMillis = millis();
  encoderButtonState = digitalRead(pinEncoderSW);

  // Detect button press (falling edge)
  if (encoderButtonState == LOW && encoderLastButtonState == HIGH && (encoderCurrentMillis - encoderLastReleaseTime > ENCODER_CLICK_DEBOUNCE_TIME)) {
    encoderLastPressTime = encoderCurrentMillis;
    
    // Check for a double-click
    if (encoderCurrentMillis - encoderLastReleaseTime < ENCODER_CLICK_DOUBLE_INTERVAL) {
      encoderClickState = ENCODER_CLICK_DOUBLE;
    } else {
      encoderClickState = ENCODER_CLICK_SINGLE;
    }
  }

  // Detect button release (rising edge)
  if (encoderButtonState == HIGH && encoderLastButtonState == LOW && (encoderCurrentMillis - encoderLastPressTime > ENCODER_CLICK_DEBOUNCE_TIME)) {
    encoderLastReleaseTime = encoderCurrentMillis;
  }

  // Handle click state
  if (encoderClickState == ENCODER_CLICK_SINGLE && (encoderCurrentMillis - encoderLastReleaseTime > ENCODER_CLICK_DOUBLE_INTERVAL)) {
    // No second press occurred within the interval; it's a single click
    encoderClickState = ENCODER_CLICK_NONE;
  } else if (encoderClickState == ENCODER_CLICK_DOUBLE) {
    // Double-click detected
    handleAnimationChange();
    Serial.println("Double click detected");
    encoderClickState = ENCODER_CLICK_NONE;
  }

  encoderLastButtonState = encoderButtonState;
}



/*
 *  setup & loop functions
 *
 */
void setup() {
  Serial.begin(115200);

  // Enable internal pull-up resistor
  pinMode(pinEncoderCLK, INPUT_PULLUP);
  pinMode(pinEncoderDT, INPUT_PULLUP);
  pinMode(pinEncoderSW, INPUT_PULLUP);
  
  pinMode(pinSwitch, INPUT_PULLUP);

  pinMode(pinLED, OUTPUT);
  
  // We use a for loop to iterate through all button pins to define their pinMode
  for (int i = 0; i < buttonCount; i++) {
    pinMode(pinButtons[i], INPUT_PULLUP);
  }

  // The function attachInterrupt calls given function (we call it as ISR) event-based, in this case due to an encoder state change
  attachInterrupt(digitalPinToInterrupt(pinEncoderCLK), updateEncoder, CHANGE);

  // Initialize FastLED
  FastLED.addLeds<WS2812, pinLED, GRB>(leds, ledCount);
  FastLED.setBrightness(brightnessLED);

  // Initialize HID-Project
  Consumer.begin();
}

void loop() {
  // Handle all behaviour except encoder rotation since it is handled by an Interrupt Service Routine (updateEncoder function) that is attached in "void setup()"
  handleLED();
  handleSwitch();
  handleButtons();
  handleEncoderDoubleClick();
  
  MidiUSB.flush(); // Sends the data that is ready to go via USB

  delay(1); // For debounce purposes
}