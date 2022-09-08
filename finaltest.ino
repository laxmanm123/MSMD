#include <FastLED.h>

#define NUM_LEDS 30
#define LED_L 13
#define LED_R 15
#define SPEAKER_MET 12
#define SPEAKER_NOTE 14
#define PIEZO_R 33
#define PIEZO_L 32

const uint8_t NOTE_PWM_CHANNEL = 0;
const uint8_t MET_PWM_CHANNEL = 2;
const CRGB LED_OFF = CRGB::Black;

#define bpm 750
#define threshold_voltage 7     
#define MISS_THRESHOLD 325.0 // ±350ms < extra hit; miss no increment
#define HIT_THRESHOLD 175.0    // ±175ms < miss <= ±350ms 
#define MET_DELAY 40          // hit <= ±175ms
#define NOTE_DELAY 100
#define NULL_NOTE 1
#define PASS_HIT_RATIO 3

class Pattern {
  private: 
    unsigned long *_left;
    unsigned long *_right;
    unsigned long pLength;
    int _leftIndex = 0;
    int _rightIndex = 0;
    int lenL;
    int lenR;
  public:
    // constructor, takes left and right note array, as well as “length”: how long the pattern plays for
    Pattern(unsigned long leftPtr[], unsigned long rightPtr[], int lenL, int lenR){
      this-> _left = leftPtr;
      this-> _right = rightPtr;
      this-> pLength = max(_left[lenL-1], _right[lenR-1]) + MISS_THRESHOLD - 100;
      this-> lenL = (int)lenL;
      this-> lenR = (int)lenR;
    }
    // 
    unsigned long peekNote(char side){
      if(side == 'L' && _leftIndex < lenL){return _left[_leftIndex];}
      if(side == 'R' && _rightIndex < lenR){return _right[_rightIndex];}
      return NULL_NOTE;
    }
    void incNote(char side){
      if(side == 'L'){_leftIndex++;}
      if(side == 'R'){_rightIndex++;}
    }
    unsigned long getLength(){
      return this->pLength;
    }
    void resetIndex(){
      this->_rightIndex = 0;
      this->_leftIndex = 0;
    }
};

// Sequence one  L R L R
unsigned long L1[] = {0, 1500};
unsigned long R1[] = {750, 2250};
Pattern p1(L1, R1, 2,2);
// Sequence two  R R L L
unsigned long L2[] = {1500, 2250};
unsigned long R2[] = {0, 750};
Pattern p2(L2, R2, 2,2);
// Sequence three L L R      R   L   L  L R L
unsigned long L3[] = {0, 375, 3000, 3750, 4500, 5250};
unsigned long R3[] = {750, 2250, 4875};
Pattern p3(L3, R3, 6,3);

// Sequence four R R L L R L
unsigned long L4[] = {1500, 2250, 3750}; 
unsigned long R4[] = {0, 750, 3000};
Pattern p4(L4, R4,3,3);

// Sequence five L L L R R R
unsigned long L5[] = {0, 750, 1500}; 
unsigned long R5[] = {2250, 3000, 3750};
Pattern p5(L5, R5,3,3);

// Sequence six R L R R L L
unsigned long L6[] = {750, 3000, 3750}; 
unsigned long R6[] = {0, 1500, 2250};
Pattern p6(L6, R6, 3, 3);

// Sequence seven L L R R R L R L 
unsigned long L7[] = {0, 750, 3750, 5250}; 
unsigned long R7[] = {1500, 2250, 3000, 4500};
Pattern p7(L7, R7,4 ,4);

// Sequence 8 R L R L L R  L R
unsigned long L8[] = {750, 2250, 3000, 4500}; 
unsigned long R8[] = {0, 1500, 3750, 5250};
Pattern p8(L8, R8, 4, 4);


// Create patterns like above and access them from patternArr
// done manually
Pattern patternArr[] = {p1, p2, p4, p5, p6, p7, p8};

CRGB leds_R[NUM_LEDS];
CRGB leds_L[NUM_LEDS];

boolean metOn = false;
// Start here
void setup(){
  // Do pin assigning and stuff here
  ledcAttachPin(SPEAKER_NOTE, NOTE_PWM_CHANNEL);
  ledcAttachPin(SPEAKER_MET, MET_PWM_CHANNEL);
  FastLED.addLeds<NEOPIXEL, LED_R>(leds_R, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LED_L>(leds_L, NUM_LEDS);
  

  Serial.begin(9600); 
  delay(1000);
  Serial.println("ready to start");
  playMetSound();
  //Serial.println(p3.getLength());

  // first “block” waiting for a signal to start the game
  boolean waiting = true;
  while(waiting){if(digitalRead(0) == 0) { waiting = false; Serial.println("BUTTON PRESSED TO START");} }

  // first “block” cleared, now continue to read out pattern and play game in general
  for(int i = 0; i < 1; i++) {
    // patternForThisRound = getRandomPattern();   TODO: MAKE PATTERNS RANDOM
    Pattern patternForThisRound = patternArr[2];
    play(patternForThisRound);
    delay(1000);
    playerTurn(patternForThisRound);
  }
}


void loop() {
  // do nothing
}

void play(Pattern pattern){
  unsigned long currTime = millis();
  unsigned long lastMet = 0;

  // Countdown 3,2,1,GO,  then on next beat write pattern
  playCountdown(currTime, lastMet);
  
  Serial.println("Now Playing Pattern");
  currTime = millis();
  unsigned long startTime = currTime;
  unsigned long left;
  unsigned long right;
  unsigned long lastLeft = 0;
  unsigned long lastRight = 0;
  while(currTime <= startTime + pattern.getLength()){
    currTime = millis();
    // metronome 
    delayPlaying('m', lastMet, currTime);
    if(currTime > lastMet + bpm){playMetSound(); lastMet = currTime; metOn = true;}

    // notes
    delayPlaying('L', lastLeft, currTime);
    delayPlaying('R', lastRight, currTime);
    left = pattern.peekNote('L');
    right = pattern.peekNote('R');
    if(left!=NULL_NOTE && currTime - startTime >= left){playNote('L'); pattern.incNote('L'); lastLeft = currTime;}
    if(right!=NULL_NOTE && currTime - startTime >= right){playNote('R'); pattern.incNote('R'); lastRight = currTime;}
  }
  pattern.resetIndex();
  Serial.println("Pattern Playing Done");
}

void playerTurn(Pattern pattern){
  unsigned long currTime = millis();
  unsigned long lastMet = currTime;
  Serial.println("Player Turn Start");

  // Countdown 3,2,1,GO,  then on next beat write pattern
  playCountdown(currTime, lastMet);
  
  // get player inputs
  Serial.println("Now Getting Player Pattern");
  currTime = millis();
  unsigned long startTime = currTime;
  unsigned long left;
  unsigned long right;
  unsigned long lastLeft = 0;
  unsigned long lastRight = 0;

  int tempArr[] = {0};

  int hitRatio[] = {0,0}; // arr[0] is # hit, arr[1] is # miss
  while(currTime <= startTime + pattern.getLength() + 150){
    currTime = millis();
    // metronome 
    delayPlaying('m', lastMet, currTime);
    if(currTime > lastMet + bpm){playMetSound(); lastMet = currTime; metOn = true;}

    // notes     
    delayPlaying('L', lastLeft, currTime);
    delayPlaying('R', lastRight, currTime);
    left = (checkForHit('L')) ? currTime - startTime: NULL_NOTE;
    right = (checkForHit('R')) ? currTime - startTime: NULL_NOTE;

    // checking if next note is too late to hit
    unsigned long temp = currTime - startTime;
    unsigned long next = pattern.peekNote('L');
    if(next != NULL_NOTE && temp > next && temp - next >= MISS_THRESHOLD){
      hitRatio[1]++; pattern.incNote('L'); Serial.println("passed note: too late to hit ");}
    next = pattern.peekNote('R');
    if(next != NULL_NOTE && temp > next && temp - next >= MISS_THRESHOLD){
      hitRatio[1]++; pattern.incNote('R'); Serial.println("passed note: too late to hit ");}
    
    if(left!=NULL_NOTE && currTime - lastLeft > 100){processHitMiss(left, pattern, 'L', hitRatio); lastLeft = currTime;}
    if(right!=NULL_NOTE && currTime - lastRight > 100){processHitMiss(right, pattern, 'R', hitRatio); lastRight = currTime;}
  }

  // display pass or fail
    delay(3000);
    if(hitRatio[1] == 0){leds_on_green('L'); leds_on_green('R'); } else 
    if(hitRatio[0] / hitRatio[1] >= PASS_HIT_RATIO){leds_on_green('L'); leds_on_green('R');} else 
    {leds_on_red('L');leds_on_red('R');}
    delay(2000);
    leds_off('L'); leds_off('R');
    pattern.resetIndex();
    Serial.println("Player Input Phase Done");
    Serial.print("Hits: ");Serial.print(hitRatio[0]);Serial.print("       Misses: ");Serial.print(hitRatio[1]); 
}

void playCountdown(unsigned long &currTime, unsigned long &lastMet){
  Serial.println("Countdown Start");
  int count = 3;
  while(count >= 0){
    currTime = millis();
    if(count != 3){delayPlaying('m', lastMet, currTime);}
    if(currTime > lastMet + bpm){playMetSound(); lastMet = currTime; count--; metOn = true;}
  }
  Serial.println("Countdown Done");
  // delay after Go: makes reading start on beat
  while(millis() < lastMet + bpm){delayPlaying('m', lastMet, millis());}
}

void processHitMiss(unsigned long hit, Pattern &p, char side, int *arr){
  int r;
  unsigned long next = p.peekNote(side);
  unsigned long e1 = (hit >= next) ? hit : next;
  unsigned long e2 = (hit >= next) ? next : hit;
  if(next == NULL_NOTE)       {r =-2;} else
  if(e1 - e2 > MISS_THRESHOLD){r =-1;} else
  if(e1 - e2 <= HIT_THRESHOLD){r = 0;} else
/*if(hit is a miss)*/         {r = 1;}

  switch(r){  // console log for testing
    case -2:
      arr[1]++; playMiss(side); 
      Serial.print("extra note: no more notes to hit: ");
      Serial.print(e1); Serial.print(" "); Serial.print(e2); Serial.print(" ");Serial.println(e1 - e2);
      break;
    case -1:
      arr[1]++; playMiss(side); 
      Serial.print("extra note: outside miss threshold: ");
      Serial.print(e1); Serial.print(" "); Serial.print(e2); Serial.print(" ");Serial.println(e1 - e2);
      break;
    case 0:
      arr[0]++; p.incNote(side); playNote(side); 
      Serial.print("hit: ");
      Serial.print(e1); Serial.print(" "); Serial.print(e2); Serial.print(" ");Serial.println(e1 - e2);
      break;
    case 1:
      arr[1]++; p.incNote(side); playMiss(side); 
      Serial.print("miss: ");
      Serial.print(e1); Serial.print(" "); Serial.print(e2); Serial.print(" ");Serial.println(e1 - e2);
      break;
    default:
      Serial.print("ayo wut"); break;
  }
}


boolean checkForHit(char c){
  if(c == 'L') {
      float piezo_v = analogRead(PIEZO_L) / 1023.0 * 5.0;
      return piezo_v > threshold_voltage;
  }
  if(c == 'R') {
      float piezo_v = analogRead(PIEZO_R) / 1023.0 * 5.0;
      return piezo_v > threshold_voltage;
  } 
}


// Just turn the speaker on
void playMetSound(){
  ledcWriteNote(MET_PWM_CHANNEL, NOTE_B, 4);
  //Serial.print("Tik");
  metOn = true;
}


// Just turn the speaker and LEDs on
void playMiss(char c){
  if(c == 'L'){leds_on_red('L');Serial.print("left, ");}
  if(c == 'R'){leds_on_red('R');Serial.print("right, ");}
}

// Just turn the speaker and LEDs on
void playNote(char c){
  if(c == 'L'){ledcWriteNote(NOTE_PWM_CHANNEL, NOTE_C, 6); leds_on_green('L'); Serial.print("left, "); }
  if(c == 'R'){ledcWriteNote(NOTE_PWM_CHANNEL, NOTE_C, 6); leds_on_green('R'); Serial.print("right, ");}
}



// Used to turn off speakers and LEDs after NOTE_DELAYms, not needed if speakers or leds
// can delay on their own without interrupting the program. Each output needs a variable that 
// tracks that last time the output was triggered
void delayPlaying(char kind, unsigned long last, unsigned long currTime){
  int DELAY = (kind == 'm') ? MET_DELAY : NOTE_DELAY;
  if(isPlaying(kind) && currTime >= last + DELAY){
    if(kind == 'm'){ ledcWrite(MET_PWM_CHANNEL, 0); metOn = false; }
    if(kind == 'L') {  
      leds_off('L');
      if(!isPlaying('R')) {
        ledcWrite(NOTE_PWM_CHANNEL,0);
      }
    }
    if(kind == 'R') {
       leds_off('R'); 
       if(!isPlaying('L')) {
        ledcWrite(NOTE_PWM_CHANNEL,0);
       }
    }
  }
}

boolean isPlaying(char c){
  if(c == 'm'){return metOn;}
  if(c == 'L'){return (leds_L[1]!= LED_OFF) ? true:false;}
  if(c == 'R'){return (leds_R[1]!= LED_OFF) ? true:false;}
  return false;
}

void leds_off(char c) {
  for(int i = 0; i < NUM_LEDS; i++) {
   if(c == 'L') {
    leds_L[i] = CRGB::Black;
   }
   if(c == 'R') { 
    leds_R[i] = CRGB::Black;
   }
  }
  FastLED.show();
}

void leds_on_green(char c) {
  for(int i = 0; i < NUM_LEDS; i++) {
   if(c == 'L') {
    leds_L[i] = CRGB::Green;
   }
   if(c == 'R') {
    leds_R[i] = CRGB::Green;
   }
  }
  FastLED.show();
}

void leds_on_red(char c) {
  for(int i = 0; i < NUM_LEDS; i++) {
   if(c == 'L') {
    leds_L[i] = CRGB::Red;
   }
   if(c == 'R') {
    leds_R[i] = CRGB::Red;
   }
  }
  FastLED.show();
}
