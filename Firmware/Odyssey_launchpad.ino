const int CH_1 = 3;     
const int CH_2 = 4;     
const int CH_3 = 5;     

const int RELAY_PIN = 6; 
const int LED_PIN   = 7; 
const int BUZZER    = 1; 

int step = 0; 

void setup() {
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  int b1 = digitalRead(BTN_1);
  int b2 = digitalRead(BTN_2);
  int b3 = digitalRead(BTN_3);

  if (step == 0) {
    if (b1 == LOW) {
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);

      digitalWrite(LED_PIN, HIGH); 
      step = 1;

      delay(300); 
    } 
    else if (b2 == LOW || b3 == LOW) {
      digitalWrite(BUZZER, HIGH);
      delay(400);
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED_PIN, LOW);
      step = 0;
      delay(300);
    }
  }

  if (step == 1) {
    if (b2 == LOW) {
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);

      step = 2;

      delay(300); 
    } 
    else if (b1 == LOW || b3 == LOW) {
      digitalWrite(BUZZER, HIGH);
      delay(400);
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED_PIN, LOW);
      step = 0;
      delay(300);
    }
  }

  if (step == 2) {
    if (b3 == LOW) {
      digitalWrite(RELAY_PIN, HIGH); 

      digitalWrite(BUZZER, HIGH);
      delay(150);
      digitalWrite(BUZZER, LOW);
      delay(100);
      digitalWrite(BUZZER, HIGH);
      delay(300);
      digitalWrite(BUZZER, LOW);

      step = 3; 
    } 
    else if (b1 == LOW || b2 == LOW) {
      digitalWrite(BUZZER, HIGH);
      delay(400);
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED_PIN, LOW);
      step = 0;
      delay(300);
    }
  }
}
