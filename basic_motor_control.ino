

const int INPUT_A1= 3;  // Motor A - Input 1 (PWM)
const int INPUT_A2 = 5;  // Motor A - Input 2 (PWM)
const int INPUT_B1 = 6;  // Motor B - Input 1 (PWM)
const int INPUT_B2 = 9;  // Motor B - Input 2 (PWM)

const int pwm_25 = 40;
const int pwm_50 = 52;
const int pwm_75 = 90;
const int pwm_100 = 255;


void setup() {
  
  Serial.begin(9600);

  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);
}

void loop() {

  if (Serial.available() > 0) {
    String userInput = Serial.readStringUntil('\n');  // Read full command
    userInput.trim();  // Remove any extra whitespace

    Serial.println("Command received: " + userInput);

  if(userInput == "f25"){
    analogWrite(INPUT_A1, pwm_25); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B1, pwm_25); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  }

  else if(userInput == "f50"){
    analogWrite(INPUT_A1, pwm_50); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B1, pwm_50); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  }

  else if(userInput == "f75"){
    analogWrite(INPUT_A1, pwm_75); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B1, pwm_75); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  }

  else if(userInput == "f100"){
    analogWrite(INPUT_A1, pwm_100); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B1, pwm_100); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  } 

  //----------------------------------------------------------------------- end of front

  else if(userInput == "b25"){
    analogWrite(INPUT_A2, pwm_25); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B2, pwm_25); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }

  else if(userInput == "b50"){
    analogWrite(INPUT_A2, pwm_50); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B2, pwm_50); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }

  else if(userInput == "b75"){
    analogWrite(INPUT_A2, pwm_75); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B2, pwm_75); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }

  else if(userInput == "b100"){
    analogWrite(INPUT_A2, pwm_100); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B2, pwm_100); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  } 

  //----------------------------------------------------------------------- end of backward

  else if(userInput == "r25_1"){
    analogWrite(INPUT_A1, pwm_25); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B2, pwm_25); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }

  else if(userInput == "r75_1"){
    analogWrite(INPUT_A1, pwm_75); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B2, pwm_75); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }

  else if(userInput == "r25_2"){
    analogWrite(INPUT_A2, pwm_25); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B1, pwm_25); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  }

  else if(userInput == "r75_2"){
    analogWrite(INPUT_A2, pwm_75); //max 255, min 0
    analogWrite(INPUT_A1, 0);

    analogWrite(INPUT_B1, pwm_75); //max 255, min 0
    analogWrite(INPUT_B2, 0);
  }

  //----------------------------------------------------------------------- end of reverse direction

  else{
    analogWrite(INPUT_A1, 0); //max 255, min 0
    analogWrite(INPUT_A2, 0);

    analogWrite(INPUT_B2, 0); //max 255, min 0
    analogWrite(INPUT_B1, 0);
  }


}
}