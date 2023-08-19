long  int left = 0;
long  int right = 0;
int encoder_left_1 = 2;
int encoder_left_2 = 4;
int encoder_right_1 = 3;
int encoder_right_2 = 5;



void setup() {
Serial.begin(115200);
 
pinMode(2,INPUT);
pinMode(4,INPUT);
pinMode(3,INPUT);
pinMode(5,INPUT);

attachInterrupt(digitalPinToInterrupt(encoder_right_1), encoder_right, CHANGE);
attachInterrupt(digitalPinToInterrupt(encoder_left_1), encoder_left, CHANGE);
}

void loop() {
Serial.print("Left");
Serial.print("\t");
Serial.print(left);
Serial.print("\t");
Serial.print("right");
Serial.print("\t");
Serial.println(right);

}


void encoder_left(){
  if(digitalRead(encoder_left_1)){
    if(!digitalRead(encoder_left_2)){
      left--;
    }
    if(digitalRead(encoder_left_2)){
      left++;
    }
  }

  if(!digitalRead(encoder_left_1)){
    if(digitalRead(encoder_left_2)){
      left--;
    }
    if(!digitalRead(encoder_left_2)){
      left++;
    }
  }
}

void encoder_right(){
  if(digitalRead(encoder_right_1)){
    if(!digitalRead(encoder_right_2)){
      right++;
    }
    if(digitalRead(encoder_right_2)){
      right--;
    }
  }

  if(!digitalRead(encoder_right_1)){
    if(digitalRead(encoder_right_2)){
      right++;
    }
    if(!digitalRead(encoder_right_2)){
      right--;
    }
  }
}