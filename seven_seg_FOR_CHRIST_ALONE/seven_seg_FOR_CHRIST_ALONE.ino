// BY GOD'S GRACE ALONE
int num_array[10][7] = {
  {1,1,1,1,1,1,0},
  {0,1,1,0,0,0,0},
  {1,1,0,1,1,0,1},
  {1,1,1,1,0,0,1},
  {0,1,1,0,0,1,1},
  {1,0,1,1,0,1,1},
  {1,0,1,1,1,1,1},
  {1,1,1,0,0,0,0},
  {1,1,1,1,1,1,1},
  {1,1,1,0,0,1,1}
};
int delay_bet_Digit = 1000;
int delay_bet_Cycle = 3000;

void setup() {
  // put your setup code here, to run once:
  for(int pin = 2; pin <= 8; pin++)
  {
    pinMode(pin,OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int counter = 1; counter <= 10; ++counter){
    delay(delay_bet_Digit);
    Num_write(counter-1);
  }

  delay(delay_bet_Cycle);
}

void Num_write(int number){
  int pin = 2;
  for (int j=0; j<7; j++){
    digitalWrite(pin, num_array[number][j]);
    pin++;
  }
}
