#include <PPMEncoder.h>

void setup() {
  Serial.begin(115200);
  ppmEncoder.begin(3);
}

bool r = 0;
int i = -1, d = 0;

# arduino takes data in form of "{Channel number}{value};"
# channe number need to be single digit
# numeration of channels goes from zero
void loop() {
  if(Serial.available()) {
    char a = Serial.read();
    if(a==';'){
      r = 1;
    }else{ 
      if(i<0){
        i = a - '0';
      }else{
        d = d * 10 + a - '0';
      }
    }
  }
  if(r){
    ppmEncoder.setChannel(i, d);
    r = d = 0; i = -1;
  }
}
