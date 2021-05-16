double kp =4.415 ;  //initialising PID coefficients
double ki =961.9 ;
double kd = 0.0003425;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint_10, setPoint_8;
double cumError, rateError;

const int setpoint_inp=A1;    //potentiometer connected to pin A1 for user defined setpoint to the buck converter

//current sensor parameters

const int analogIn = A0;
int mVperAmp = 66;          // 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; 
double Voltage = 0;
double Amps = 0;

void setup(){
        setPoint_10 = analogRead(setpoint_inp);   //set point at zero degrees
        setPoint_8 = map(setPoint_10,0,1023,0,255);  // Mega2560 has a 10 bit ADC. It must be mapped according to required 8-bit value.
        analogWrite(3,0);                      //setting duty cycle to zero intitially
}    

void loop(){
        setPoint_10 = analogRead(setpoint_inp);       // refresh value of setpoint
        setPoint_8 = map(setPoint_10,0,1023,0,255);
        
        input = current_feedback();                //read from ACS712 current sensor
        output = computePID(input);
        delay(100);
        analogWrite(3, output);                //control the duty cycle of the buck converter

}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint_8 - inp;                              // determine error
        cumError += error * elapsedTime;                     // compute integral
        rateError = (error - lastError)/elapsedTime;         // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;  //PID output               

        lastError = error;                                  //remember current error
        previousTime = currentTime;                         //remember current time

        return out;                                         //have function return the PID output. This value ranges from 0-255.
}


double current_feedback()
{
   RawValue = analogRead(analogIn);
   Voltage = map(RawValue,0,1023,0,5000); // Gets you mV
   Amps = abs(((Voltage - ACSoffset) / mVperAmp));
   return(Voltage);   //use this for voltage feedback
   return(Amps);      //use this for current feedback
}
