//hangt aan de rechterpoort  -> 22
#include <VirtualWire.h>

const int transmit_pin = 7;
const int receive_pin = 0;
const int transmit_en_pin = 3;
char check = '(';

void setup()
{
    delay(1000);
    Serial.begin(9600);  // Debugging only
    Serial.println("setup");
    // Initialise the IO and ISR
    vw_set_tx_pin(transmit_pin);
    vw_set_rx_pin(receive_pin);
    vw_set_ptt_pin(transmit_en_pin);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(8000);  // Bits per sec
    vw_rx_start();       // Start the receiver PLL running
}

void loop()
{
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
  int i;
  
  // Message with a good checksum received, print it.
  if (((((char)buf[0]) == check )|| ((char)buf[buflen-1]) == check)){
  
  Serial.write(buf, buflen);
  if (check == '(')
  {check = ')';}
  else
  {check = '(';}   
  Serial.println();
  }
    }
}
