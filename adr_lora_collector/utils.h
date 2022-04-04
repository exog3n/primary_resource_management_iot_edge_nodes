void printHex2(unsigned value)
{
    value &= 0xFF ;
    
    if (value < 16)
    {
      Serial.print('0');
    }
    
    Serial.print(value, HEX);
}
