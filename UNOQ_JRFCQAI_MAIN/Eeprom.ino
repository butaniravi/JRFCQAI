
/*
 * EEPROM
 */
void EEPROM_Write(uint8_t addr, int8_t data)
{
  Wire1.beginTransmission(eeprom_address);
  Wire1.write(addr);
  Wire1.write(data);
  Wire1.endTransmission();
  delay(1);
}

/*
 * EEPROM
 */
int8_t EEPROM_Read(uint8_t addr)
{
  int8_t data = 0xFF;
  Wire1.beginTransmission(eeprom_address);
  Wire1.write(addr);
  Wire1.endTransmission();
  Wire1.requestFrom(eeprom_address, 1);

  if (Wire1.available()) data = Wire1.read();
  return data;
}
