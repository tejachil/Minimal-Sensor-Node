void writeByte(uint8_t address, uint8_t subAddress, uint8_t data){
  i2c_start((address << 1) | I2C_WRITE);
  i2c_write(subAddress);
  i2c_write(data);
  i2c_stop();
}

uint8_t readByte(uint8_t address, uint8_t subAddress){
  uint8_t data; // `data` will store the register data	 
  i2c_start((address << 1) | I2C_WRITE);
  i2c_write(subAddress);
  i2c_rep_start((address << 1) | I2C_READ);
  data = i2c_read(true);
  i2c_stop();
  
  return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){
  i2c_start((address << 1) | I2C_WRITE);
  i2c_write(subAddress);
  i2c_rep_start((address << 1) | I2C_READ);
  for(int i = 0; i < count; ++i){
    dest[i] = i2c_read((i == (count - 1)));
  }
  i2c_stop();
}
