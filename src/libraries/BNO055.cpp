#include "BNO055.h"
#include "mbed.h"

BNO055::BNO055(PinName sda, PinName scl) : _i2c(sda,scl)
{
    //Set I2C fast and bring reset line high
    _i2c.frequency(100000);
    address = BNOAddress;
    accel_scale = 0.001f;
    rate_scale = 1.0f/16.0f;
    angle_scale = 1.0f/16.0f;
    temp_scale = 1;
}

void BNO055::reset()
{
//Perform a power-on-reset
    readchar(BNO055_SYS_TRIGGER_ADDR);
    rx = rx | 0x20;
    writechar(BNO055_SYS_TRIGGER_ADDR,rx);
//Wait for the system to come back up again (datasheet says 650ms)
    wait_ms(675);
}

bool BNO055::check()
{
//Check we have communication link with the chip
    setpage(0);
    wait_ms(2);
    readchar(BNO055_CHIP_ID_ADDR);
    if (rx != 0xA0) return false;
//Grab the chip ID and software versions
    tx[0] = BNO055_CHIP_ID_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,7,false);
    wait_ms(1);
    ID.id = rawdata[0];
    ID.accel = rawdata[1];
    ID.mag = rawdata[2];
    ID.gyro = rawdata[3];
    ID.sw[0] = rawdata[4];
    ID.sw[1] = rawdata[5];
    ID.bootload = rawdata[6];
    setpage(1);
    tx[0] = BNO055_UNIQUE_ID_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,ID.serial,16,false);
    wait_ms(1);
    setpage(0);
    wait_ms(2);
    return true;
}


void BNO055::SetExternalCrystal(bool valor)
{
// Read the current status from the device
    setmode(OPERATION_MODE_CONFIG);
    wait_ms(25);
    writechar(BNO055_PAGE_ID_ADDR,0);

    if (valor) {
        writechar(BNO055_SYS_TRIGGER_ADDR,0x80);
    } else {
        writechar(BNO055_SYS_TRIGGER_ADDR,0x00);
    }
    wait_ms(10);
}



void BNO055::set_accel_units(char units)
{
    readchar(BNO055_UNIT_SEL_ADDR);
    if(units == MPERSPERS) {
        rx = rx & 0xFE;
        accel_scale = 0.01f;
    } else {
        rx = rx | units;
        accel_scale = 0.001f;
    }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::set_anglerate_units(char units)
{
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == DEG_PER_SEC) {
        rx = rx & 0xFD;
        rate_scale = 1.0f/16.0f;
    } else {
        rx = rx | units;
        rate_scale = 1.0f/900.0f;
    }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::set_angle_units(char units)
{
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == DEGREES) {
        rx = rx & 0xFB;
        angle_scale = 1.0f/16.0f;
    } else {
        rx = rx | units;
        angle_scale = 1.0f/900.0f;
    }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::set_temp_units(char units)
{
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == CENTIGRADE) {
        rx = rx & 0xEF;
        temp_scale = 1;
    } else {
        rx = rx | units;
        temp_scale = 2;
    }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::set_orientation(char units)
{
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == WINDOWS) rx = rx & 0x7F;
    else rx = rx | units;
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::config_BW(char adr, char bandwidth) // char range, char bandwidth
{
    setpage(1);
    wait_ms(5);
    writechar(adr,bandwidth);
    
    }

void BNO055::setmode(char omode)
{
    writechar(BNO055_OPR_MODE_ADDR,omode);
    op_mode = omode;
    wait_ms(30);
}

void BNO055::setpowermode(char pmode)
{
    writechar(BNO055_PWR_MODE_ADDR,pmode);
    pwr_mode = pmode;
    wait_ms(30);

}

void BNO055::get_accel(void)
{
    tx[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    accel.rawx = (rawdata[1] << 8 | rawdata[0]);
    accel.rawy = (rawdata[3] << 8 | rawdata[2]);
    accel.rawz = (rawdata[5] << 8 | rawdata[4]);
    accel.x = double(accel.rawx)*accel_scale;
    accel.y = double(accel.rawy)*accel_scale;
    accel.z = double(accel.rawz)*accel_scale;
}

void BNO055::get_gyro(void)
{
    tx[0] = BNO055_GYRO_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    gyro.rawx = (rawdata[1] << 8 | rawdata[0]);
    gyro.rawy = (rawdata[3] << 8 | rawdata[2]);
    gyro.rawz = (rawdata[5] << 8 | rawdata[4]);
    gyro.x = double(gyro.rawx)*rate_scale;
    gyro.y = double(gyro.rawy)*rate_scale;
    gyro.z = double(gyro.rawz)*rate_scale;
}

void BNO055::get_mag(void)
{
    tx[0] = BNO055_MAG_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    mag.rawx = (rawdata[1] << 8 | rawdata[0]);
    mag.rawy = (rawdata[3] << 8 | rawdata[2]);
    mag.rawz = (rawdata[5] << 8 | rawdata[4]);
    mag.x = double(mag.rawx);
    mag.y = double(mag.rawy);
    mag.z = double(mag.rawz);
}

void BNO055::get_lia(void)
{
    tx[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    lia.rawx = (rawdata[1] << 8 | rawdata[0]);
    lia.rawy = (rawdata[3] << 8 | rawdata[2]);
    lia.rawz = (rawdata[5] << 8 | rawdata[4]);
    lia.x = double(lia.rawx)*accel_scale;
    lia.y = double(lia.rawy)*accel_scale;
    lia.z = double(lia.rawz)*accel_scale;
}

void BNO055::get_grv(void)
{
    tx[0] = BNO055_GRAVITY_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    gravity.rawx = (rawdata[1] << 8 | rawdata[0]);
    gravity.rawy = (rawdata[3] << 8 | rawdata[2]);
    gravity.rawz = (rawdata[5] << 8 | rawdata[4]);
    gravity.x = double(gravity.rawx)*accel_scale;
    gravity.y = double(gravity.rawy)*accel_scale;
    gravity.z = double(gravity.rawz)*accel_scale;
}

void BNO055::get_quat(void)
{
    tx[0] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,8,0);
    wait_ms(1);
    quat.raww = (rawdata[1] << 8 | rawdata[0]);
    quat.rawx = (rawdata[3] << 8 | rawdata[2]);
    quat.rawy = (rawdata[5] << 8 | rawdata[4]);
    quat.rawz = (rawdata[7] << 8 | rawdata[6]);
    quat.w = double(quat.raww)/16384.0f;
    quat.x = double(quat.rawx)/16384.0f;
    quat.y = double(quat.rawy)/16384.0f;
    quat.z = double(quat.rawz)/16384.0f;
}

void BNO055::get_angles(void)
{
    tx[0] = BNO055_EULER_H_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address+1,rawdata,6,0);
    wait_ms(1);
    euler.rawyaw = (rawdata[1] << 8 | rawdata[0]);
    euler.rawroll = (rawdata[3] << 8 | rawdata[2]);
    euler.rawpitch = (rawdata[5] << 8 | rawdata[4]);
    euler.yaw = double(euler.rawyaw)*angle_scale;
    euler.roll = double(euler.rawroll)*angle_scale;
    euler.pitch = double(euler.rawpitch)*angle_scale;
}

uint8_t BNO055::getSystemStatus(char location)
{
  setpage(0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

//  if (system_status != 0)
//    *system_status = 
    readchar(location);

//  /* Self Test Results (see section )
//     --------------------------------
//     1 = test passed, 0 = test failed
//
//     Bit 0 = Accelerometer self test
//     Bit 1 = Magnetometer self test
//     Bit 2 = Gyroscope self test
//     Bit 3 = MCU self test
//
//     0x0F = all good! */
//
//  if (self_test_result != 0)
//    *self_test_result = readchar(BNO055_SELFTEST_RESULT_ADDR);
//
//  /* System Error (see section 4.3.59)
//     ---------------------------------
//     0 = No error
//     1 = Peripheral initialization error
//     2 = System initialization error
//     3 = Self test result failed
//     4 = Register map value out of range
//     5 = Register map address out of range
//     6 = Register map write error
//     7 = BNO low power mode not available for selected operat ion mode
//     8 = Accelerometer power mode not available
//     9 = Fusion algorithm configuration error
//     A = Sensor configuration error */
//
//  if (system_error != 0)
//    *system_error     = readchar(BNO055_SYS_ERR_ADDR);

  //wait_ms(5);
  return rx;
}

void BNO055::get_temp(void)
{
    readchar(BNO055_TEMP_ADDR);
    temperature = rx / temp_scale;
}

void BNO055::get_calib(void)
{
    readchar(BNO055_CALIB_STAT_ADDR);
    calib = rx & 0x03;
}

void BNO055::read_calibration_data(void)
{
    char tempmode = op_mode;
    setmode(OPERATION_MODE_CONFIG);
    wait_ms(20);
    tx[0] = ACCEL_OFFSET_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.read(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
}

void BNO055::write_calibration_data(void)
{

    char tempmode = op_mode;
    setmode(OPERATION_MODE_CONFIG);
    wait_ms(20);
    setpage(0);


// Clibração da aceleração no eixo X
    tx[0] = ACCEL_OFFSET_X_MSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
    
    // Clibração da aceleração no eixo Y
    tx[0] = ACCEL_OFFSET_Y_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
    
    // Clibração da aceleração no eixo Z
    tx[0] = ACCEL_OFFSET_Z_MSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
    
    
    // Clibração da giroscópio no eixo X
    tx[0] = GYRO_OFFSET_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
    
    // Clibração da giroscópio no eixo Y
    tx[0] = GYRO_OFFSET_Y_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
    
    // Clibração da giroscópio no eixo Z
    tx[0] = GYRO_OFFSET_Z_LSB_ADDR;
    _i2c.write(address,tx,1,true);
    wait_ms(1);
    _i2c.write(address,calibration,22,false);
    wait_ms(1);
    setmode(tempmode);
    wait_ms(10);
}

void BNO055::set_mapping(char orient)
{
    switch (orient) {
        case 0:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x04);
            break;
        case 1:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 2:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 3:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x02);
            break;
        case 4:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x03);
            break;
        case 5:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x01);
            break;
        case 6:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x07);
            break;
        case 7:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x05);
            break;
        default:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
    }
}