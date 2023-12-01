#include "./drv8353.hpp"


//********************************************************
//Global functions
//********************************************************

void DRV8353Driver::setupSPI(int _miso_pin, int _mosi_pin, int _sck_pin, SPIClass* _spi) {
	// TODO make SPI speed configurable
	spi = _spi;
	spi->setMISO(_miso_pin);
	spi->setMOSI(_mosi_pin);
	spi->setSCLK(_sck_pin);
	settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(cs_pin, OUTPUT);
	digitalWrite(cs_pin, HIGH); // switch off

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
};

void DRV8353Driver::setupDRV8353()
{

	readStatus();
    readControl();
    readSettings();
	disable();
}

void DRV8353Driver::setupeEnableDRV8353()
{
	pinMode(enable_pin, OUTPUT); 	
}

bool DRV8353Driver::checkFault()
{
    if(isFault()) return true; //drv8353_faults::GeneralFault;
    return false; //drv8353_faults::NoFault;
}

/**
 * Clears fault values, writes a clear to the DRV8353
*/
void DRV8353Driver::clearFaults()
{
    driver_control.CLR_FLT = 0b1;
    setControlRegister();
}   

/**
 * Reads fault statuses
*/
void DRV8353Driver::readStatus()
{
    readStatusRegisters();
}

drv8353_status1 DRV8353Driver::readStatus1()
{
	return read_status1;
}

drv8353_status2 DRV8353Driver::readStatus2()
{
	return read_status2;
}

/**
 * Read only the control registers
*/
void DRV8353Driver::readControl()
{
    readControlRegister();
}

/**
 * Read only the settings registers
*/
void DRV8353Driver::readSettings()
{
    readGateDriveRegisters();
    readOCPRegister();
    readCSARegister();
}

/**
 * Send controls and update variables
*/
void DRV8353Driver::updateControl()
{
    setControlRegister();
}

/**
 * Send settings and update variables
*/
void DRV8353Driver::updateSettings()
{
    setGateDriveRegisters();
    setOCPRegister();
    setCSARegister();
}

void DRV8353Driver::enable()
{
	digitalWrite(enable_pin, HIGH); 
}

void DRV8353Driver::disable()
{
	digitalWrite(enable_pin, LOW); 
}

//********************************************************
//Private functions
//********************************************************
uint16_t DRV8353Driver::readSPI(drv8353_register_adderess addr) {
	digitalWrite(cs_pin, 0);
	spi->beginTransaction(settings);
	uint16_t data = formatAddress(addr, 0x0000, drv8353_spi_rw::spi_write);
	uint16_t result = spi->transfer16(data);
	spi->endTransaction();
	digitalWrite(cs_pin, 1);
	return result;
}

uint16_t DRV8353Driver::writeSPI(drv8353_register_adderess addr, uint16_t data) {
	digitalWrite(cs_pin, 0);
	spi->beginTransaction(settings);
	data = formatAddress(addr, data, drv8353_spi_rw::spi_write);
	uint16_t result = spi->transfer16(data);
	spi->endTransaction();
	digitalWrite(cs_pin, 1);
	return result;
}

void DRV8353Driver::readStatusRegisters()
{	
    read_status1.reg = readSPI(drv8353_register_adderess::Status1_Addr); //Expected 1st response = 0b0000 0000 0000 0000 0x0000 0
	delayMicroseconds(1); // delay at least 400ns between operations
	read_status2.reg = readSPI(drv8353_register_adderess::Status2_Addr); //Expected 1st response = 0b0000 1000 0000 0000 0x0800 2048
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::readControlRegister()
{
    driver_control.reg = readSPI(drv8353_register_adderess::Driver_Control_Addr); //Expected 1st response = 0b0001 0000 0000 0000 0x1000 4096 
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::readGateDriveRegisters()
{
    gate_drive_hs.reg = readSPI(drv8353_register_adderess::Gate_Drive_HS_Addr); //Expected 1st response = 0b0001 1011 1111 1111 0x1BFF 7167
	delayMicroseconds(1); // delay at least 400ns between operations
    gate_drive_ls.reg = readSPI(drv8353_register_adderess::Gate_Drive_LS_Addr); //Expected 1st response = 0b0010 0111 1111 1111 0x27FF 10239
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::readOCPRegister()
{
    ocp_control.reg = readSPI(drv8353_register_adderess::OCP_Control_Addr); //Expected 1st response = 0b0010 1001 0110 1001 0x2969 10601
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::readCSARegister()
{
    csa_control.reg = readSPI(drv8353_register_adderess::CSA_Control_Addr); //Expected 1st response = 0b0011 0110 0000 0011 0x3603 13827
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::setControlRegister()
{
    driver_control.reg = writeSPI(drv8353_register_adderess::Driver_Control_Addr, driver_control.reg);
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::setGateDriveRegisters()
{
    gate_drive_hs.reg = writeSPI(drv8353_register_adderess::Gate_Drive_HS_Addr, gate_drive_hs.reg);
	delayMicroseconds(1); // delay at least 400ns between operations
    gate_drive_ls.reg  = writeSPI(drv8353_register_adderess::Gate_Drive_LS_Addr, gate_drive_ls.reg);
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::setOCPRegister()
{
    ocp_control.reg = writeSPI(drv8353_register_adderess::OCP_Control_Addr, ocp_control.reg);
	delayMicroseconds(1); // delay at least 400ns between operations
}

void DRV8353Driver::setCSARegister()
{
    csa_control.reg = writeSPI(drv8353_register_adderess::CSA_Control_Addr, csa_control.reg);
	delayMicroseconds(1); // delay at least 400ns between operations
}


