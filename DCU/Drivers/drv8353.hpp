
/**
 * \file  drv8353.hpp
 * 
 * \brief Device driver for the DRV8353 SPI/PWM Three Phase BLDC FET driver. 
 *        This module allows for interfacing with the driver to read/write 
 *        driver/amplifier/drive settings from/to it.
 * 
 * Copyright, 2023 DoorDash, Inc. All Rights Reserved.
 */

#pragma once 

//==============================================================================
// Includes
//==============================================================================

#include "Arduino.h"
#include <SPI.h>

//==============================================================================
// Defines
//==============================================================================

//==============================================================================
// Types
//==============================================================================

/**
 * Status Registers for the DRV8353. Holds fault codes/faults
*/
typedef union {
	struct __attribute__((__packed__)) {
		uint16_t FAULT:1;
		uint16_t VSD_OCP:1;
		uint16_t GDF:1;
		uint16_t UVLO:1;
		uint16_t OTSD:1;
		uint16_t VDS_HA:1;
		uint16_t VDS_LA:1;
        uint16_t VDS_HB:1;
		uint16_t VDS_LB:1;
        uint16_t VDS_HC:1;
		uint16_t VDS_LC:1;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_status1;

typedef union {
	struct __attribute__((__packed__)) {
		uint16_t SA_OC:1;
		uint16_t SB_OC:1;
		uint16_t SC_OC:1;
		uint16_t OTW:1;
		uint16_t GDUV:1;
		uint16_t VGS_HA:1;
		uint16_t VGS_LA:1;
        uint16_t VGS_HB:1;
		uint16_t VGS_LB:1;
        uint16_t VGS_HC:1;
		uint16_t VGS_LC:1;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_status2;

/**
 * Control Register for the DRV8353. Allows you to change settings.
 * Has the brake/coast control in it.
*/
typedef union {
	struct __attribute__((__packed__)) {
		uint16_t OCP_ACT:1;
		uint16_t DIS_GDUV:1;
		uint16_t DIS_GDF:1;
		uint16_t OTW_REP:1;
		uint16_t PWM_MODE:2;
		uint16_t PWM_COM1:1;
		uint16_t PWM_DIR1:1;
        uint16_t COAST:1;
		uint16_t BRAKE:1;
        uint16_t CLR_FLT:1;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_control;

/**
 * Drive setting Registers for the DRV8353.
 * Set drive voltage/time/current for FETs
*/
typedef union {
	struct __attribute__((__packed__)) {
		uint16_t LOCK:3;
		uint16_t IDRIVEP_HS:4;
		uint16_t IDRIVEN_HS:4;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_gate_drive_hs;

typedef union {
	struct __attribute__((__packed__)) {
		uint16_t CBC:1;
        uint16_t TDRIVE:1;
		uint16_t IDRIVEP_LS:4;
		uint16_t IDRIVEN_LS:4;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_gate_drive_ls;

/**
 * Drive setting Registers for the DRV8353.
 * Set drive voltage/time/current for FETs
*/
typedef union {
	struct __attribute__((__packed__)) {
		uint16_t TRETRY:1;
        uint16_t DEAD_TIME:2;
		uint16_t OCP_MODE:2;
		uint16_t OCP_DEG:2;
        uint16_t VDS_LVL:4;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_ocp_control;

typedef union {
	struct __attribute__((__packed__)) {
		uint16_t CSA_FET:1;
        uint16_t VREF_DIV:1;
        uint16_t LS_REF:1;
        uint16_t CSA_GAIN:2;
		uint16_t DIS_SEN:1;
		uint16_t CSA_CAL_A:1;
        uint16_t CSA_CAL_B:1;
        uint16_t CSA_CAL_C:1;
        uint16_t SEN_LVL:2;
		uint16_t ADDR:5;
	};
	uint16_t reg;
} drv8353_csa_control;


/**
 * Register addresses on the DRV8353
*/
enum class drv8353_spi_rw
{
    spi_read = 0x10,
    spi_write = 0x00,
};

/**
 * Register addresses on the DRV8353
*/
enum class drv8353_register_adderess
{
    Status1_Addr = 0x0,
    Status2_Addr = 0x1,
    Driver_Control_Addr = 0x2,
    Gate_Drive_HS_Addr = 0x3,
    Gate_Drive_LS_Addr = 0x4,
    OCP_Control_Addr = 0x5,
    CSA_Control_Addr = 0x6
};

/**
 * All possible DRV8353 faults
*/
enum class drv8353_faults
{
    NoFault = 0,
    GeneralFault,
    VDSOvercurrentFault,
    GateDriveFault,
    UndervoltageLockoutFault,
    OverTempShutdown,
    VDSOverCurrentHAFault,
    VDSOverCurrentLAFault,
    VDSOverCurrentHBFault,
    VDSOverCurrentLBFault,
    VDSOverCurrentHCFault,
    VDSOverCurrentLCFault,    
    OvercurrentSOA,
    OvercurrentSOB,
    OvercurrentSOC,
    OverTempWarning,
    GateDischargeUnderVoltage,
    VGSHAFault,
    VGSLAFault,
    VGSHBFault,
    VGSLBFault,
    VGSHCFault,
    VGSLCFault
};

/**
 * OCP Actuation 
*/
enum class drv8353_ocp_act {
	Single_HB_Shutdown = 0b0, //Single half-bridge shutdown in response to VDS_OCP and SEN_OCP
	Triple_HB_Shutdown = 0b1  //All half-bridges shutdown in response to VDS_OCP and SEN_OCP
};

/**
 * VCP Undervoltage Fault Enable/Disable
*/
enum class drv8353_dis_gduv {
	Lockout_Fault_Enabled = 0b0, //VCP and VGLS undervoltage lockout fault is enabled
	Lockout_Fault_Disabled = 0b1  //VCP and VGLS undervoltage lockout fault is disabled
};

/**
 * Gate Drive Fault Enable/Disable
*/
enum class drv8353_gdf {
	Gate_Drive_Fault_Enabled = 0b0, 
	Gate_Drive_Fault_Disabled = 0b1
};

/**
 * Overtemp Reporting enable/disable on fault/nFault pins
*/
enum class drv8353_otw_reporting {
	OTW_Not_Reported = 0b0, 
	OTW_Reported = 0b1
};

/**
 * Determine primarily function of the driver
*/
enum class drv8353_pwmmode {
	PWM6_Mode = 0b00,
	PWM3_Mode = 0b01,
	PWM1_Mode = 0b10,
	PWM_Independant_Mode = 0b11
};

/**
 * Commutation for 1x PWM Mode
*/
enum class drv8353_commutation_pwm1mode {
	SynchronousPWM_Mode = 0b0,
	AsynchronousPWM_Mode = 0b1
};

/**
 * Overide for direction pin
*/
enum class drv8353_direction_pwm1mode {
	Override_Disabled = 0b0,
	Override_Forward = 0b1
};

/**
 * Coast. Puts all FETs in High Z
*/
enum class drv8353_coast {
	Coast_Mode_Disable = 0b0,
	Coast_Mode_Enable = 0b1
};

/**
 * Brake. Shunts all low side FETs
*/
enum class drv8353_brake {
	Shunt_Brake_Off = 0b0,
	Shunt_Brake_On = 0b1
};

/**
 * Gate drive lock
*/
enum class drv8353_gdlock {
	Lock = 0b0,
	Unlock = 0b1
};

/**
 * High side/low side gate drive current sourcing
*/
enum class drv8353_gdip {
	GDISP_50mA = 0b0000,
    GDISP_100mA = 0b0010,
	GDISP_150mA = 0b0011,
    GDISP_300mA = 0b0100,
	GDISP_350mA = 0b0101,
    GDISP_400mA = 0b0110,
	GDISP_450mA = 0b0111,
    GDISP_550mA = 0b1000,
	GDISP_600mA = 0b1001,
    GDISP_650mA = 0b1010,
	GDISP_700mA = 0b1011,
    GDISP_850mA = 0b1100,
	GDISP_900mA = 0b1101,
    GDISP_950mA = 0b1110,
	GDISP_1000mA = 0b0111
};

/**
 * High side/low side gate drive current sinking
*/
enum class drv8353_gdin {
	GDISN_100mA = 0b0000,
    GDISN_200mA = 0b0010,
	GDISN_300mA = 0b0011,
    GDISN_600mA = 0b0100,
	GDISN_700mA = 0b0101,
    GDISN_800mA = 0b0110,
	GDISN_900mA = 0b0111,
    GDISN_1100mA = 0b1000,
	GDISN_1200mA = 0b1001,
    GDISN_1300mA = 0b1010,
	GDISN_1400mA = 0b1011,
    GDISN_1700mA = 0b1100,
	GDISN_1800mA = 0b1101,
    GDISN_1900mA = 0b1110,
	GDISN_2000mA = 0b0111
};

/**
 * Clear fault options
*/
enum class drv8353_cbc {
	Fault_Clear_tRetry = 0b0,
	Fault_Clear_PWM_tRetry = 0b1
};

/**
 * Gate driving time
*/
enum class drv8353_tdrive {
	GDTime_500ns = 0b00,
	GDTime_1000ns = 0b01,
    GDTime_2000ns = 0b10,
	GDTime_4000ns = 0b11
};

/**
 * Retry time for VSC_OCP
*/
enum class drv8353_tretry {
	Retry_8ms = 0b0,
	Retry_50us = 0b1
};

/**
 * Gate driving time
*/
enum class drv8353_dead_time {
	Deadtime_50ns = 0b00,
	Deadtime_100ns = 0b01,
    Deadtime_200ns = 0b10,
	Deadtime_400ns = 0b11
};

/**
 * Overcurrent mode
*/
enum class drv8353_ocp_mode {
	Overcurrent_Latched_Fault = 0b00,
	Overcurrent_Auto_Retry_Fault = 0b01,
    Overcurrent_Report_Only_No_Action = 0b10,
	Overcurrent_No_Report_No_Action = 0b11
};

/**
 * Overcurrent deglitch
*/
enum class drv8353_ocp_deg {
	Overcurrent_Deglitch_1us = 0b00,
	Overcurrent_Deglitch_2us = 0b01,
    Overcurrent_Deglitch_4us = 0b10,
	Overcurrent_Deglitch_8us = 0b11
};

/**
 * Voltage drain/source levels
*/
enum class drv8353_vds_level {
	VDS_60mV = 0b0000,
	VDS_70mV = 0b0001,
    VDS_80mV = 0b0010,
	VDS_90mV = 0b0011,
    VDS_100mV = 0b0100,
	VDS_200mV = 0b0101,
    VDS_300mV = 0b0110,
	VDS_400mV = 0b0111,
    VDS_500mV = 0b1000,
	VDS_600mV = 0b1001,
    VDS_700mV = 0b1010,
	VDS_800mV = 0b1011,
    VDS_900mV = 0b1100,
	VDS_1000mV = 0b1101,
    VDS_1500mV = 0b1110,
	VDS_2000mV = 0b0111
};

/**
 * Current sense amplifier positive input
*/
enum class drv8353_csa_fet {
	CSA_Positive_SPx = 0b0,
	CSA_Positive_SHx = 0b1
};

/**
 * VRef Division. Bidirectional/Unidirectional (bidirectional is 1/2 uni)
*/
enum class drv8353_vref_div {
	CSA_Unidirectional = 0b0,
	CSA_Bidrectional = 0b1
};

/**
 * Lowside OCP reference selection
*/
enum class drv8353_ls_ref {
	OCP_LS_SHx_to_SPx = 0b0,
	OCP_LS_SHx_to_SNx = 0b1
};

/**
 * Current sense amplifier gain
*/
enum class drv8353_csa_gain {
	CSA_Gain_5VpV = 0b00,
	CSA_Gain_10VpV = 0b01,
    CSA_Gain_20VpV = 0b10,
	CSA_Gain_40VpV = 0b11
};

/**
 * Enable/Disable CSA overcurrent fault
*/
enum class drv8353_dis_sense {
	Overcurrent_Sense_Enable = 0b0,
	Overcurrent_Sense_Disable = 0b1
};

/**
 * Enable/Disable CSA overcurrent fault
*/
enum class drv8353_csa_cal_x {
	Normal_Operation = 0b0,
	Shorted_Inputs_Offset_Calibration = 0b1
};

/**
 * Sensing level for over current protection
*/
enum class drv8353_sen_lvl {
	CSA_OCP_250mV = 0b00,
	CSA_OCP_500mV = 0b01,
    CSA_OCP_750mV = 0b10,
	CSA_OCP_1000mV = 0b11
};

//==============================================================================
// Class definitions
//==============================================================================

class DRV8353Driver {

	public:
		DRV8353Driver(int _cs_pin, int _enable_pin, int _nfault_pin) : cs_pin(_cs_pin), enable_pin(_enable_pin), nfault_pin(_nfault_pin), spi(&SPI), settings(1000000, MSBFIRST, SPI_MODE1) {};
		virtual ~DRV8353Driver() {};

		void setupSPI(int _miso_pin = PIN_SPI_MISO, int _mosi_pin = PIN_SPI_MOSI, int _sck_pin = PIN_SPI_SCK, SPIClass* _spi = &SPI);

        void setupDRV8353();            //Setup the DRV8353 class variables
		void setupeEnableDRV8353();		//Set pins needed for DRV8353
        bool checkFault();    			//Returns true if fault 
        void clearFaults();             //Clears fault values, writes a clear to the DRV8353
        void readStatus();              //Reads fault statuses
		void enable();					//Enable the DRV8353 through the enable pin
		void disable();					//Disbale the DRV8353 through the enable pin

        //Register commands
        void readControl();             //Read only the control registers
        void readSettings();            //Read only the settings registers
        void updateControl();           //Send controls and update variables
        void updateSettings();          //Send settings and update variables
		drv8353_status1 readStatus1();
		drv8353_status2 readStatus2();

        //DRV8353 control Get/Set functions
        drv8353_ocp_act getOCP_ACT(){return (drv8353_ocp_act)driver_control.OCP_ACT;};
        drv8353_dis_gduv getDIS_GDUV(){return (drv8353_dis_gduv)driver_control.DIS_GDUV;};
        drv8353_otw_reporting getOTW_Reporting(){return (drv8353_otw_reporting)driver_control.OTW_REP;};
        drv8353_pwmmode getPWM_MODE(){return (drv8353_pwmmode)driver_control.PWM_MODE;};
        drv8353_commutation_pwm1mode get1PWM_Commutation(){return (drv8353_commutation_pwm1mode)driver_control.PWM_COM1;};
        drv8353_direction_pwm1mode get1PWM_Direction(){return (drv8353_direction_pwm1mode)driver_control.PWM_DIR1;};
        drv8353_coast getCoast(){return (drv8353_coast)driver_control.COAST;};
        drv8353_brake getBrake(){return (drv8353_brake)driver_control.BRAKE;};
        void setOCP_ACT(drv8353_ocp_act _value){driver_control.OCP_ACT = (uint16_t)_value;};
        void setDIS_GDUV(drv8353_dis_gduv _value){driver_control.DIS_GDUV = (uint16_t)_value;};
        void setOTW_Reporting(drv8353_otw_reporting _value){driver_control.OTW_REP = (uint16_t)_value;};
        void setPWM_MODE(drv8353_pwmmode _value){driver_control.PWM_MODE = (uint16_t)_value;};
        void set1PWM_Commutation(drv8353_commutation_pwm1mode _value){driver_control.PWM_COM1 = (uint16_t)_value;};
        void set1PWM_Direction(drv8353_direction_pwm1mode _value){driver_control.PWM_DIR1 = (uint16_t)_value;};
        void setCoast(drv8353_coast _value){driver_control.COAST = (uint16_t)_value;};
        void setBrake(drv8353_brake _value){driver_control.BRAKE = (uint16_t)_value;};

        //DRV8353 gate drive Get/Set functions 
        drv8353_gdlock getGDHS_Lock(){return (drv8353_gdlock)gate_drive_hs.LOCK;};
        drv8353_gdip getGateCurrentDrivePHS(){return (drv8353_gdip)gate_drive_hs.IDRIVEP_HS;};
        drv8353_gdin getGateCurrentDriveNHS(){return (drv8353_gdin)gate_drive_hs.IDRIVEN_HS;};
        drv8353_cbc getCBC(){return (drv8353_cbc)gate_drive_ls.CBC;};
        drv8353_tdrive getGateDriveTime(){return (drv8353_tdrive)gate_drive_ls.TDRIVE;};
        drv8353_gdip getGateCurrentDrivePLS(){return (drv8353_gdip)gate_drive_ls.IDRIVEP_LS;};
        drv8353_gdin getGateCurrentDriveNLS(){return (drv8353_gdin)gate_drive_ls.IDRIVEN_LS;};
        void setGDHS_Lock(drv8353_gdlock _value){gate_drive_hs.LOCK = (uint16_t)_value;};
        void setGateCurrentDrivePHS(drv8353_gdip _value){gate_drive_hs.IDRIVEP_HS = (uint16_t)_value;};
        void setGateCurrentDriveNHS(drv8353_gdin _value){gate_drive_hs.IDRIVEN_HS = (uint16_t)_value;};
        void setCBC(drv8353_cbc _value){gate_drive_ls.CBC = (uint16_t)_value;};
        void setGateDriveTime(drv8353_tdrive _value){gate_drive_ls.TDRIVE = (uint16_t)_value;};
        void setGateCurrentDrivePLS(drv8353_gdip _value){gate_drive_ls.IDRIVEP_LS = (uint16_t)_value;};
        void setGateCurrentDriveNLS(drv8353_gdin _value){gate_drive_ls.IDRIVEN_LS = (uint16_t)_value;};

        //DRV8353 OCP Control Get/Set functions 
        drv8353_tretry getTRetry(){return (drv8353_tretry)ocp_control.TRETRY;};
        drv8353_dead_time getDeadtime(){return (drv8353_dead_time)ocp_control.DEAD_TIME;};
        drv8353_ocp_mode getOCPMode(){return (drv8353_ocp_mode)ocp_control.OCP_MODE;};
        drv8353_ocp_deg getOCPDeglitch(){return (drv8353_ocp_deg)ocp_control.OCP_DEG;};
        drv8353_vds_level getVDSLevel(){return (drv8353_vds_level)ocp_control.VDS_LVL;};
        void setTRetry(drv8353_tretry _value){ocp_control.TRETRY = (uint16_t)_value;};
        void setDeadtime(drv8353_dead_time _value){ocp_control.DEAD_TIME = (uint16_t)_value;};
        void setOCPMode(drv8353_ocp_mode _value){ocp_control.OCP_MODE = (uint16_t)_value;};
        void setOCPDeglitch(drv8353_ocp_deg _value){ocp_control.OCP_DEG = (uint16_t)_value;};
        void setVDSLevel(drv8353_vds_level _value){ocp_control.VDS_LVL = (uint16_t)_value;};

        //DRV8353 CSA Control Get/Set functions 
        drv8353_csa_fet getCSAFET(){return (drv8353_csa_fet)csa_control.CSA_FET;};
        drv8353_vref_div getVrefDiv(){return (drv8353_vref_div)csa_control.VREF_DIV;};
        drv8353_ls_ref getLSRef(){return (drv8353_ls_ref)csa_control.LS_REF;};
        drv8353_csa_gain getCSAGain(){return (drv8353_csa_gain)csa_control.CSA_GAIN;};
        drv8353_dis_sense getDisSense(){return (drv8353_dis_sense)csa_control.DIS_SEN;};
        drv8353_csa_cal_x getCSACalA(){return (drv8353_csa_cal_x)csa_control.CSA_CAL_A;};
        drv8353_csa_cal_x getCSACalB(){return (drv8353_csa_cal_x)csa_control.CSA_CAL_B;};
        drv8353_csa_cal_x getCSACalC(){return (drv8353_csa_cal_x)csa_control.CSA_CAL_C;};
        drv8353_sen_lvl getSenseLevel(){return (drv8353_sen_lvl)csa_control.SEN_LVL;};
        void setCSAFET(drv8353_csa_fet _value){csa_control.CSA_FET = (uint16_t)_value;};
        void setVrefDiv(drv8353_vref_div _value){csa_control.VREF_DIV = (uint16_t)_value;};
        void setLSRef(drv8353_ls_ref _value){csa_control.LS_REF = (uint16_t)_value;};
        void setCSAGain(drv8353_csa_gain _value){csa_control.CSA_GAIN = (uint16_t)_value;};
        void setDisSense(drv8353_dis_sense _value){csa_control.DIS_SEN = (uint16_t)_value;};
        void setCSACalA(drv8353_csa_cal_x _value){csa_control.CSA_CAL_A = (uint16_t)_value;};
        void setCSACalB(drv8353_csa_cal_x _value){csa_control.CSA_CAL_B = (uint16_t)_value;};
        void setCSACalC(drv8353_csa_cal_x _value){csa_control.CSA_CAL_C = (uint16_t)_value;};
        void setSenseLevel(drv8353_sen_lvl _value){csa_control.SEN_LVL = (uint16_t)_value;};

	private:
		//Pins
		int enable_pin;
		int nfault_pin;

        //Registers
    	drv8353_status1 read_status1;
        drv8353_status2 read_status2;
        drv8353_control driver_control;
        drv8353_gate_drive_hs gate_drive_hs;
        drv8353_gate_drive_ls gate_drive_ls;
        drv8353_ocp_control ocp_control;
        drv8353_csa_control csa_control;

        //SPI interface
		int cs_pin;
		int miso_pin;
		int mosi_pin;
		int sck_pin;
		SPIClass* spi;
		SPISettings settings;
        
        /**
        * Fault checking functions
        */
        bool isFault() { return read_status1.FAULT==0b1;};
	    bool isVDSOvercurrentFault() { return read_status1.VSD_OCP==0b1;};
	    bool isGateDriveFault() { return read_status1.GDF==0b1;};
	    bool isUndervoltageLockoutFault() { return read_status1.UVLO==0b1;};
	    bool isOverTempShutdown() { return read_status1.OTSD==0b1; };
	    bool isVDSOverCurrentHAFault() { return read_status1.VDS_HA==0b1;};
	    bool isVDSOverCurrentLAFault() { return read_status1.VDS_LA==0b1;};
        bool isVDSOverCurrentHBFault() { return read_status1.VDS_HB==0b1;};
	    bool isVDSOverCurrentLBFault() { return read_status1.VDS_LB==0b1;};
        bool isVDSOverCurrentHCFault() { return read_status1.VDS_HC==0b1;};
	    bool isVDSOverCurrentLCFault() { return read_status1.VDS_LC==0b1;};
        bool isOvercurrentSOA() { return read_status2.SA_OC==0b1;};
	    bool isOvercurrentSOB() { return read_status2.SB_OC==0b1;};
	    bool isOvercurrentSOC() { return read_status2.SC_OC==0b1;};
	    bool isOverTempWarning() { return read_status2.OTW==0b1;};
	    bool isGateDischargeUnderVoltage() { return read_status2.GDUV==0b1; };
	    bool isVGSHAFault() { return read_status2.VGS_HA==0b1;};
	    bool isVGSLAFault() { return read_status2.VGS_LA==0b1;};
        bool isVGSHBFault() { return read_status2.VGS_HB==0b1;};
	    bool isVGSLBFault() { return read_status2.VGS_LB==0b1;};
        bool isVGSHCFault() { return read_status2.VGS_HC==0b1;};
	    bool isVGSLCFault() { return read_status2.VGS_LC==0b1;};

		//Individual register checking
        void readStatusRegisters();     //Reads the two status registers from the IC
        void readControlRegister();     //Reads the comtrol register from the IC
        void readGateDriveRegisters();  //Reads the gate drive registers from the IC
        void readOCPRegister();         //Reads the current protection register from the IC
        void readCSARegister();         //Reads the current sense amplifier register from the IC
        void setControlRegister();      //Sets the control register w/ class variable driver_control
        void setGateDriveRegisters();   //Sets the gate registers w/ class variable gate_drive_hs and gate_drive_ls
        void setOCPRegister();          //Sets the OCP register w/ class variable ocp_control
        void setCSARegister();          //Sets the CSA register w/ class variable csa_control

		//SPI functions
		uint16_t readSPI(drv8353_register_adderess addr);					//Read SPI
		uint16_t writeSPI(drv8353_register_adderess addr, uint16_t data);	//Send SPI
	
		//Helper functions 
		//This function attaches the address to the 'unused' bits on each register union.
		uint16_t formatAddress(drv8353_register_adderess _addr, uint16_t _reg, drv8353_spi_rw _rw){return ((((uint8_t)_rw|(uint8_t)_addr) << 11) | ((0x07FF) & _reg));}
};
