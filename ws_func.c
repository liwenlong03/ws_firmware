/**
 *  \file new 1
 *  \brief Brief
 */

#include "ws_func.h"
#include "ethernet.h"
#include "eeprom.h"

// input - cmd
uint8_t ws_i_cmd_valve_on = 0;
uint8_t ws_i_cmd_bypass = 0;
uint8_t ws_i_cmd_reset = 0;

// input - properties
double ws_i_warning_flow = 11.4;
double ws_i_fault_flow = 7.615;
double ws_i_stablization_delay = 0;
double ws_i_startup_leak = 0;
uint32_t ws_i_cmd_leak_response = 1;				    // 0: slow, 1: normal, 2: fast


// internal - default paras out of factory
uint8_t isThereNewCtrParas = 0;
double ws_def_warning_flow = 11.44;				    // 11.4 LPM
double ws_def_fault_flow = 7.62;					    // 7.6 LPM
double ws_def_stablization_delay = 2;			    // 2 s
double ws_def_startup_leak = 3.8;				    // 3.8 LPM
uint8_t ws_def_cmd_leak_response = 1;			    // 1 for normal

// internal - parameters/attributes
uint8_t ws_attr_err_flag = 0;						// Set by leak detection, reset by reset cmd from remote
uint32_t ws_valveon_timestamp = 0;
uint8_t ws_flag_after_startup_delay = 0;
double ws_i_startup_leak_in_flowvolme_for_detection = 0;	// in our algorithm, such detection is campare flow volume instead of flowrate.
#define qv_buf_size 100
double qv_flowrate_1 = 0;			                // volume flowrate in l/min
double qv_flowrate_2 = 0;			                // volume flowrate in l/min
double qv_flowrate_buf_1[qv_buf_size] = {0};
double qv_flowrate_buf_2[qv_buf_size] = {0};

// internal - coefficient for flowrate calculation.
static double kf_coff_freq_output = 0.1843;              // for DIN15, kf=0.1843 (l/min)/f
static double q0_axis_intercept = 0;	            // For DIN15, q0 = -0.2

// internal - threshold for leakage detection
double threshold_deviation_aver = 3;			// the deviation of 2 sensors' average
//static double threshold_deviation_rate = 0;			// the changing rate of 2 sensor aver's dev
double threshold_deviation_int = 0.05;			// the integral of 2 sensors' dev


// output - feedback
double ws_o_current_flow = 0;
uint8_t ws_o_is_valve_on = 0;
uint8_t ws_o_is_Bypassed = 0;
uint8_t ws_o_is_leak_detected = 0;
uint8_t ws_o_is_flow_warning = 0;
uint8_t ws_o_is_flow_fault = 0;
uint8_t ws_o_is_flow_ok = 0;
uint8_t ws_o_inflow_status_index = 0;				// 2: 0k, 1:warning, 0: fault
uint8_t ws_o_is_oktoweld = 0;
uint8_t ws_o_is_minflow = 0;		                //????
uint8_t ws_o_is_caploss = 0;                        // the difference with leak_detection
uint8_t ws_o_status_index = 0;

double flow_dev_aver = 0;
double flow_dev_int = 0;		// integral of dev in unit of Litre
double flow_aver_1 = 0, flow_aver_2 = 0;


// private function pre-declaration.

// private functions definition.
void ws_read_flowsensor1 ()
{
	uint16_t ws_flow_sensor_freq = 0;	// sensor pulse output freq in Hz
    
    // update the value of ws_flow_sensor_freq of snesor 1.
	if (alive_monitor_1.AliveFlag == 1) {
		ws_flow_sensor_freq = fls_1.pulse_freq;
	}
	else {
		ws_flow_sensor_freq = 0;
	}

    // moving average required here?
	qv_flowrate_1 = kf_coff_freq_output * ws_flow_sensor_freq + q0_axis_intercept;
}

void ws_read_newctrlparas ()
{
	if (isThereNewCtrParas !=0) {
		isThereNewCtrParas = 0;

		EEPROMProgram ((uint32_t*)&ws_i_warning_flow, EEPROMAddrFromBlock(1), 8);
		EEPROMProgram ((uint32_t*)&ws_i_fault_flow, EEPROMAddrFromBlock(1)+8, 8);
		EEPROMProgram ((uint32_t*)&ws_i_cmd_leak_response, EEPROMAddrFromBlock(1)+16, 4);
		EEPROMProgram ((uint32_t*)&ws_i_stablization_delay, EEPROMAddrFromBlock(1)+20, 8);
		EEPROMProgram ((uint32_t*)&ws_i_startup_leak, EEPROMAddrFromBlock(1)+28, 8);

		// update new startup leak detection threshold.
		ws_i_startup_leak_in_flowvolme_for_detection = ws_i_startup_leak/60*ws_i_stablization_delay;
	}
}

void ws_handle_error_state()
{
	// once error flag is set, only clear it by reset cmd.
	// in the meanwhile, reset the reset cmd.
	// this will not work with EIP!!.
	if (ws_i_cmd_reset == 1 && ws_attr_err_flag==1) {
		ws_attr_err_flag = 0;
		ws_i_cmd_reset = 0;
	}
}

void ws_read_flowsensor2 ()
{
	uint16_t ws_flow_sensor_freq = 0;	// sensor pulse output freq in Hz
    
    // update the value of ws_flow_sensor_freq of sensor 2.
	if (alive_monitor_2.AliveFlag == 1) {
		ws_flow_sensor_freq = fls_2.pulse_freq;
	}
	else {
		ws_flow_sensor_freq = 0;
	}
    
  	// moving average required here?  
	qv_flowrate_2 = kf_coff_freq_output * ws_flow_sensor_freq + q0_axis_intercept;
}

// read control parameters from Eth.
void ws_read_eth_input()
{
    // the required ctl paras include
    // turn on bypass
    // Turnon valve
	ws_i_cmd_valve_on = EIPS_MSG_Input.data.cmd_valve_ctr;
}

void ws_update_eth_output ()
{
	EIPS_MSG_Output.data.nFlowrate= (uint8_t)(qv_flowrate_1 *10);       // feedback flowrate in 10*l/min.
	EIPS_MSG_Output.data.isValveOn = ws_o_is_valve_on;
	EIPS_MSG_Output.data.isLeakDetected = ws_o_is_leak_detected;
}

// currently http input and output are updated directly in http SSI and CGI routines.
// this approach could be changed in future for more flexibility such as allowing other configuration source through Telnet.
// void ws_read_http_input () {}
// void ws_update_http_output () {}

void ws_bypass_control ()
{
	ws_o_is_Bypassed = (ws_i_cmd_bypass == 0)?0:1;
}


// how to handle eip and http are both in work??
void ws_valve_control ()
{
	// TBD: shall AND process result here to finally decide whether to open/close valve.
if (ws_attr_err_flag == 0) {
	// only response to valve control when ws is not in err state
	if (ws_i_cmd_valve_on != 0) {

		if (ws_o_is_valve_on == 0) {
			// only set when previously valve is off.
			ws_valveon_timestamp = Time_GetMs();
			ws_flag_after_startup_delay = 0;
		}
		GPIO_TagWrite(GPIOTag_VALVE_CMD, 1);
		ws_o_is_valve_on = 1;

	}
	else {
		GPIO_TagWrite(GPIOTag_VALVE_CMD, 0);
		ws_o_is_valve_on = 0;
	}
}
else {
	//if in err state, keep the valve off
	GPIO_TagWrite(GPIOTag_VALVE_CMD, 0);
	ws_o_is_valve_on = 0;
}
}

// determine whether in startup period.
void ws_startup_detecion()
{
	if (ws_flag_after_startup_delay == 0 && ws_o_is_valve_on==1) {
		if ( (Time_GetMs() - ws_valveon_timestamp) > (ws_i_stablization_delay*1000) ) {
			ws_flag_after_startup_delay = 1;
		}
	}
}


// detect the flow inlet
// not sure about this point. does it make sense to have only sensor involved to detect supply flow?
// besides, does this detection require smooth flowrate after filtering.
void ws_flowrate_detect_flowin ()
{
    // detect the leakage.
    
    // firstly identify the startup and make some detection accordingly.
    

	// detect the input flowrate status
	// and update flow status indication output accordingly.
    // this detection has only sensor 1 value involved.
	//if (qv_flowrate_1 > ws_i_warning_flow) {
	if (qv_flowrate_1 > ws_i_warning_flow || ws_i_cmd_bypass == 1) {
		//if bypassed, or flow is ok, set of status monitor to OK.
		ws_o_is_oktoweld = 1;
		ws_o_is_minflow = 1;

		ws_o_is_flow_ok = 1;
		ws_o_is_flow_warning = 0;
		ws_o_is_flow_fault = 0;
		ws_o_inflow_status_index = 2;
	}
	else if (qv_flowrate_1 > ws_i_fault_flow) {
		ws_o_is_oktoweld = 1;
		ws_o_is_minflow = 0;

		ws_o_is_flow_ok = 0;
		ws_o_is_flow_warning = 1;
		ws_o_is_flow_fault = 0;
		ws_o_inflow_status_index = 1;
	}
	else {
		ws_o_is_oktoweld = 0;
		ws_o_is_minflow = 0;

		ws_o_is_flow_ok = 0;
		ws_o_is_flow_warning = 0;
		ws_o_is_flow_fault = 1;
		ws_o_inflow_status_index = 0;
	}
}

// calculate and check whether there is a leakage
// when bypassed, calculation will be still done.
void ws_flowrate_detect_leakage ()
{
    uint16_t i = qv_buf_size - 1;
	//uint16_t order = qv_buf_size;

    // for effiency, calculate moving average in advance of buffer update.
	flow_aver_1 = (qv_flowrate_buf_1[0] - qv_flowrate_buf_1[qv_buf_size-1]) / qv_buf_size + flow_aver_1;
	flow_aver_2 = (qv_flowrate_buf_2[0] - qv_flowrate_buf_2[qv_buf_size-1]) / qv_buf_size + flow_aver_2;
	flow_dev_aver = flow_aver_1 - flow_aver_2;

	// calculate the intergal of dev of buffered samples.
	// assume that flowrate was sampled every 5ms.
	// flow_dev_int is in unit of Litre.
	flow_dev_int = flow_dev_int  + ( (qv_flowrate_1 - qv_flowrate_2) - (qv_flowrate_buf_1[qv_buf_size-1] - qv_flowrate_buf_2[qv_buf_size-1]) ) /12000;

	// update the data buffer.
    for (i = qv_buf_size - 1; i > 0; i--) {

        qv_flowrate_buf_1[i] = qv_flowrate_buf_1[i - 1];
        qv_flowrate_buf_2[i] = qv_flowrate_buf_2[i - 1];
    }
    qv_flowrate_buf_1[0] = qv_flowrate_1;
    qv_flowrate_buf_2[0] = qv_flowrate_2;
    
    // check whether leak condition is met.
    // report leakage when measurement result meet the predefined condition.
    // in case of software by pass, still record/report the leak detection result, but without setting err flag.
    // detecion will only be active after startup_delay.
    if (ws_flag_after_startup_delay == 0) {
    	// only detect startup leak detection for cap loss when pressure applied in the pipe
    	if(flow_dev_int > ws_i_startup_leak_in_flowvolme_for_detection) {
    		ws_o_is_caploss = 1;
    		ws_o_is_leak_detected = 1;
    		return;
    	}
    }

    if (flow_dev_aver > threshold_deviation_aver && flow_dev_int > threshold_deviation_int) {
    	ws_o_is_leak_detected = 1;

    	if (ws_o_is_Bypassed == 0) {
    		ws_attr_err_flag = 1;		// set error flag here. Once it's set, it will be reset only by reset cmd.
    	}
    }
    else {
    	ws_o_is_leak_detected = 0;
    }
}

// exported functions
void ws_init ()
{
	// flow sensor is inited in flowsensor.c

	// init valve control port
	GPIO_TagConfigProperties(GPIOTag_VALVE_CMD, GPIO_SET_OUT_PUSHPULL, GPIO_SPD_MID);
    
    // init ws related variables.
	//read values from eeprom
	EEPROMRead ((uint32_t*)&ws_i_warning_flow, EEPROMAddrFromBlock(1), 8);
	EEPROMRead ((uint32_t*)&ws_i_fault_flow, EEPROMAddrFromBlock(1)+8, 8);
	EEPROMRead ((uint32_t*)&ws_i_cmd_leak_response, EEPROMAddrFromBlock(1)+16, 4);
	EEPROMRead ((uint32_t*)&ws_i_stablization_delay, EEPROMAddrFromBlock(1)+20, 8);
	EEPROMRead ((uint32_t*)&ws_i_startup_leak, EEPROMAddrFromBlock(1)+28, 8);

	// start up detection of cap loss is done by checking average of (flowin-flowout)
	// a smaller ws_i_startup_leak means s sensitive detection. (here the value's meansing is actually the flowrate of leak flow through cap)
	// the most non sensitive detection means all deviation of flowin-flowout are caused by delay.
	ws_i_startup_leak_in_flowvolme_for_detection = ws_i_startup_leak/60*ws_i_stablization_delay;
}

void ws_status_update ()
{
	if (ws_o_is_valve_on == 0) {
		ws_o_status_index = ws_valve_off;
		return;
	}

	if (ws_o_is_valve_on !=0 && qv_flowrate_1 <0.6) {
		ws_o_status_index = ws_valve_on_but_flowis0;
		return;
	}

	if (ws_o_is_leak_detected == 1) {
		ws_o_status_index = ws_leak_detected;
		return;
	}

	if (ws_o_is_valve_on ==1 && ws_o_is_flow_ok==1 && ws_o_is_leak_detected==0) {
		ws_o_status_index = ws_ok_to_weld;
		return;
	}
}

// no method in this fun is subject to the risk of being blocked.
// this fun should be called by main loop periodically.
void ws_process ()
{
	// read cmd/paras input
	// ws_read_ctl_parameters();
	ws_read_newctrlparas();
	ws_handle_error_state();

	// read the setting of bypass
	ws_bypass_control();

	// read sensor values
	ws_read_flowsensor1();
	ws_read_flowsensor2();

	// process sensor values and set properties accordingly.
	ws_flowrate_detect_flowin();
	ws_flowrate_detect_leakage();

	// control actuator.
	ws_valve_control();
	ws_startup_detecion();

	// update output in ETH.
	ws_update_eth_output();

	// update status bt a index. this will be used by http routine.
	ws_status_update();

}
