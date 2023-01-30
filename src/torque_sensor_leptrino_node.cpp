#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string.h>
#include <pCommon.h>
#include <rs_comm.h>
#include <pComResInternal.h>

#define PRG_VER	"Ver 1.0.0"

typedef struct ST_SystemInfo {
	int com_ok;
} SystemInfo;

void App_Init(std::string port_name);
void App_Close(void);
int GetRcv_to_Cmd( char *rcv, char *prm);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(void);
void GetLimit(void);
void SerialStart(void);
void SerialStop(void);
void get_product_info();
void get_limit(float* limit);
void serial_stop();

SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];

using namespace std;

geometry_msgs::WrenchStamped raw_wrench_msgs_;
geometry_msgs::WrenchStamped past_raw_wrench_msgs_;

float cutoff_coef = 0;
float sensor_offset[6] = {0,0,0,0,0,0};
int offset_count = 0;

bool start_calib_flag = false;
float calib_count=0;
float calib_error[6] = {0,0,0,0,0,0};
ST_R_DATA_GET_F *stForce;
float Limit[6];

void OmegaCallback(const std_msgs::Float64ConstPtr& msg)
{
    cutoff_coef = msg->data;
}

void OffsetCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
	sensor_offset[0] = msg->data[0];
	sensor_offset[1] = msg->data[1];
	sensor_offset[2] = msg->data[2];
	sensor_offset[3] = msg->data[3];
	sensor_offset[4] = msg->data[4];
	sensor_offset[5] = msg->data[5];
}

void calibrate(const std_msgs::BoolConstPtr& msg) {
	if (msg->data) {
		if (!start_calib_flag) {
			ROS_INFO("start force sensor calibration...");
			start_calib_flag = true;
			for(int i=0;i<6;i++) calib_error[i]=0;
			calib_count = 0;
		} else {
			for(int i=0;i<6;i++) {
				calib_error[i] = stForce->ssForce[i] / (calib_count+1) + calib_count*calib_error[i] / (calib_count+1);
			}
			calib_count++;
		}
	} else {
		if (start_calib_flag) {
			ROS_INFO("calibration finished.");
			start_calib_flag = false;
		}
	}
}



int main(int argc, char **argv)
{
  cout << "torque sensor data collector start" << endl;
  ros::init(argc, argv, "torque_sensor_leptrino");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> raw_wrench_pub_;
  raw_wrench_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(n, "raw_wrench", 60));

  ros::Subscriber sub_omega = n.subscribe("leptrino/cutoffcoef", 1, &OmegaCallback);
  ros::Subscriber sub_offset = n.subscribe("leptrino/offset", 1, &OffsetCallback);
  ros::Subscriber sub_calibrate = n.subscribe("leptrino/calibrate", 1, &calibrate);

  ros::Rate loop_rate(500);
  ros::Time time;

  // トルクセンサ用変数
	int i, l = 0, rt = 0;
	int mode_step = 0;
	int AdFlg = 0, EndF = 0;
	long cnt = 0;
	UCHAR strprm[256];
	ST_RES_HEAD *stCmdHead;
	ST_R_GET_INF *stGetInfo;
  	ST_R_LEP_GET_LIMIT *stGetLimit;

	ros::Time offsetStartTime;

	std::string port_name = ros::names::remap("port_name");
	if (port_name == "port_name") pn.getParam("port_name",port_name);
	if (port_name == "port_name") {
		ROS_ERROR("PLEASE DEFINE PORT FOR TORQUE SENSOR!");
		return 0;
	}

	App_Init(port_name);
	if (gSys.com_ok == NG) {
		printf("ComPort Open Fail\n");
		ROS_ERROR("COMPORT OPEN FAIL");
		return 0;
	}
	// 製品情報取得
	get_product_info();
	// 定格取得
	get_limit(Limit);

	SerialStart();
	EndF = 0;
	offsetStartTime = ros::Time::now();
	while (ros::ok())
	{
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			memset(CommRcvBuff,0,sizeof(CommRcvBuff));
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
			}
			if (raw_wrench_pub_->trylock()) {
				raw_wrench_msgs_.wrench.force.x = (( Limit[0] / 10000.0 * (stForce->ssForce[0]-calib_error[0]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.force.x;
				raw_wrench_msgs_.wrench.force.y = (( Limit[1] / 10000.0 * (stForce->ssForce[1]-calib_error[1]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.force.y;
				raw_wrench_msgs_.wrench.force.z = ((-Limit[2] / 10000.0 * (stForce->ssForce[2]-calib_error[2]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.force.z;
				raw_wrench_msgs_.wrench.torque.x = (( Limit[3] / 10000.0 * (stForce->ssForce[3]-calib_error[3]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.torque.x;
				raw_wrench_msgs_.wrench.torque.y = (( Limit[4] / 10000.0 * (stForce->ssForce[4]-calib_error[4]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.torque.y;
				raw_wrench_msgs_.wrench.torque.z = ((-Limit[5] / 10000.0 * (stForce->ssForce[5]-calib_error[5]))) * (1-cutoff_coef) + cutoff_coef * past_raw_wrench_msgs_.wrench.torque.z;
				raw_wrench_msgs_.header.stamp = ros::Time::now();
				raw_wrench_pub_->msg_ = raw_wrench_msgs_;
				raw_wrench_pub_->unlockAndPublish();
				past_raw_wrench_msgs_ = raw_wrench_msgs_;
			}
			for (int i=0;i<6;i++) {
				if (stForce->ssForce[i] > 10000.0 ) {
					ROS_INFO("[CAUTION] EXCEEDED FORCE/TORQUE APPLIED!!");
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	// 終了処理
	printf("torque node ended");
	serial_stop();
	App_Close();

	return 0;
}

// ----------------------------------------------------------------------------------
//	アプリケーション初期化
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Init(std::string port_name)
{
	int rt;
	
	//Commポート初期化
	gSys.com_ok = NG;
	rt = Comm_Open(port_name.c_str());
	if ( rt==OK ) {
		Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
		gSys.com_ok = OK;
	}
}

// ----------------------------------------------------------------------------------
//	アプリケーション終了処理
// ----------------------------------------------------------------------------------
//	引数	: non
//	戻り値	: non
// ----------------------------------------------------------------------------------
void App_Close(void)
{
	printf("Application Close\n");
	
	if ( gSys.com_ok == OK) {
		Comm_Close();
	}
}

/*********************************************************************************
* Function Name  : HST_SendResp
* Description    : データを整形して送信する
* Input          : pucInput 送信データ
*                : 送信データサイズ
* Output         : 
* Return         : 
*********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
	USHORT usCnt;
	UCHAR ucWork;
	UCHAR ucBCC = 0;
	UCHAR *pucWrite = &CommSendBuff[0];
	USHORT usRealSize;
	
	// データ整形 
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_STX;					// STX 
	pucWrite++;
	usRealSize =2;
	
	for (usCnt = 0; usCnt < usSize; usCnt++) {
		ucWork = pucInput[usCnt];
		if (ucWork == CHR_DLE) {			// データが0x10ならば0x10を付加 
			*pucWrite = CHR_DLE;			// DLE付加 
			pucWrite++;						// 書き込み先 
			usRealSize++;					// 実サイズ
			// BCCは計算しない!
		}
		*pucWrite = ucWork;					// データ 
		ucBCC ^= ucWork;					// BCC 
		pucWrite++;							// 書き込み先 
		usRealSize++;						// 実サイズ 
	}
	
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_ETX;					// ETX 
	ucBCC ^= CHR_ETX;						// BCC計算 
	pucWrite++;
	*pucWrite = ucBCC;						// BCC付加 
	usRealSize += 3;
	
	Comm_SendData(&CommSendBuff[0], usRealSize);
	
	return OK;
}

void GetProductInfo(void)
{
	USHORT len;
	
	printf("Get SensorInfo\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_GET_INF;				// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void GetLimit(void) {
	USHORT len;
	
	printf("Get SensorLimit\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_GET_LIMIT;				// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStart(void)
{
	USHORT len;
	
	printf("Start\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_START;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void SerialStop(void)
{
	USHORT len;
	
	printf("Stop\n");
	len = 0x04;								// データ長
	SendBuff[0] = len;						// レングス
	SendBuff[1] = 0xFF;						// センサNo.
	SendBuff[2] = CMD_DATA_STOP;			// コマンド種別
	SendBuff[3] = 0;						// 予備
	
	SendData(SendBuff, len);
}

void get_product_info() {
	int EndF=0;
	GetProductInfo();
	while(1) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			CommRcvBuff[0]=0; 
			
			int rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				ST_R_GET_INF *stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
				printf("Version:%s\n", stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
				printf("SerialNo:%s\n", stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
				printf("Type:%s\n", stGetInfo->scPName);
				printf("\n");
				EndF = 1;
			}
			
		}
		if ( EndF==1 ) break;
	}
}

void get_limit(float *limit) {
	int EndF=0;
	GetLimit();
	while(1) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//受信データ有
			CommRcvBuff[0]=0; 
			int rt = Comm_GetRcvData( CommRcvBuff );
			printf("rt:%d",rt);
			printf("\n");
			if ( rt>0 ) {
				ST_R_LEP_GET_LIMIT *stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
				for(int i=0;i<6;i++) {
					limit[i] = stGetLimit->fLimit[i];
				}
				printf("Limit:Fx:%f,Fy:%f,Fz:%f,Mx:%f,My:%f,Mz:%f",
				limit[0],limit[1],limit[2],limit[3],limit[4],limit[5]);
				printf("\n");
				EndF = 1;
			}
		}
		if ( EndF==1 ) break;
	}
}

void serial_stop() {
	int EndF=0;
	SerialStop();
	ST_RES_HEAD *stCmdHead = (ST_RES_HEAD *)CommRcvBuff;
	if (stCmdHead->ucCmd == CMD_DATA_STOP) {
		printf("Receive Stop Response:");
		int l = stCmdHead->ucLen;
		for (int i=0; i<l; i++) {
			printf("%02x ", CommRcvBuff[i]);
		}
		printf("\n");
		EndF = 1;
	}
}