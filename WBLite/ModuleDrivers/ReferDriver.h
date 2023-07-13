#ifndef REFERDRIVER_H
#define REFERDRIVER_H
#include <main.h>
#include <stdio.h>
#include "UART_LL.h"
/*
	配置菜单
*/
// <<< Use Configuration Wizard in Context Menu >>>

// <o>比赛模式选择
//  <i>比赛模式涉及到裁判系统数据处理方式, 请确保正确选择
//  <0x00=> RMUL
//  <0x01=> RMUC 
#define RF_GAME_MODE 0x00

// <o>最大无间隔接收包数
	//  <i>一段数据中可能包含包的最大数目
	//  <i>Default: 5
	#define RX_PACK_MAX  8

// <c1>启用客户端UI绘制
// <i>使能相关功能
//#define RF_UI_ENABLE
// </c>	
	
//<h> 比赛信息数据
//<i>cmd_id范围: 0x0001 - 0x0005
	// <c1>接收比赛状态数据
	//  <i>cmd_id: 0x0001
	//#define RF_GAME_STATUE
	// </c>
	
	// <c1>接收比赛结果数据
	//  <i>cmd_id: 0x0002
	//#define RF_GAME_RESULT
	// </c>
	
	// <c1>接收比赛机器人血量数据
	//  <i>cmd_id: 0x0003
	#define RF_GAME_HP
	// </c>
	
	// <c1>接收飞镖发射状态
	//  <i>cmd_id: 0x0004
	//#define RF_GAME_DART
	// </c>
	
	// <c1>接收AI赛加成与惩罚状态
	//  <i>cmd_id: 0x0005
	//#define RF_GAME_ICRA
	// </c>	
//</h>

//<h> 场地道具数据
//<i>cmd_id范围: 0x0101 - 0x0105
	// <c1>接收场地事件数据
	//  <i>cmd_id: 0x0101
	//#define RF_FIELD_EVENT
	// </c>
	
	// <c1>接收场地补给站动作标识数据
	//  <i>cmd_id: 0x0102
	//#define RF_FIELD_SUPPLY
	// </c>
	

	// <c1>接收裁判警告数据
	//  <i>cmd_id: 0x0104
	//#define RF_FIELD_WARNING
	// </c>
	
	// <c1>接收飞镖发射口倒计时
	//  <i>cmd_id: 0x0105
	//#define RF_FIELD_DARTTIME
	// </c>
//</h>

//<h> 机器人数据
//<i>cmd_id范围: 0x0201 - 0x020A
	// <c1>接收机器人状态数据
	//  <i>cmd_id: 0x0201
	#define RF_ROBOT_STATUE
	// </c>
	
	// <c1>接收实时功率热量数据
	//  <i>cmd_id: 0x0202
	#define RF_ROBOT_POWERHEAT
	// </c>
	
	// <c1>接收机器人位置数据
	//  <i>cmd_id: 0x0203
	//#define RF_ROBOT_POSITION
	// </c>
	
	// <c1>接收机器人增益数据
	//  <i>cmd_id: 0x0204
	#define RF_ROBOT_BUFFER
	// </c>
	
	// <c1>接收空中机器人能量状态数据
	//  <i>cmd_id: 0x0205 本数据只有控制机器人主控发送
	//#define RF_ROBOT_DRONEENERGY
	// </c>
	
	// <c1>接收伤害状态数据
	//  <i>cmd_id: 0x0206
	#define RF_ROBOT_HURT
	// </c>
	
	// <c1>接收实时射击数据
	//  <i>cmd_id: 0x0207
	#define RF_ROBOT_SHOOT
	// </c>
	
	// <c1>接收子弹剩余发射数
	//  <i>cmd_id: 0x0208 RMUL与RMUC存在差异
	#define RF_ROBOT_BULLET
	// </c>
	
	// <c1>接收机器人 RFID 状态
	//  <i>cmd_id: 0x0209
	#define RF_ROBOT_RFID
	// </c>
	
	// <c1>接收飞镖机器人客户端指令数据
	//  <i>cmd_id: 0x020A
	//#define RF_ROBOT_DARTCMD
	// </c>	
	
	// <c1>接收己方机器人位置坐标
	//  <i>cmd_id: 0x020B RMUC Only
	//#define RF_ROBOT_TEAMPOSITION
	// </c>	
	
	// <c1>接收机器人标记数据
	//  <i>cmd_id: 0x020C RMUC Only
	//#define RF_ROBOT_MARK
	// </c>	
	
//</h>

//<h> 高级交互数据
//<i>cmd_id范围: 0x0301 - 0x0305
	
	// <e>启用机器人间通信
	// <i>cmd_id: 0x0301
	#define RF_ITF_ROBOTDATA 0
	// <o>交互数据段长度
	// <i>Default: 8 Byte
	// <i>Max: 113 Byte
	#define RF_ITF_ROBOTDATA_LEN  8
	#if (RF_ITF_ROBOTDATA==0)
	#undef RF_ITF_ROBOTDATA
	#endif
	// </e>

	
	// <e>启用自定义控制器
	// <i>cmd_id: 0x0302
	#define RF_ITF_CUSTOMCTR 0
	
		// <o>控制器数据段长度
		// <i>Default: 8 Byte
		// <i>Max: 30 Byte
		#define RF_ITF_CUSTOMCTR_LEN  8

		// <c1>接收自定义控制器模拟键鼠信息
		//  <i>cmd_id: 0x0306
		#define RF_ITF_CUSTOMCLIENT
		// </c>
	
	#if (RF_ITF_CUSTOMCTR==0)
	#undef RF_ITF_CUSTOMCTR
	#undef RF_ITF_MAPRADAR
	#endif
	// </e>
	
	
	// <c1>接收客户端小地图交互数据
	//  <i>cmd_id: 0x0303
	//#define RF_ITF_MAPCLIENT
	// </c>

	// <c1>接收图传键鼠信息
	//  <i>cmd_id: 0x0304
	//#define RF_ITF_CLIENT
	// </c>
	
	// <c1>接收客户端小地图信息
	//  <i>cmd_id: 0x0305
	//#define RF_ITF_MAPRADAR
	// </c>
	
	// <c1>接收客户端小地图信息
	//  <i>cmd_id: 0x0307
	//#define RF_ITF_SENTRY
	// </c>
	
//</h>

// <<< end of configuration section >>>

/*
	裁判系统接收结构体
*/
__PACKED_STRUCT robot_referee_status_t
{
	#ifdef RF_GAME_STATUE			//0x0001
	typedef __PACKED_STRUCT 
	{
		uint8_t game_type : 4;
		uint8_t game_progress : 4;
		uint16_t stage_remain_time;
		uint64_t SyncTimeStamp;
	}ext_game_status_t;
	ext_game_status_t ext_game_status;
	#endif
	
	#ifdef RF_GAME_RESULT			//0x0002
	typedef __PACKED_STRUCT
	{
		uint8_t winner;
	}ext_game_result_t;
	ext_game_result_t ext_game_result;
	#endif
	
	#ifdef RF_GAME_HP				//0x0003
	typedef __PACKED_STRUCT
	{
		uint16_t red_1_robot_HP;
		uint16_t red_2_robot_HP; 
		uint16_t red_3_robot_HP; 
		uint16_t red_4_robot_HP; 
		uint16_t red_5_robot_HP; 
		uint16_t red_7_robot_HP; 
		uint16_t red_outpost_HP;
		uint16_t red_base_HP; 
		uint16_t blue_1_robot_HP; 
		uint16_t blue_2_robot_HP; 
		uint16_t blue_3_robot_HP; 
		uint16_t blue_4_robot_HP; 
		uint16_t blue_5_robot_HP; 
		uint16_t blue_7_robot_HP; 
		uint16_t blue_outpost_HP;
		uint16_t blue_base_HP;
	}ext_game_robot_HP_t;
	ext_game_robot_HP_t ext_game_robot_HP;
	#endif
	
	#ifdef RF_GAME_DART				//0x0004
	/*官方没给*/
	#endif
	
	#ifdef RF_GAME_ICRA				//0x0005
	typedef __PACKED_STRUCT
	{
		uint8_t F1_zone_status:1;
		uint8_t F1_zone_buff_debuff_status:3; 
		uint8_t F2_zone_status:1;
		uint8_t F2_zone_buff_debuff_status:3; 
		uint8_t F3_zone_status:1;
		uint8_t F3_zone_buff_debuff_status:3; 
		uint8_t F4_zone_status:1;
		uint8_t F4_zone_buff_debuff_status:3; 
		uint8_t F5_zone_status:1;
		uint8_t F5_zone_buff_debuff_status:3; 
		uint8_t F6_zone_status:1;
		uint8_t F6_zone_buff_debuff_status:3;
		uint16_t red1_bullet_left;
		uint16_t red2_bullet_left;
		uint16_t blue1_bullet_left;
		uint16_t blue2_bullet_left;
		uint8_t lurk_mode;
		uint8_t res;
	}ext_ICRA_buff_debuff_zone_and_lurk_status_t;
	ext_ICRA_buff_debuff_zone_and_lurk_status_t ext_ICRA_buff_debuff_zone_and_lurk_status;
	#endif
	
	#ifdef RF_FIELD_EVENT			//0x0101
	typedef __PACKED_STRUCT
	{
		uint32_t event_type;
	}ext_event_data_t;
	ext_event_data_t ext_event_data;
	#endif
	
	#ifdef RF_FIELD_SUPPLY			//0x0102
	typedef __PACKED_STRUCT
	{
		uint8_t supply_projectile_id; 
		uint8_t supply_robot_id; 
		uint8_t supply_projectile_step; 
		uint8_t supply_projectile_num;
	}ext_supply_projectile_action_t;
	ext_supply_projectile_action_t ext_supply_projectile_action;
	#endif
	
	#ifdef RF_FIELD_WARNING			//0x0104
		#if (RF_GAME_MODE==0x01)//RMUC
		typedef __PACKED_STRUCT
		{
			uint8_t level;
			uint8_t offending_robot_id;
		}referee_warning_t;
		referee_warning_t referee_warning;
		#elif (RF_GAME_MODE==0x00)//RMUL
		typedef __PACKED_STRUCT
		{
			uint8_t level;
			uint8_t foul_robot_id; 
		}ext_referee_warning_t;
		ext_referee_warning_t ext_referee_warning;
		#endif
	#endif
		
	#ifdef RF_FIELD_DARTTIME		//0x0105
	typedef __PACKED_STRUCT
	{
		uint8_t dart_remaining_time;
	}ext_dart_remaining_time_t;
	ext_dart_remaining_time_t ext_dart_remaining_time;
	#endif
	
	#ifdef RF_ROBOT_STATUE			//0x0201
	typedef __PACKED_STRUCT
	{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t remain_HP;
		uint16_t max_HP;
		uint16_t shooter_id1_17mm_cooling_rate;
		uint16_t shooter_id1_17mm_cooling_limit;
		uint16_t shooter_id1_17mm_speed_limit;
		uint16_t shooter_id2_17mm_cooling_rate;
		uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
		uint16_t shooter_id1_42mm_cooling_rate;
		uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;
		uint16_t chassis_power_limit;
		uint8_t mains_power_gimbal_output : 1;
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
	}ext_game_robot_status_t;
	ext_game_robot_status_t ext_game_robot_status;
	#endif
	
	#ifdef RF_ROBOT_POWERHEAT		//0x0202
	typedef __PACKED_STRUCT
	{
		uint16_t chassis_volt; 
		uint16_t chassis_current; 
		float chassis_power; 
		uint16_t chassis_power_buffer; 
		uint16_t shooter_id1_17mm_cooling_heat;
		uint16_t shooter_id2_17mm_cooling_heat;
		uint16_t shooter_id1_42mm_cooling_heat;
	}ext_power_heat_data_t;	
	ext_power_heat_data_t ext_power_heat_data;
	#endif
	
	#ifdef RF_ROBOT_POSITION		//0x0203
	typedef __PACKED_STRUCT
	{
		float x;
		float y;
		float z;
		float yaw;
	}ext_game_robot_pos_t;
	ext_game_robot_pos_t ext_game_robot_pos;
	#endif
	
	#ifdef RF_ROBOT_BUFFER			//0x0204
	typedef __PACKED_STRUCT
	{
		uint8_t power_rune_buff;
	}ext_buff_t;
	ext_buff_t ext_buff;
	#endif
	
	#ifdef RF_ROBOT_DRONEENERGY		//0x0205
	typedef __PACKED_STRUCT			
	{
		uint8_t attack_time;
	}aerial_robot_energy_t;
	aerial_robot_energy_t aerial_robot_energy;
	#endif
	
	#ifdef RF_ROBOT_HURT			//0x0206
	typedef __PACKED_STRUCT			
	{
		uint8_t armor_id : 4;
		uint8_t hurt_type : 4;
	}ext_robot_hurt_t;
	ext_robot_hurt_t ext_robot_hurt;
	#endif
	
	#ifdef RF_ROBOT_SHOOT			//0x0207
	typedef __PACKED_STRUCT			
	{
		uint8_t bullet_type;
		uint8_t shooter_id;
		uint8_t bullet_freq;
		float bullet_speed;
	}ext_shoot_data_t;
	ext_shoot_data_t ext_shoot_data;
	#endif
	
	#ifdef RF_ROBOT_BULLET			//0x0208
		#if (RF_GAME_MODE==0x01)//RMUC
		typedef __PACKED_STRUCT
		{
			uint16_t projectile_allowance_17mm;
			uint16_t projectile_allowance_42mm;
			uint16_t remaining_gold_coin;
		}projectile_allowance_t;
		projectile_allowance_t projectile_allowance;
		#elif (RF_GAME_MODE==0x00)//RMUL
		typedef __PACKED_STRUCT
		{
			uint16_t bullet_remaining_num_17mm;
			uint16_t bullet_remaining_num_42mm;
			uint16_t coin_remaining_num;
		}ext_bullet_remaining_t;
		ext_bullet_remaining_t ext_bullet_remaining;
		#endif
	#endif
		
	#ifdef RF_ROBOT_RFID			//0x0209
	typedef __PACKED_STRUCT
	{
		uint32_t rfid_status;
	}ext_rfid_status_t;
	ext_rfid_status_t ext_rfid_status;
	#endif
	
	#ifdef RF_ROBOT_DARTCMD			//0x020A
	typedef __PACKED_STRUCT
	{
		uint8_t dart_launch_opening_status;
		uint8_t dart_attack_target;
		uint16_t target_change_time;
		uint16_t operate_launch_cmd_time;
	}ext_dart_client_cmd_t;	
	ext_dart_client_cmd_t ext_dart_client_cmd;
	#endif
	
	#if (RF_GAME_MODE == 0x01)//RMUC
		#ifdef RF_ROBOT_TEAMPOSITION//0x020B
		typedef __PACKED_STRUCT
		{
			float hero_x;
			float hero_y;
			float engineer_x;
			float engineer_y;
			float standard_3_x;
			float standard_3_y;
			float standard_4_x;
			float standard_4_y;
			float standard_5_x;
			float standard_5_y;
		}ground_robot_position_t;
		ground_robot_position_t ground_robot_position;
		#endif
		#ifdef RF_ROBOT_MARK		//0x020C
		typedef __PACKED_STRUCT
		{
			uint8_t mark_hero_progress;
			uint8_t mark_engineer_progress;
			uint8_t mark_standard_3_progress;
			uint8_t mark_standard_4_progress;
			uint8_t mark_standard_5_progress;
			uint8_t mark_sentry_progress;
		}radar_mark_data_t;
		radar_mark_data_t radar_mark_data;
		#endif
	#endif
		
	#ifdef RF_ITF_ROBOTDATA			//0x0301
	typedef __PACKED_STRUCT
	{
		uint8_t data[RF_ITF_ROBOTDATA_LEN];
	}robot_interactive_data_t;
	robot_interactive_data_t robot_interactive_data;
	#endif
	
	#ifdef RF_ITF_CUSTOMCTR			//0x0302
	typedef __PACKED_STRUCT
	{
		uint8_t data[RF_ITF_CUSTOMCTR_LEN];
	}custom_robot_data_t;
	custom_robot_data_t custom_robot_data;
	#endif
	
	#ifdef RF_ITF_MAPCLIENT			//0x0303
	typedef __PACKED_STRUCT
	{
		float target_position_x;
		float target_position_y;
		float target_position_z;
		uint8_t commd_keyboard;
		uint16_t target_robot_ID;
	}ext_robot_mapcommand_t;
	ext_robot_mapcommand_t ext_robot_mapcommand;
	#endif
	
	#ifdef RF_ITF_CLIENT			//0x0304
	typedef __PACKED_STRUCT
	{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		int8_t left_button_down;
		int8_t right_button_down;
		uint16_t keyboard_value;
		uint16_t reserved;
	}ext_robot_command_t;
	ext_robot_command_t ext_robot_command;
	#endif
	
	#ifdef RF_ITF_MAPRADAR			//0x0305
	typedef __PACKED_STRUCT
	{
		uint16_t target_robot_ID;
		float target_position_x;
		float target_position_y;
	}ext_client_map_command_t;
	ext_client_map_command_t ext_client_map_command;
	#endif
	
	#if (RF_GAME_MODE == 1)//RMUC
		#ifdef RF_ITF_CUSTOMCLIENT	//0x0306
		typedef __PACKED_STRUCT
		{
			uint16_t key_value;
			uint16_t x_position:12;
			uint16_t mouse_left:4;
			uint16_t y_position:12;
			uint16_t mouse_right:4;
			uint16_t reserved;
		}custom_client_data_t;
		custom_client_data_t custom_client_data;
		#endif
		#ifdef RF_ITF_SENTRY		//0x0307
		typedef __PACKED_STRUCT
		{
			uint8_t intention;
			uint16_t start_position_x;
			uint16_t start_position_y;
			int8_t delta_x[49];
			int8_t delta_y[49];
		}map_sentry_data_t;
		map_sentry_data_t map_sentry_data;
		#endif
	#endif
	
};


typedef struct
{
	uint8_t *data;
	uint16_t Size;
}ref_msg_t;

typedef struct
{
	uint8_t  packnum;
	uint32_t packhead[RX_PACK_MAX];
	uint32_t dstaddr[RX_PACK_MAX];
	uint16_t packlen[RX_PACK_MAX];
	uint16_t byteoff;
}ref_dma_t;

#ifdef RF_UI_ENABLE

/*组ID*/
/*以防有人整花活手动限制这么多*/
/*建议一个图层一个组*/
enum eUI_GROUP
{
    UI_GROUP_0 = 0,
	UI_GROUP_1 = 1,
	UI_GROUP_2 = 2,
    UI_GROUP_3 = 3,
	UI_GROUP_4 = 4,
	UI_GROUP_5 = 5,
    UI_GROUP_6 = 6,
	UI_GROUP_7 = 7,
	UI_GROUP_8 = 8,
	UI_GROUP_9 = 9
};

/*组内图形数量ID*/
enum eUI_GROUP_NUM
{
    UI_GROUP_GNUM_1 = 1,
	UI_GROUP_GNUM_2 = 2,
	UI_GROUP_GNUM_5 = 5,
	UI_GROUP_GNUM_7 = 7,
};

/*组内图形ID*/
/*从1开始方便后续对齐*/
enum eUI_GROUP_GID
{
    UI_GROUP_GID_0 = 1,
	UI_GROUP_GID_1,
	UI_GROUP_GID_2,
	UI_GROUP_GID_3,
	UI_GROUP_GID_4,
	UI_GROUP_GID_5,
	UI_GROUP_GID_6
};

enum eUI_LAYER_ID
{
    UI_LAYER_0=0,
	UI_LAYER_1,
	UI_LAYER_2,
	UI_LAYER_3,
	UI_LAYER_4,
	UI_LAYER_5,
	UI_LAYER_6,
	UI_LAYER_7,
	UI_LAYER_8,
	UI_LAYER_9,
	UI_LAYER_ALL
};

enum eUI_GRAPHE_OPERATE
{
    UI_GRAPHE_OPERATE_NOP=0,
	UI_GRAPHE_OPERATE_ADD,
	UI_GRAPHE_OPERATE_CHANGE,
	UI_GRAPHE_OPERATE_DELETE
};

enum eUI_COLOR
{
    UI_COLOR_RED_BLUE=0,
	UI_COLOR_YELLOW,
	UI_COLOR_GREEN,
	UI_COLOR_ORANGE,
	UI_COLOR_VIOLETRED,
	UI_COLOR_PINK,
	UI_COLOR_BLUEGREEN,
	UI_COLOR_BLACK,
	UI_COLOR_WHITE
};

/*图形配置结构体*/
typedef __PACKED_STRUCT
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
	
} graphic_data_struct_t;

/*
GroupName: 	图形组名称
GraphNum:	图形组中图形数目
GroupLayer:	图形组中图形所在层
pbuf:		图形缓冲区, 长度= 15 + 15*图形组中图形数目
*/
typedef struct
{
	eUI_GROUP		GroupName;
	eUI_GROUP_NUM	GraphNum;
	eUI_LAYER_ID 		GroupLayer;
	uint8_t *pbuf;
}group_config_t;

class cUI
{
	protected:
		
	group_config_t* pGroups[10]={0};
	uint16_t RobotID = 0;
	
	public:
	uint8_t  GroupNum = 0;
	uint8_t	 UITxSeq = 0;
	
	uint8_t SetRobotID(uint16_t RobotID);
	uint8_t LayerDelete(uint8_t *pbuf, eUI_LAYER_ID Layer);
	uint8_t GraphGroupReg(group_config_t* pGroup);	
	uint8_t GraphGroupDelete(eUI_GROUP GroupName);
	uint8_t GrapgGroupPack(eUI_GROUP GroupName,uint8_t* puibuf);
	
	uint8_t GraphGroupUpdateLine		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x,uint16_t start_y, uint16_t end_x, uint16_t end_y);

	uint8_t GraphGroupUpdateRectangle	(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y);
	
	uint8_t GraphGroupUpdateCircle		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t radius);
	
	uint8_t GraphGroupUpdateEllipse	(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);

	uint8_t GraphGroupUpdateArc		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y);


	
	uint8_t CharacterUpdate			( uint8_t* pbuf, eUI_LAYER_ID Layer, char *name, eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint8_t size, 
										uint16_t start_x, uint16_t start_y, const char *str, ...);
	
};

#endif
/*
	裁判系统类
*/
class cRef : public cUART
{
	protected:
	cDMA M2M;
	public:
	#ifdef RF_UI_ENABLE
	cUI DrawUI;
	#endif
	ref_msg_t Ref_Msg ={0};
	ref_dma_t DMA_Msg ={0};
	
	uint32_t error_packp = 0;
	uint32_t error_pack = 0;
	uint8_t IsRefOffline = 1;
	
	robot_referee_status_t *robot_referee_status;
	
	uint8_t  RefDataConfig(robot_referee_status_t* pbuf);
	uint8_t  M2M_Init(DMA_TypeDef *DMAx,uint32_t DMA_CH);
	uint32_t M2M_CheckDst(uint16_t CMD);
	uint8_t  M2M_Update(uint8_t* data,uint16_t len);
	void 	 M2M_Transmit(uint8_t packid);
};
#endif