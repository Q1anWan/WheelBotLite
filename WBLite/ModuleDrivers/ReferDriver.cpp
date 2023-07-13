#include "ReferDriver.h"
#include "stdarg.h"
#include "CRC16.h"
#include "CRC8.h"
/*
	整体思路
	结构体直接进行拷贝
	LSB
	1.核验包头数据确定数据有效性
	2.确定命令ID
	3.内存拷贝

*/
uint8_t  cRef::RefDataConfig(robot_referee_status_t* pbuf)
{
	if(pbuf==0)
	{return 1;}
	this->robot_referee_status = pbuf;
	return 0;
}

uint8_t  cRef::M2M_Init(DMA_TypeDef *DMAx,uint32_t DMA_CH)
{
	this->M2M.Config(DMAx,DMA_CH);
	this->M2M.ClearFlag_TC();
	this->M2M.ClearFlag_HT();
	this->M2M.ClearFlag_TE();
	LL_DMA_EnableIT_TC(this->M2M.DMAx,this->M2M.CH);
	return 0;
}

uint8_t cRef::M2M_Update(uint8_t* data,uint16_t len)
{
	/*重置有效包数*/
	this->DMA_Msg.packnum = 0;	
	/*动态更新的指针*/
	uint8_t *pdata_process = data;
	
	/*包解析*/
	while(pdata_process[0] == 0xA5)
	{
		/*查询存储区地址*/
		this->DMA_Msg.dstaddr[this->DMA_Msg.packnum] = this->M2M_CheckDst( (uint16_t)pdata_process[6]<<8 |  (uint16_t)pdata_process[5] );
		/*获取本包长度*/
		this->DMA_Msg.packlen[this->DMA_Msg.packnum] = (uint16_t)pdata_process[2]<<8  |  (uint16_t)pdata_process[1];
		/*判断本包是否被需要*/
		if(this->DMA_Msg.dstaddr[this->DMA_Msg.packnum]==0)
		{	
			/*找不到对应结构体的包被丢弃*/
			pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
			continue;
		}
		/*校验本包CRC16*/
		if( cal_crc16_table(pdata_process, this->DMA_Msg.packlen[this->DMA_Msg.packnum]+7 ) != ( (uint16_t)pdata_process[this->DMA_Msg.packlen[this->DMA_Msg.packnum]+8]<<8 | (uint16_t)pdata_process[this->DMA_Msg.packlen[this->DMA_Msg.packnum]+7] ) )
		{
			/*CRC16错误的包被丢弃*/
			pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
			this->error_packp = (uint32_t)this->DMA_Msg.dstaddr[this->DMA_Msg.packnum];
			this->error_pack++;
			continue;
		}
		/*计算本包数据位指针*/
		this->DMA_Msg.packhead[this->DMA_Msg.packnum]	= (uint32_t)pdata_process + 7;
		/*更新指针位置*/
		pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
		/*更新有效包数*/
		this->DMA_Msg.packnum++;
		/*检查是否本次解析结束*/
		if((uint32_t)(pdata_process - data)>=len)
		{
			break;
		}
	}
	return 0;
}

void cRef::M2M_Transmit(uint8_t packid)
{
		/*msg[packnum-1]发送*/
		LL_DMA_SetM2MSrcAddress(this->M2M.DMAx,this->M2M.CH,this->DMA_Msg.packhead[packid]);
		LL_DMA_SetM2MDstAddress(this->M2M.DMAx,this->M2M.CH,this->DMA_Msg.dstaddr[packid]);
		LL_DMA_SetDataLength(this->M2M.DMAx,this->M2M.CH, this->DMA_Msg.packlen[packid]);
		this->M2M.EnableCH();
}

/*
	O(1)复杂度内存地址提取
*/
uint32_t cRef::M2M_CheckDst(uint16_t CMD)
{
	switch(CMD)
	{
		#ifdef RF_GAME_STATUE			//0x0001
		case 0x0001:
			return (uint32_t)&this->robot_referee_status->ext_game_status;
		break;
		#endif
		#ifdef RF_GAME_RESULT			//0x0002
		case 0x0002:
			return (uint32_t)&this->robot_referee_status->ext_game_result;
		break;
		#endif	
		#ifdef RF_GAME_HP				//0x0003
		case 0x0003:
			return (uint32_t)&this->robot_referee_status->ext_game_robot_HP;
		break;
		#endif
		#ifdef RF_GAME_DART				//0x0004
		case 0x0004:
			return 0;
		break;
		#endif
		#ifdef RF_GAME_ICRA				//0x0005
		case 0x0005:
			return (uint32_t)&this->robot_referee_status->ext_ICRA_buff_debuff_zone_and_lurk_status;
		break;
		#endif
		
		
		#ifdef RF_FIELD_EVENT			//0x0101
		case 0x0101:
			return (uint32_t)&this->robot_referee_status->ext_event_data;
		break;
		#endif
		#ifdef RF_FIELD_SUPPLY			//0x0102
		case 0x0102:
			return (uint32_t)&this->robot_referee_status->ext_supply_projectile_action;
		break;
		#endif
		#ifdef RF_FIELD_WARNING			//0x0104
		case 0x0104:
			#if (RF_GAME_MODE==0x01)//RMUC
			return (uint32_t)&this->robot_referee_status->referee_warning;
			#elif (RF_GAME_MODE==0x00)//RMUL
			return (uint32_t)&this->robot_referee_status->ext_referee_warning;
			#endif			
		break;
		#endif
		#ifdef RF_FIELD_DARTTIME		//0x0105
		case 0x0105:
			return (uint32_t)&this->robot_referee_status->ext_dart_remaining_time;
		break;
		#endif
		
		
		#ifdef RF_ROBOT_STATUE			//0x0201
		case 0x0201:
			return (uint32_t)&this->robot_referee_status->ext_game_robot_status;
		break;
		#endif
		#ifdef RF_ROBOT_POWERHEAT		//0x0202
		case 0x0202:
			return (uint32_t)&this->robot_referee_status->ext_power_heat_data;
		break;
		#endif
		#ifdef RF_ROBOT_POSITION		//0x0203
		case 0x0203:
			return (uint32_t)&this->robot_referee_status->ext_game_robot_pos;
		break;
		#endif
		#ifdef RF_ROBOT_BUFFER			//0x0204
		case 0x0204:
			return (uint32_t)&this->robot_referee_status->ext_buff;
		break;
		#endif
		#ifdef RF_ROBOT_DRONEENERGY		//0x0205
		case 0x0205:
			return (uint32_t)&this->robot_referee_status->aerial_robot_energy;
		break;
		#endif
		#ifdef RF_ROBOT_HURT			//0x0206
		case 0x0206:
			return (uint32_t)&this->robot_referee_status->ext_robot_hurt;
		break;
		#endif
		#ifdef RF_ROBOT_SHOOT			//0x0207
		case 0x0207:
			return (uint32_t)&this->robot_referee_status->ext_shoot_data;
		break;
		#endif
		#ifdef RF_ROBOT_BULLET			//0x0208
		case 0x0208:
			#if (RF_GAME_MODE==0x01)//RMUC
			return (uint32_t)&this->robot_referee_status->projectile_allowance;
			#elif (RF_GAME_MODE==0x00)//RMUL
			return (uint32_t)&this->robot_referee_status->ext_bullet_remaining;
			#endif
		break;
		#endif
		#ifdef RF_ROBOT_RFID			//0x0209
		case 0x0209:
			return (uint32_t)&this->robot_referee_status->ext_rfid_status;
		break;		
		#endif
		#ifdef RF_ROBOT_DARTCMD			//0x020A
		case 0x020A:
			return (uint32_t)&this->robot_referee_status->ext_dart_client_cmd;
		break;		
		#endif
		#if (RF_GAME_MODE == 0x01)//RMUC
			#ifdef RF_ROBOT_TEAMPOSITION//0x020B
			case 0x020B:
				return (uint32_t)&this->robot_referee_status->ground_robot_position;
			break;	
			#endif
			#ifdef RF_ROBOT_MARK		//0x020C
			case 0x020C:
				return (uint32_t)&this->robot_referee_status->radar_mark_data;
			break;	
			#endif
		#endif
		
		
		#ifdef RF_ITF_ROBOTDATA			//0x0301
		case 0x0301:
			return (uint32_t)&this->robot_referee_status->robot_interactive_data;
		break;		
		#endif
		#ifdef RF_ITF_CUSTOMCTR			//0x0302
		case 0x0302:
			return (uint32_t)&this->robot_referee_status->custom_robot_data;
		break;		
		#endif
		#ifdef RF_ITF_MAPCLIENT			//0x0303
		case 0x0303:
			return (uint32_t)&this->robot_referee_status->ext_robot_mapcommand;
		break;		
		#endif
		#ifdef RF_ITF_CLIENT			//0x0304
		case 0x0304:
			return (uint32_t)&this->robot_referee_status->ext_robot_command;
		break;		
		#endif
		#ifdef RF_ITF_MAPRADAR			//0x0305
		case 0x0305:
			return (uint32_t)&this->robot_referee_status->ext_client_map_command;
		break;		
		#endif
		#if (RF_GAME_MODE == 0x01)//RMUC
			#ifdef RF_ITF_CUSTOMCLIENT	//0x0306
			case 0x0306:
				return (uint32_t)&this->robot_referee_status->custom_client_data;
			break;	
			#endif
			#ifdef RF_ITF_SENTRY		//0x0307
			case 0x0307:
				return (uint32_t)&this->robot_referee_status->map_sentry_data;
			break;	
			#endif
		#endif		
		default:
			return 0;
	}
}

#ifdef RF_UI_ENABLE
/*
设置UI相关的机器人ID,客户端ID代码中自行计算
这关系到UI能不能被正确显示
本条函数还会更新所有已注册图形组的相关ID配置
*/
uint8_t cUI::SetRobotID(uint16_t RobotID)
{
	/*别的不说这个肯定非法*/
	if(RobotID==0){return 1;}
	this->RobotID = RobotID;
	/*更新所有已注册图形组的配置ID*/
	for(uint8_t i=0;i<10;i++)
	{
		if(this->pGroups[i]!=0)
		{
			/*写入发送者ID*/
			this->pGroups[i]->pbuf[9]  = this->RobotID & 0xFF;
			this->pGroups[i]->pbuf[10] = this->RobotID >> 8;
			/*写入接收者ID*/
			this->pGroups[i]->pbuf[11] = (this->RobotID+0x100) & 0xFF;
			this->pGroups[i]->pbuf[12] = (this->RobotID+0x100) >> 8;	
		}
	}
	return 0;
}
/*
删图层
pbuf	: 命令缓冲区
Operator: 图层操作符, 选中 UI_LAYER_ALL 将擦除全部图层
*/
uint8_t cUI::LayerDelete(uint8_t *pbuf, eUI_LAYER_ID Layer)
{
	/*缓冲区异常*/
	if(pbuf==0)return 1;
	/*写入包头*/
	pbuf[0] = 0xA5;
	/*写入数据长度*/
	pbuf[1] = 8;
	pbuf[2] = 0;
	/*下面这俩需要动态更新*/
	/*写入包序号*/
	pbuf[3] = UITxSeq++;
	/*计算CRC8*/
	pbuf[4] = cal_crc8_table(pbuf,4);
	/*写入cmd_id*/
	pbuf[5] = 0x01;
	pbuf[6] = 0x03;
	/*写入内容ID*/
	pbuf[7] = 0x00;
	pbuf[8] = 0x01;
	/*写入发送者ID*/
	pbuf[9]  = this->RobotID & 0xFF;
	pbuf[10] = this->RobotID >> 8;
	/*写入接收者ID*/
	pbuf[11] = (this->RobotID+0x100) & 0xFF;
	pbuf[12] = (this->RobotID+0x100) >> 8;	
	/*单图层擦除*/
	if(Layer<10){pbuf[13] = 1;}
	/*全图层擦除*/
	else{pbuf[13] = 2;}	
	/*写入图层ID*/
	pbuf[14] = Layer;	
	/*计算CRC16*/
	uint16_t CRC16 = cal_crc16_table(pbuf,15);
	/*写入CRC16*/
	pbuf[15]		= CRC16 & 0xFF;
	pbuf[16]		= CRC16 >> 8;
	
	return 0;
}	

/*
本函数将注册某个图形组
GroupName	: 图形组名称
RETRUN:		:	0:注册成功 1:图形组配置结构体异常 2:图形组已被注册 3:图形组缓冲区异常
*/
uint8_t cUI::GraphGroupReg(group_config_t* pGroup)
{
	/*图形组配置不对的清退*/
	if(pGroup==0)return 1;
	/*提取下组名称方便后续处理*/
	uint8_t gpName = (uint8_t)pGroup->GroupName;
	/*已被注册的清退*/
	if(this->pGroups[gpName]!=0){return 2;}	
	/*缓冲区异常的清退*/
	if(this->pGroups[gpName]->pbuf!=0){return 3;}	
	/*注册缓冲区*/
	this->pGroups[gpName] = pGroup;
	
	/*写入缓冲区包头*/
	this->pGroups[gpName]->pbuf[0] = 0xA5;//120 0X0104
	/*计算数据长度*/
	uint16_t tmp = 6 + 15 * (uint8_t)this->pGroups[gpName]->GraphNum;
	this->pGroups[gpName]->pbuf[1] = tmp & 0xFF;
	this->pGroups[gpName]->pbuf[2] = tmp >> 8;
	/*下面这俩需要动态更新*/
//	/*写入包序号*/
//	this->pGroups[GroupName]->pbuf[3] = UITxSeq;
//	/*计算CRC8*/
//	this->pGroups[GroupName]->pbuf[4] = cal_crc8_table(this->pGroups[GroupName]->pbuf,4);
	/*写入cmd_id*/
	this->pGroups[gpName]->pbuf[5] = 0x01;
	this->pGroups[gpName]->pbuf[6] = 0x03;
	
	/*写入内容ID*/
	switch((uint8_t)this->pGroups[gpName]->GraphNum)
	{
		case 1:tmp = 0x0101;break;
		case 2:tmp = 0x0102;break;
		case 5:tmp = 0x0103;break;
		case 7:tmp = 0x0104;break;
	}
	this->pGroups[gpName]->pbuf[7] = tmp & 0xFF;
	this->pGroups[gpName]->pbuf[8] = tmp >>	8;
	/*写入发送者ID*/
	this->pGroups[gpName]->pbuf[9]  = this->RobotID & 0xFF;
	this->pGroups[gpName]->pbuf[10] = this->RobotID >> 8;
	/*写入接收者ID*/
	this->pGroups[gpName]->pbuf[11] = (this->RobotID+0x100) & 0xFF;
	this->pGroups[gpName]->pbuf[12] = (this->RobotID+0x100) >> 8;	
	
	/*计数器+1*/
	GroupNum++;
	return 0;
}

/*
本函数将删除某个图形组的注册
GroupName	: 图形组名称
*/
uint8_t cUI::GraphGroupDelete(eUI_GROUP GroupName)
{
	/*没被注册的直接返回*/
	if(this->pGroups[GroupName]==0){return 1;}
	/*检查缓冲区状态*/
	if(this->pGroups[GroupName]->pbuf==0){return 1;}
	/*已注册的清除指针后计数器-1*/
	this->pGroups[GroupName]=0;
	GroupNum--;
	return 0;
}

/*
GroupName	: 图形组名称
puibuf		: 用于获取图形数据包位置, 该指针被用于后续数据发送(记得清Cache)
RETURN		: 1:图形组名称错误 0:正常
*/
uint8_t cUI::GrapgGroupPack(eUI_GROUP GroupName,uint8_t* puibuf)
{
	/*没被注册的直接返回*/
	if(this->pGroups[GroupName]==0){return 1;}
	/*写入包序号*/
	this->pGroups[GroupName]->pbuf[3] = this->UITxSeq++;
	/*计算CRC8*/
	this->pGroups[GroupName]->pbuf[4] = cal_crc8_table(this->pGroups[GroupName]->pbuf,4);
	/*计算到CRC16的数据长度*/
	uint8_t datalen = 13 + 15 * (uint8_t)this->pGroups[GroupName]->GraphNum;
	/*计算CRC16*/
	uint16_t CRC16 = cal_crc16_table(this->pGroups[GroupName]->pbuf,datalen);
	/*写入CRC16*/
	this->pGroups[GroupName]->pbuf[datalen]		= CRC16 & 0xFF;
	this->pGroups[GroupName]->pbuf[datalen+1]	= CRC16 >> 8;
	/*输出UI数据缓冲区指针*/
	puibuf = this->pGroups[GroupName]->pbuf;
	return 0;
}

/*
刷新某图形组某图像为线条
GroupName	: 组ID		0-9
GraphID		: 图像ID	取决于组允许的最大数目
name		: 图像名称	空格则使用上次图像ID设置的图像名称
operate		: 操作符	
color		: 颜色
width		: 线宽
start_x		: 起点X
start_y		: 起点Y
end_x		: 终点X
end_y		: 终点Y
*/
uint8_t cUI::GraphGroupUpdateLine		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x,uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
	/*没注册返回1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*图形ID超出组个数限制返回2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*调整指针位置*/
	/*偏移量: 5Byte 帧头 + 2Byte cmd_id + 6Byte 内容ID + 15*(本图形GID-1)*/
	/*GID从1开始*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-空;1-增加;2-修改;3-删除
	p_graphic_data_struct->graphic_tpye=0; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//图层0-9
	p_graphic_data_struct->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
//	p_graphic_data_struct->start_angle=0; 											//空\无影响
//	p_graphic_data_struct->end_angle=0; 											//空\无影响
	p_graphic_data_struct->width=width; //线条宽度
	p_graphic_data_struct->start_x=start_x; //起点x坐标
	p_graphic_data_struct->start_y=start_y; //起点y坐标
//	p_graphic_data_struct->radius=0; 												//空\无影响
	p_graphic_data_struct->end_x=end_x; //终点x坐标
	p_graphic_data_struct->end_y=end_y; //终点y坐标
	
	return 0;
}

/*
刷新某图形组某图像为矩形
GroupName	: 组ID		0-9
GraphID		: 图像ID	取决于组允许的最大数目
name		: 图像名称	空格则使用上次图像ID设置的图像名称
operate		: 操作符	
color		: 颜色
width		: 线宽
start_x		: 起点X
start_y		: 起点Y
opposite_x	: 对角X
opposite_y	: 对角Y
*/
uint8_t cUI::GraphGroupUpdateRectangle	(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y)
{
	/*没注册返回1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*图形ID超出组个数限制返回2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*调整指针位置*/
	/*偏移量: 5Byte 帧头 + 2Byte cmd_id + 6Byte 内容ID + 15*(本图形GID-1)*/
	/*GID从1开始*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-空;1-增加;2-修改;3-删除
	p_graphic_data_struct->graphic_tpye=1; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//图层0-9
	p_graphic_data_struct->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
//	p_graphic_data_struct->start_angle=0; 											//空\无影响
//	p_graphic_data_struct->end_angle=0; 											//空\无影响
	p_graphic_data_struct->width=width; 											//线条宽度
	p_graphic_data_struct->start_x=start_x; 										//起点x坐标
	p_graphic_data_struct->start_y=start_y; 										//起点y坐标
//	p_graphic_data_struct->radius=0; 												//空\无影响
	p_graphic_data_struct->end_x=opposite_x; 										//终点x坐标
	p_graphic_data_struct->end_y=opposite_y; 										//终点y坐标
	
	return 0;
}

/*
刷新某图形组某图像为圆
GroupName	: 组ID		0-9
GraphID		: 图像ID	取决于组允许的最大数目
name		: 图像名称	空格则使用上次图像ID设置的图像名称
operate		: 操作符	
color		: 颜色
width		: 线宽
center_x	: 圆心X
center_x	: 圆心Y
radius		: 半径
*/
uint8_t cUI::GraphGroupUpdateCircle		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t radius)
{
	/*没注册返回1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*图形ID超出组个数限制返回2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*调整指针位置*/
	/*偏移量: 5Byte 帧头 + 2Byte cmd_id + 6Byte 内容ID + 15*(本图形GID-1)*/
	/*GID从1开始*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-空;1-增加;2-修改;3-删除
	p_graphic_data_struct->graphic_tpye=2; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//图层0-9
	p_graphic_data_struct->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
//	p_graphic_data_struct->start_angle=0; 											//空\无影响
//	p_graphic_data_struct->end_angle=0; 											//空\无影响
	p_graphic_data_struct->width=width; 											//线条宽度
	p_graphic_data_struct->start_x=center_x; 										//起点x坐标
	p_graphic_data_struct->start_y=center_y; 										//起点y坐标
	p_graphic_data_struct->radius=radius; 											//半径
//	p_graphic_data_struct->end_x=0;													//空\无影响
//	p_graphic_data_struct->end_y=0; 												//空\无影响
	
	return 0;
}

/*
刷新某图形组某图像为椭圆
GroupName	: 组ID		0-9
GraphID		: 图像ID	取决于组允许的最大数目
name		: 图像名称	空格则使用上次图像ID设置的图像名称
operate		: 操作符	
color		: 颜色
width		: 线宽
center_x	: 圆心X
center_x	: 圆心Y
half_axis_x	: 半轴X高
half_axis_y	: 半轴Y高
*/
uint8_t cUI::GraphGroupUpdateEllipse(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
	/*没注册返回1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*图形ID超出组个数限制返回2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*调整指针位置*/
	/*偏移量: 5Byte 帧头 + 2Byte cmd_id + 6Byte 内容ID + 15*(本图形GID-1)*/
	/*GID从1开始*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-空;1-增加;2-修改;3-删除
	p_graphic_data_struct->graphic_tpye=3; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//图层0-9
	p_graphic_data_struct->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
//	p_graphic_data_struct->start_angle=0; 											//空\无影响
//	p_graphic_data_struct->end_angle=0; 											//空\无影响
	p_graphic_data_struct->width=width; 											//线条宽度
	p_graphic_data_struct->start_x=center_x; 										//起点x坐标
	p_graphic_data_struct->start_y=center_y; 										//起点y坐标
//	p_graphic_data_struct->radius=0; 												//空\无影响
	p_graphic_data_struct->end_x=half_axis_x;										//半轴X高
	p_graphic_data_struct->end_y=half_axis_y; 										//半轴Y高
	
	return 0;
}

/*
刷新某图形组某图像为圆弧
GroupName	: 组ID		0-9
GraphID		: 图像ID	取决于组允许的最大数目
name		: 图像名称	空格则使用上次图像ID设置的图像名称
operate		: 操作符	
color		: 颜色
width		: 线宽
start_angle	: 起始角度
end_angle	: 终止角度
center_x	: 圆心X
center_y	: 圆心Y
half_axis_x	: 半轴X高
half_axis_y	: 半轴Y高
*/
uint8_t cUI::GraphGroupUpdateArc		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
	/*没注册返回1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*图形ID超出组个数限制返回2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*调整指针位置*/
	/*偏移量: 5Byte 帧头 + 2Byte cmd_id + 6Byte 内容ID + 15*(本图形GID-1)*/
	/*GID从1开始*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-空;1-增加;2-修改;3-删除
	p_graphic_data_struct->graphic_tpye=4; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//图层0-9
	p_graphic_data_struct->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
	p_graphic_data_struct->start_angle=start_angle; 								//起始角度
	p_graphic_data_struct->end_angle=end_angle; 									//终止角度
	p_graphic_data_struct->width=width; 											//线条宽度
	p_graphic_data_struct->start_x=center_x; 										//圆心X
	p_graphic_data_struct->start_y=center_y; 										//圆心Y
//	p_graphic_data_struct->radius=0; 												//空\无影响
	p_graphic_data_struct->end_x=half_axis_x;										//半轴X高
	p_graphic_data_struct->end_y=half_axis_y; 										//半轴Y高
	
	return 0;
}

/*
加载指定字符到指定内存区域
buf			: 内存区域,长度60Byte
Layer		: 字符所在层
name		: 字符名称
operate		: 操作符	
color		: 颜色
width		: 线宽
start_x		: 起始X
start_y		: 起始Y
str ... 	: 不定长字符数据
*/
uint8_t cUI::CharacterUpdate( uint8_t* pbuf, eUI_LAYER_ID Layer, char *name, eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint8_t size, 
										uint16_t start_x, uint16_t start_y, const char *str, ...)
{
	/*缓冲区异常*/
	if(pbuf==0)return 1;
	/*写入缓冲区包头*/
	pbuf[0] = 0xA5;//120 0X0104
	/*计算数据长度*/
	uint16_t datalen = 51;
	pbuf[1] = datalen & 0xFF;
	pbuf[2] = datalen >> 8;
	/*下面这俩需要动态更新*/
	/*写入包序号*/
	pbuf[3] = UITxSeq++;
	/*计算CRC8*/
	pbuf[4] = cal_crc8_table(pbuf,4);
	/*写入cmd_id*/
	pbuf[5] = 0x01;
	pbuf[6] = 0x03;
	
	/*写入内容ID*/
	pbuf[7] = 0x10;
	pbuf[8] = 0x01;
	/*写入发送者ID*/
	pbuf[9]  = this->RobotID & 0xFF;
	pbuf[10] = this->RobotID >> 8;
	/*写入接收者ID*/
	pbuf[11] = (this->RobotID+0x100) & 0xFF;
	pbuf[12] = (this->RobotID+0x100) >> 8;	
	
	
	
	/*计算字符串长度,并将字符串输出到数据区*/
	va_list ap;
	va_start(ap, str);
	uint8_t len = vsnprintf( (char*)pbuf+28, 30, str, ap);
	va_end(ap);
	
	/*配置发送*/
	graphic_data_struct_t *p_client_custom_character =  (graphic_data_struct_t *)(pbuf+13);

	
	p_client_custom_character->graphic_name[0]=name[0];
	p_client_custom_character->graphic_name[1]=name[1];
	p_client_custom_character->graphic_name[2]=name[2];
	p_client_custom_character->operate_tpye=operate;		 							//0-空;1-增加;2-修改;3-删除
	p_client_custom_character->graphic_tpye=4; 											//0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
	p_client_custom_character->layer=Layer;												//图层0-9
	p_client_custom_character->color=color; 											//0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
	p_client_custom_character->start_angle=size*10; 									//字体大小
	p_client_custom_character->end_angle=len; 											//字符长度
	p_client_custom_character->width=size; 												//字体线宽
	p_client_custom_character->start_x=start_x; 										//起始点X
	p_client_custom_character->start_y=start_x; 										//起始点Y
//	p_client_custom_character->radius=0; 												//空\无影响
//	p_client_custom_character->end_x=0;													//空\无影响
//	p_client_custom_character->end_y=0; 												//空\无影响
	
	/*计算CRC16*/
	uint16_t CRC16 = cal_crc16_table(pbuf,58);
	/*写入CRC16*/
	pbuf[58]		= CRC16 & 0xFF;
	pbuf[59]		= CRC16 >> 8;
	
	return 0;
}

#endif
