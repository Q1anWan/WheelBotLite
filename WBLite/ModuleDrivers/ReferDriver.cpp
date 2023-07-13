#include "ReferDriver.h"
#include "stdarg.h"
#include "CRC16.h"
#include "CRC8.h"
/*
	����˼·
	�ṹ��ֱ�ӽ��п���
	LSB
	1.�����ͷ����ȷ��������Ч��
	2.ȷ������ID
	3.�ڴ濽��

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
	/*������Ч����*/
	this->DMA_Msg.packnum = 0;	
	/*��̬���µ�ָ��*/
	uint8_t *pdata_process = data;
	
	/*������*/
	while(pdata_process[0] == 0xA5)
	{
		/*��ѯ�洢����ַ*/
		this->DMA_Msg.dstaddr[this->DMA_Msg.packnum] = this->M2M_CheckDst( (uint16_t)pdata_process[6]<<8 |  (uint16_t)pdata_process[5] );
		/*��ȡ��������*/
		this->DMA_Msg.packlen[this->DMA_Msg.packnum] = (uint16_t)pdata_process[2]<<8  |  (uint16_t)pdata_process[1];
		/*�жϱ����Ƿ���Ҫ*/
		if(this->DMA_Msg.dstaddr[this->DMA_Msg.packnum]==0)
		{	
			/*�Ҳ�����Ӧ�ṹ��İ�������*/
			pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
			continue;
		}
		/*У�鱾��CRC16*/
		if( cal_crc16_table(pdata_process, this->DMA_Msg.packlen[this->DMA_Msg.packnum]+7 ) != ( (uint16_t)pdata_process[this->DMA_Msg.packlen[this->DMA_Msg.packnum]+8]<<8 | (uint16_t)pdata_process[this->DMA_Msg.packlen[this->DMA_Msg.packnum]+7] ) )
		{
			/*CRC16����İ�������*/
			pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
			this->error_packp = (uint32_t)this->DMA_Msg.dstaddr[this->DMA_Msg.packnum];
			this->error_pack++;
			continue;
		}
		/*���㱾������λָ��*/
		this->DMA_Msg.packhead[this->DMA_Msg.packnum]	= (uint32_t)pdata_process + 7;
		/*����ָ��λ��*/
		pdata_process += (9 + this->DMA_Msg.packlen[this->DMA_Msg.packnum]);
		/*������Ч����*/
		this->DMA_Msg.packnum++;
		/*����Ƿ񱾴ν�������*/
		if((uint32_t)(pdata_process - data)>=len)
		{
			break;
		}
	}
	return 0;
}

void cRef::M2M_Transmit(uint8_t packid)
{
		/*msg[packnum-1]����*/
		LL_DMA_SetM2MSrcAddress(this->M2M.DMAx,this->M2M.CH,this->DMA_Msg.packhead[packid]);
		LL_DMA_SetM2MDstAddress(this->M2M.DMAx,this->M2M.CH,this->DMA_Msg.dstaddr[packid]);
		LL_DMA_SetDataLength(this->M2M.DMAx,this->M2M.CH, this->DMA_Msg.packlen[packid]);
		this->M2M.EnableCH();
}

/*
	O(1)���Ӷ��ڴ��ַ��ȡ
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
����UI��صĻ�����ID,�ͻ���ID���������м���
���ϵ��UI�ܲ��ܱ���ȷ��ʾ
���������������������ע��ͼ��������ID����
*/
uint8_t cUI::SetRobotID(uint16_t RobotID)
{
	/*��Ĳ�˵����϶��Ƿ�*/
	if(RobotID==0){return 1;}
	this->RobotID = RobotID;
	/*����������ע��ͼ���������ID*/
	for(uint8_t i=0;i<10;i++)
	{
		if(this->pGroups[i]!=0)
		{
			/*д�뷢����ID*/
			this->pGroups[i]->pbuf[9]  = this->RobotID & 0xFF;
			this->pGroups[i]->pbuf[10] = this->RobotID >> 8;
			/*д�������ID*/
			this->pGroups[i]->pbuf[11] = (this->RobotID+0x100) & 0xFF;
			this->pGroups[i]->pbuf[12] = (this->RobotID+0x100) >> 8;	
		}
	}
	return 0;
}
/*
ɾͼ��
pbuf	: �������
Operator: ͼ�������, ѡ�� UI_LAYER_ALL ������ȫ��ͼ��
*/
uint8_t cUI::LayerDelete(uint8_t *pbuf, eUI_LAYER_ID Layer)
{
	/*�������쳣*/
	if(pbuf==0)return 1;
	/*д���ͷ*/
	pbuf[0] = 0xA5;
	/*д�����ݳ���*/
	pbuf[1] = 8;
	pbuf[2] = 0;
	/*����������Ҫ��̬����*/
	/*д������*/
	pbuf[3] = UITxSeq++;
	/*����CRC8*/
	pbuf[4] = cal_crc8_table(pbuf,4);
	/*д��cmd_id*/
	pbuf[5] = 0x01;
	pbuf[6] = 0x03;
	/*д������ID*/
	pbuf[7] = 0x00;
	pbuf[8] = 0x01;
	/*д�뷢����ID*/
	pbuf[9]  = this->RobotID & 0xFF;
	pbuf[10] = this->RobotID >> 8;
	/*д�������ID*/
	pbuf[11] = (this->RobotID+0x100) & 0xFF;
	pbuf[12] = (this->RobotID+0x100) >> 8;	
	/*��ͼ�����*/
	if(Layer<10){pbuf[13] = 1;}
	/*ȫͼ�����*/
	else{pbuf[13] = 2;}	
	/*д��ͼ��ID*/
	pbuf[14] = Layer;	
	/*����CRC16*/
	uint16_t CRC16 = cal_crc16_table(pbuf,15);
	/*д��CRC16*/
	pbuf[15]		= CRC16 & 0xFF;
	pbuf[16]		= CRC16 >> 8;
	
	return 0;
}	

/*
��������ע��ĳ��ͼ����
GroupName	: ͼ��������
RETRUN:		:	0:ע��ɹ� 1:ͼ�������ýṹ���쳣 2:ͼ�����ѱ�ע�� 3:ͼ���黺�����쳣
*/
uint8_t cUI::GraphGroupReg(group_config_t* pGroup)
{
	/*ͼ�������ò��Ե�����*/
	if(pGroup==0)return 1;
	/*��ȡ�������Ʒ����������*/
	uint8_t gpName = (uint8_t)pGroup->GroupName;
	/*�ѱ�ע�������*/
	if(this->pGroups[gpName]!=0){return 2;}	
	/*�������쳣������*/
	if(this->pGroups[gpName]->pbuf!=0){return 3;}	
	/*ע�Ỻ����*/
	this->pGroups[gpName] = pGroup;
	
	/*д�뻺������ͷ*/
	this->pGroups[gpName]->pbuf[0] = 0xA5;//120 0X0104
	/*�������ݳ���*/
	uint16_t tmp = 6 + 15 * (uint8_t)this->pGroups[gpName]->GraphNum;
	this->pGroups[gpName]->pbuf[1] = tmp & 0xFF;
	this->pGroups[gpName]->pbuf[2] = tmp >> 8;
	/*����������Ҫ��̬����*/
//	/*д������*/
//	this->pGroups[GroupName]->pbuf[3] = UITxSeq;
//	/*����CRC8*/
//	this->pGroups[GroupName]->pbuf[4] = cal_crc8_table(this->pGroups[GroupName]->pbuf,4);
	/*д��cmd_id*/
	this->pGroups[gpName]->pbuf[5] = 0x01;
	this->pGroups[gpName]->pbuf[6] = 0x03;
	
	/*д������ID*/
	switch((uint8_t)this->pGroups[gpName]->GraphNum)
	{
		case 1:tmp = 0x0101;break;
		case 2:tmp = 0x0102;break;
		case 5:tmp = 0x0103;break;
		case 7:tmp = 0x0104;break;
	}
	this->pGroups[gpName]->pbuf[7] = tmp & 0xFF;
	this->pGroups[gpName]->pbuf[8] = tmp >>	8;
	/*д�뷢����ID*/
	this->pGroups[gpName]->pbuf[9]  = this->RobotID & 0xFF;
	this->pGroups[gpName]->pbuf[10] = this->RobotID >> 8;
	/*д�������ID*/
	this->pGroups[gpName]->pbuf[11] = (this->RobotID+0x100) & 0xFF;
	this->pGroups[gpName]->pbuf[12] = (this->RobotID+0x100) >> 8;	
	
	/*������+1*/
	GroupNum++;
	return 0;
}

/*
��������ɾ��ĳ��ͼ�����ע��
GroupName	: ͼ��������
*/
uint8_t cUI::GraphGroupDelete(eUI_GROUP GroupName)
{
	/*û��ע���ֱ�ӷ���*/
	if(this->pGroups[GroupName]==0){return 1;}
	/*��黺����״̬*/
	if(this->pGroups[GroupName]->pbuf==0){return 1;}
	/*��ע������ָ��������-1*/
	this->pGroups[GroupName]=0;
	GroupNum--;
	return 0;
}

/*
GroupName	: ͼ��������
puibuf		: ���ڻ�ȡͼ�����ݰ�λ��, ��ָ�뱻���ں������ݷ���(�ǵ���Cache)
RETURN		: 1:ͼ�������ƴ��� 0:����
*/
uint8_t cUI::GrapgGroupPack(eUI_GROUP GroupName,uint8_t* puibuf)
{
	/*û��ע���ֱ�ӷ���*/
	if(this->pGroups[GroupName]==0){return 1;}
	/*д������*/
	this->pGroups[GroupName]->pbuf[3] = this->UITxSeq++;
	/*����CRC8*/
	this->pGroups[GroupName]->pbuf[4] = cal_crc8_table(this->pGroups[GroupName]->pbuf,4);
	/*���㵽CRC16�����ݳ���*/
	uint8_t datalen = 13 + 15 * (uint8_t)this->pGroups[GroupName]->GraphNum;
	/*����CRC16*/
	uint16_t CRC16 = cal_crc16_table(this->pGroups[GroupName]->pbuf,datalen);
	/*д��CRC16*/
	this->pGroups[GroupName]->pbuf[datalen]		= CRC16 & 0xFF;
	this->pGroups[GroupName]->pbuf[datalen+1]	= CRC16 >> 8;
	/*���UI���ݻ�����ָ��*/
	puibuf = this->pGroups[GroupName]->pbuf;
	return 0;
}

/*
ˢ��ĳͼ����ĳͼ��Ϊ����
GroupName	: ��ID		0-9
GraphID		: ͼ��ID	ȡ����������������Ŀ
name		: ͼ������	�ո���ʹ���ϴ�ͼ��ID���õ�ͼ������
operate		: ������	
color		: ��ɫ
width		: �߿�
start_x		: ���X
start_y		: ���Y
end_x		: �յ�X
end_y		: �յ�Y
*/
uint8_t cUI::GraphGroupUpdateLine		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x,uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
	/*ûע�᷵��1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*ͼ��ID������������Ʒ���2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*����ָ��λ��*/
	/*ƫ����: 5Byte ֡ͷ + 2Byte cmd_id + 6Byte ����ID + 15*(��ͼ��GID-1)*/
	/*GID��1��ʼ*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-��;1-����;2-�޸�;3-ɾ��
	p_graphic_data_struct->graphic_tpye=0; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//ͼ��0-9
	p_graphic_data_struct->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
//	p_graphic_data_struct->start_angle=0; 											//��\��Ӱ��
//	p_graphic_data_struct->end_angle=0; 											//��\��Ӱ��
	p_graphic_data_struct->width=width; //�������
	p_graphic_data_struct->start_x=start_x; //���x����
	p_graphic_data_struct->start_y=start_y; //���y����
//	p_graphic_data_struct->radius=0; 												//��\��Ӱ��
	p_graphic_data_struct->end_x=end_x; //�յ�x����
	p_graphic_data_struct->end_y=end_y; //�յ�y����
	
	return 0;
}

/*
ˢ��ĳͼ����ĳͼ��Ϊ����
GroupName	: ��ID		0-9
GraphID		: ͼ��ID	ȡ����������������Ŀ
name		: ͼ������	�ո���ʹ���ϴ�ͼ��ID���õ�ͼ������
operate		: ������	
color		: ��ɫ
width		: �߿�
start_x		: ���X
start_y		: ���Y
opposite_x	: �Խ�X
opposite_y	: �Խ�Y
*/
uint8_t cUI::GraphGroupUpdateRectangle	(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y)
{
	/*ûע�᷵��1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*ͼ��ID������������Ʒ���2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*����ָ��λ��*/
	/*ƫ����: 5Byte ֡ͷ + 2Byte cmd_id + 6Byte ����ID + 15*(��ͼ��GID-1)*/
	/*GID��1��ʼ*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-��;1-����;2-�޸�;3-ɾ��
	p_graphic_data_struct->graphic_tpye=1; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//ͼ��0-9
	p_graphic_data_struct->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
//	p_graphic_data_struct->start_angle=0; 											//��\��Ӱ��
//	p_graphic_data_struct->end_angle=0; 											//��\��Ӱ��
	p_graphic_data_struct->width=width; 											//�������
	p_graphic_data_struct->start_x=start_x; 										//���x����
	p_graphic_data_struct->start_y=start_y; 										//���y����
//	p_graphic_data_struct->radius=0; 												//��\��Ӱ��
	p_graphic_data_struct->end_x=opposite_x; 										//�յ�x����
	p_graphic_data_struct->end_y=opposite_y; 										//�յ�y����
	
	return 0;
}

/*
ˢ��ĳͼ����ĳͼ��ΪԲ
GroupName	: ��ID		0-9
GraphID		: ͼ��ID	ȡ����������������Ŀ
name		: ͼ������	�ո���ʹ���ϴ�ͼ��ID���õ�ͼ������
operate		: ������	
color		: ��ɫ
width		: �߿�
center_x	: Բ��X
center_x	: Բ��Y
radius		: �뾶
*/
uint8_t cUI::GraphGroupUpdateCircle		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t radius)
{
	/*ûע�᷵��1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*ͼ��ID������������Ʒ���2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*����ָ��λ��*/
	/*ƫ����: 5Byte ֡ͷ + 2Byte cmd_id + 6Byte ����ID + 15*(��ͼ��GID-1)*/
	/*GID��1��ʼ*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-��;1-����;2-�޸�;3-ɾ��
	p_graphic_data_struct->graphic_tpye=2; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//ͼ��0-9
	p_graphic_data_struct->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
//	p_graphic_data_struct->start_angle=0; 											//��\��Ӱ��
//	p_graphic_data_struct->end_angle=0; 											//��\��Ӱ��
	p_graphic_data_struct->width=width; 											//�������
	p_graphic_data_struct->start_x=center_x; 										//���x����
	p_graphic_data_struct->start_y=center_y; 										//���y����
	p_graphic_data_struct->radius=radius; 											//�뾶
//	p_graphic_data_struct->end_x=0;													//��\��Ӱ��
//	p_graphic_data_struct->end_y=0; 												//��\��Ӱ��
	
	return 0;
}

/*
ˢ��ĳͼ����ĳͼ��Ϊ��Բ
GroupName	: ��ID		0-9
GraphID		: ͼ��ID	ȡ����������������Ŀ
name		: ͼ������	�ո���ʹ���ϴ�ͼ��ID���õ�ͼ������
operate		: ������	
color		: ��ɫ
width		: �߿�
center_x	: Բ��X
center_x	: Բ��Y
half_axis_x	: ����X��
half_axis_y	: ����Y��
*/
uint8_t cUI::GraphGroupUpdateEllipse(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
	/*ûע�᷵��1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*ͼ��ID������������Ʒ���2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*����ָ��λ��*/
	/*ƫ����: 5Byte ֡ͷ + 2Byte cmd_id + 6Byte ����ID + 15*(��ͼ��GID-1)*/
	/*GID��1��ʼ*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-��;1-����;2-�޸�;3-ɾ��
	p_graphic_data_struct->graphic_tpye=3; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//ͼ��0-9
	p_graphic_data_struct->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
//	p_graphic_data_struct->start_angle=0; 											//��\��Ӱ��
//	p_graphic_data_struct->end_angle=0; 											//��\��Ӱ��
	p_graphic_data_struct->width=width; 											//�������
	p_graphic_data_struct->start_x=center_x; 										//���x����
	p_graphic_data_struct->start_y=center_y; 										//���y����
//	p_graphic_data_struct->radius=0; 												//��\��Ӱ��
	p_graphic_data_struct->end_x=half_axis_x;										//����X��
	p_graphic_data_struct->end_y=half_axis_y; 										//����Y��
	
	return 0;
}

/*
ˢ��ĳͼ����ĳͼ��ΪԲ��
GroupName	: ��ID		0-9
GraphID		: ͼ��ID	ȡ����������������Ŀ
name		: ͼ������	�ո���ʹ���ϴ�ͼ��ID���õ�ͼ������
operate		: ������	
color		: ��ɫ
width		: �߿�
start_angle	: ��ʼ�Ƕ�
end_angle	: ��ֹ�Ƕ�
center_x	: Բ��X
center_y	: Բ��Y
half_axis_x	: ����X��
half_axis_y	: ����Y��
*/
uint8_t cUI::GraphGroupUpdateArc		(	eUI_GROUP GroupName, eUI_GROUP_GID GraphID, char *name,
										eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint16_t width,
										uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
	/*ûע�᷵��1*/
	if(this->pGroups[GroupName]==0){return 1;}	
	/*ͼ��ID������������Ʒ���2*/
	if((uint8_t)GraphID > (uint8_t)this->pGroups[GroupName]->GraphNum){return 2;}	
	
	/*����ָ��λ��*/
	/*ƫ����: 5Byte ֡ͷ + 2Byte cmd_id + 6Byte ����ID + 15*(��ͼ��GID-1)*/
	/*GID��1��ʼ*/
	graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(this->pGroups[GroupName]->pbuf + 15*(uint8_t)GraphID - 2);
	
	if(strcmp(" ",name))
	{	
		p_graphic_data_struct->graphic_name[0]=name[0];
		p_graphic_data_struct->graphic_name[1]=name[1];
		p_graphic_data_struct->graphic_name[2]=name[2];
	}
	
	p_graphic_data_struct->operate_tpye=(uint8_t)operate; 							//0-��;1-����;2-�޸�;3-ɾ��
	p_graphic_data_struct->graphic_tpye=4; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_graphic_data_struct->layer=(uint8_t)this->pGroups[GroupName]->GroupLayer;		//ͼ��0-9
	p_graphic_data_struct->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
	p_graphic_data_struct->start_angle=start_angle; 								//��ʼ�Ƕ�
	p_graphic_data_struct->end_angle=end_angle; 									//��ֹ�Ƕ�
	p_graphic_data_struct->width=width; 											//�������
	p_graphic_data_struct->start_x=center_x; 										//Բ��X
	p_graphic_data_struct->start_y=center_y; 										//Բ��Y
//	p_graphic_data_struct->radius=0; 												//��\��Ӱ��
	p_graphic_data_struct->end_x=half_axis_x;										//����X��
	p_graphic_data_struct->end_y=half_axis_y; 										//����Y��
	
	return 0;
}

/*
����ָ���ַ���ָ���ڴ�����
buf			: �ڴ�����,����60Byte
Layer		: �ַ����ڲ�
name		: �ַ�����
operate		: ������	
color		: ��ɫ
width		: �߿�
start_x		: ��ʼX
start_y		: ��ʼY
str ... 	: �������ַ�����
*/
uint8_t cUI::CharacterUpdate( uint8_t* pbuf, eUI_LAYER_ID Layer, char *name, eUI_GRAPHE_OPERATE operate, eUI_COLOR color, uint8_t size, 
										uint16_t start_x, uint16_t start_y, const char *str, ...)
{
	/*�������쳣*/
	if(pbuf==0)return 1;
	/*д�뻺������ͷ*/
	pbuf[0] = 0xA5;//120 0X0104
	/*�������ݳ���*/
	uint16_t datalen = 51;
	pbuf[1] = datalen & 0xFF;
	pbuf[2] = datalen >> 8;
	/*����������Ҫ��̬����*/
	/*д������*/
	pbuf[3] = UITxSeq++;
	/*����CRC8*/
	pbuf[4] = cal_crc8_table(pbuf,4);
	/*д��cmd_id*/
	pbuf[5] = 0x01;
	pbuf[6] = 0x03;
	
	/*д������ID*/
	pbuf[7] = 0x10;
	pbuf[8] = 0x01;
	/*д�뷢����ID*/
	pbuf[9]  = this->RobotID & 0xFF;
	pbuf[10] = this->RobotID >> 8;
	/*д�������ID*/
	pbuf[11] = (this->RobotID+0x100) & 0xFF;
	pbuf[12] = (this->RobotID+0x100) >> 8;	
	
	
	
	/*�����ַ�������,�����ַ��������������*/
	va_list ap;
	va_start(ap, str);
	uint8_t len = vsnprintf( (char*)pbuf+28, 30, str, ap);
	va_end(ap);
	
	/*���÷���*/
	graphic_data_struct_t *p_client_custom_character =  (graphic_data_struct_t *)(pbuf+13);

	
	p_client_custom_character->graphic_name[0]=name[0];
	p_client_custom_character->graphic_name[1]=name[1];
	p_client_custom_character->graphic_name[2]=name[2];
	p_client_custom_character->operate_tpye=operate;		 							//0-��;1-����;2-�޸�;3-ɾ��
	p_client_custom_character->graphic_tpye=4; 											//0-ֱ��;1-����;2-��Բ;3-��Բ;4-Բ��;5-������;6-������;7-�ַ�
	p_client_custom_character->layer=Layer;												//ͼ��0-9
	p_client_custom_character->color=color; 											//0-������ɫ;1-��ɫ;2-��ɫ;3-��ɫ;4-�Ϻ�ɫ;5-��ɫ;6-��ɫ;7-��ɫ;8-��ɫ
	p_client_custom_character->start_angle=size*10; 									//�����С
	p_client_custom_character->end_angle=len; 											//�ַ�����
	p_client_custom_character->width=size; 												//�����߿�
	p_client_custom_character->start_x=start_x; 										//��ʼ��X
	p_client_custom_character->start_y=start_x; 										//��ʼ��Y
//	p_client_custom_character->radius=0; 												//��\��Ӱ��
//	p_client_custom_character->end_x=0;													//��\��Ӱ��
//	p_client_custom_character->end_y=0; 												//��\��Ӱ��
	
	/*����CRC16*/
	uint16_t CRC16 = cal_crc16_table(pbuf,58);
	/*д��CRC16*/
	pbuf[58]		= CRC16 & 0xFF;
	pbuf[59]		= CRC16 >> 8;
	
	return 0;
}

#endif
