//B register allocation map
#define B_REQ 80 // B register Request code as 
//                      0-3: rovi request
//                      10-13 set target address
#define I_ARG 80 // param for request and error code on return
#define I_SVRPORT I_ARG+1
#define I_ADDR I_ARG+2
#define I_PORT I_ARG+6

static char tg_addr[4];  //must be big endian
static unsigned short tg_port;
static unsigned short self_port;

#include "motoPlus.h"
extern STATUS GetBVar(UINT16 index, long *value);
extern STATUS SetBVar(UINT16 index, long value);
extern STATUS GetIVar(UINT16 index, long *value);
extern STATUS SetIVar(UINT16 index, long value);
extern STATUS SetDVar(UINT16 index, long value);
extern STATUS GetSVar(UINT16 index, char *value);

extern STATUS SetDVar(UINT16 index, long value);
extern SEM_ID semid2,semid3;

typedef struct{
    USHORT usType;
    USHORT usIndex;
    CHAR VariableType;
    CHAR config;
    USHORT ToolNo_UserNo;
    LONG ulPosdata[9];
} MP_POSVAR_DATA2;

void mpTask1();
void mpTask2();
void mpTask3();

char *send_reply(char *msg);
void error_handler(int sig){
  printf("signal %d\n",sig);
}

void mpTask1(void){
  puts("Activate Task1");
  LONG ivalue=0;
  GetIVar(I_SVRPORT,&ivalue);
  self_port=ivalue;
  GetIVar(I_ADDR,&ivalue);
  tg_addr[0]=ivalue;
  GetIVar(I_ADDR+1,&ivalue);
  tg_addr[1]=ivalue;
  GetIVar(I_ADDR+2,&ivalue);
  tg_addr[2]=ivalue;
  GetIVar(I_ADDR+3,&ivalue);
  tg_addr[3]=ivalue;
  GetIVar(I_PORT,&ivalue);
  tg_port=ivalue;

  mpTaskDelay(1000);
  mpSemGive(semid2);
  mpSemGive(semid3);
  mpSuspendSelf;
}

void mpTask2(int arg1, int arg2){
  STATUS status=mpSemTake(semid2,WAIT_FOREVER);
  LONG bvalue=255;
  LONG ivalue=0;
  char tmpbuf[100];
  char *resp;
  puts("Activate Task2(Rooter)");
  SetBVar(B_REQ,bvalue);

  while(1){
    int rc;
    mpTaskDelay(100);
    if (status==ERROR){
      printf("semTake Error!\n");
      continue;
    }
    resp=NULL;
    GetBVar(B_REQ,&bvalue);
    if(bvalue==255) continue;
    GetIVar(I_ARG,&ivalue);
	printf("ivalue %d\n",ivalue);
    switch(bvalue){
    case 0:
      resp=send_reply("X0()");
      break;
    case 1:
      {
        memset(tmpbuf,0,sizeof(tmpbuf));
        MP_CTRL_GRP_SEND_DATA send_data;
        MP_CART_POS_RSP_DATA rsp_data;
        send_data.sCtrlGrp=0;
        if(mpGetCartPos(&send_data,&rsp_data)==0){
          sprintf(tmpbuf,"X1(%ld,%ld,%ld,%ld,%ld,%ld)",rsp_data.lPos[0],rsp_data.lPos[1],rsp_data.lPos[2],rsp_data.lPos[3],rsp_data.lPos[4],rsp_data.lPos[5]);
	    }
	    else{
	      sprintf(tmpbuf,"X1()");
	    }
	  }
      resp=send_reply(tmpbuf);
      break;
    case 2:
      resp=send_reply("X2()");
      if(strncmp(resp,"OK",2)==0){
  	    MP_POSVAR_DATA2 posvar;
	    posvar.usType=MP_RESTYPE_VAR_ROBOT;
        posvar.config=0;
        posvar.ToolNo_UserNo=0;
	    posvar.VariableType=16;//MP_RESTYPE_VAR_BASE;
	    posvar.usIndex=ivalue;
	    posvar.ulPosdata[0]=0;
	    char *tok=resp;
	    int i;
	    for(i=1;;i++){
	      tok=strchr(tok,',');
	      if(tok!=NULL) posvar.ulPosdata[i]=atol(++tok);
	      else break;
	    }
        rc=mpPutPosVarData((MP_POSVAR_DATA *)&posvar, 1);
      }
      break;
    case 3:
      memset(tmpbuf,0,sizeof(tmpbuf));
//      strcpy(tmpbuf,"X3(");
//      rc=GetSVar(ivalue,tmpbuf+3);
//      printf("GetSVar(%d) %s\n",rc,tmpbuf+3);
//      strcat(tmpbuf,")");
      sprintf(tmpbuf,"X3(%d)",ivalue);
      printf("X3 %s\n",tmpbuf);
      resp=send_reply(tmpbuf);
      break;
    }
    bvalue=255;
    if(strncmp(resp,"NG",2)==0) ivalue=atoi(resp+3);
    else ivalue=0;
    SetIVar(I_ARG,ivalue);
    SetBVar(B_REQ,bvalue);
  }
  mpSuspendSelf;
}

void mpTask3(void){
  STATUS status=mpSemTake(semid3,WAIT_FOREVER);
  puts("Activate Task3(Joint server)");
  int sockHandle = mpSocket(AF_INET, SOCK_STREAM, 0);
  printf("sock handle %d\n",sockHandle);
  struct sockaddr_in serverSockAddr;
  memset(&serverSockAddr, 0, sizeof(serverSockAddr));
  serverSockAddr.sin_family = AF_INET;
  serverSockAddr.sin_addr.s_addr = INADDR_ANY;
  serverSockAddr.sin_port = mpHtons(self_port);
  int rc = mpBind(sockHandle, (struct sockaddr *)&serverSockAddr, sizeof(serverSockAddr));
  if (rc < 0) goto closeSockHandle;
  rc = mpListen(sockHandle, SOMAXCONN);
  printf("listen %d\n",rc);
  if (rc < 0) goto closeSockHandle;
  while (1){
    struct sockaddr_in clientSockAddr;
    memset(&clientSockAddr, 0, sizeof(clientSockAddr));
    int sizeofSockAddr = sizeof(clientSockAddr);
    int acceptHandle = mpAccept(sockHandle, (struct sockaddr *)&clientSockAddr,&sizeofSockAddr);
    printf("Accept %d\n",acceptHandle);
    if (acceptHandle < 0) break;
    while(1){
      char buff[256];
      mpTaskDelay(100);
//      memset(buff,0,sizeof(buff));
//      int ret = mpRecv(acceptHandle, buff, 256, 0);
//      if(ret<0){
//        puts("Task3 Recv error");
//        break;
//      }
      MP_CTRL_GRP_SEND_DATA sData;
      MP_DEG_POS_RSP_DATA joint_data;
      sData.sCtrlGrp = 0;
      memset(&joint_data, 0, sizeof(joint_data));
      STATUS stat=mpGetDegPos(&sData,&joint_data);
      if(stat==ERROR){
        puts("getDegPos error");
        continue;
      }
      strcpy(buff,"J(");
      int i=0;
      while(1){
	    sprintf(buff+strlen(buff),"%ld",joint_data.lDegPos[i]);
	    if(++i<MAX_PULSE_AXES) strcat(buff,",");
	    else{
    	    strcat(buff,")\n");
	        break;
	    }
	  }
      MP_CART_POS_RSP_DATA rsp_data;
      sData.sCtrlGrp = 0;
      if(mpGetCartPos(&sData,&rsp_data)==0){
        sprintf(buff+strlen(buff),"P(%ld,%ld,%ld,%ld,%ld,%ld)\n",rsp_data.lPos[0],rsp_data.lPos[1],rsp_data.lPos[2],rsp_data.lPos[3],rsp_data.lPos[4],rsp_data.lPos[5]);
	  }
	  int len = strlen(buff);
      int ret = mpSend(acceptHandle, buff, len, 0);
      if(ret<0){
        puts("joint send error");
        break;
      }
    }
    mpClose(acceptHandle);
  }
closeSockHandle:
  mpClose(sockHandle);
  puts("Task3 suspended");
  mpSuspendSelf;
}

char *send_reply(char *msg){
  int sockHandle = mpSocket(AF_INET, SOCK_STREAM, 0);
  printf("send reply socket %d\n",sockHandle);
  if (sockHandle < 0) return NULL;
  
  struct sockaddr_in sockAddr;
  memset((char *)&sockAddr,0,sizeof(sockAddr));
  sockAddr.sin_family = AF_INET;
  memcpy((char *)&sockAddr.sin_addr.s_addr,tg_addr,sizeof(sockAddr.sin_addr.s_addr));
  sockAddr.sin_port = mpHtons(tg_port);
  printf("try connectiog %d\n",tg_port);

  if(mpConnect(sockHandle, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) < 0) {
    mpClose(sockHandle);
    perror("ERROR connecting");
    return NULL;
  }

  char sbuff[256];
  strcpy(sbuff,msg);
  int n = mpSend(sockHandle, sbuff, strlen(sbuff), 0);
  if (n < 0) {
    mpClose(sockHandle);
    perror("ERROR writing to socket");
    return NULL;
  }

  static char rbuff[256];
  memset(rbuff,0,sizeof(rbuff));
  n = mpRecv(sockHandle, rbuff, 256, 0);
  if (n < 0) {
    mpClose(sockHandle);
    perror("ERROR reading from socket");
    return NULL;
  }
  mpClose(sockHandle);
  return rbuff;
}
