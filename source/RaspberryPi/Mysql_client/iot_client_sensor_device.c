#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>

#define BUF_SIZE 100
#define NAME_SIZE 20
#define ARR_CNT 10

typedef struct _RGB {
	int R, G, B;
}RGB;

void* send_msg(void* arg);
void* recv_msg(void* arg);
void error_handling(char* msg);
void getLampData(int* sock, MYSQL* conn, char** pArray, int i);
void insertLampLog(int* sock, MYSQL* conn, char** pArray);
void getRGB(int* sock, MYSQL* conn, char** pArray, RGB* pRGB);
void updateLampData(int* sock, MYSQL* conn, char** pArray);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

// sql 결과 함수화
// ----------------------------------------------------------
void sqlDMLres(MYSQL* conn, int res, const char* strDML);
// ----------------------------------------------------------

int main(int argc, char* argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread;
	void* thread_return;

	if (argc != 4) {
		printf("Usage : %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	sprintf(name, "%s", argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void*)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void*)&sock);

	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	if(sock != -1)
		close(sock);
	return 0;
}


void* send_msg(void* arg)
{
	int* sock = (int*)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE + 2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
	while (1) {
		memset(msg, 0, sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if (!strncmp(msg, "quit\n", 5)) {
				*sock = -1;
				return NULL;
			}
			else if (msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg, msg);
			if (write(*sock, name_msg, strlen(name_msg)) <= 0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if (ret == 0)
		{
			if (*sock == -1)
				return NULL;
		}
	}
}

void* recv_msg(void* arg)
{
	MYSQL* conn;
	MYSQL_ROW sqlrow;
	
	int res;
	char sql_cmd[200] = { 0 };
	char* host = "localhost";
	char* user = "miniproject";
	char* pass = "pwmini";
	char* dbname = "miniproject2";

	int* sock = (int*)arg;
	int i;
	char* pToken;
	char* pArray[ARR_CNT] = { 0 };

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	int str_len;

	int setRv,setGv,setBv;
	
	int setOntime, setMode, setSpeed;
	
	
	conn = mysql_init(NULL);

	puts("MYSQL startup");
	if (!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0)))
	{
		fprintf(stderr, "ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else
		printf("Connection Successful!\n\n");

	while (1) {
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if (str_len <= 0)
		{
			*sock = -1;
			return NULL;
		}
		fputs(name_msg, stdout);
//		name_msg[str_len-1] = 0;   //'\n' 제거
		name_msg[strcspn(name_msg,"\n")] = '\0';

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while (pToken != NULL)
		{
			pArray[i] = pToken;
			if ( ++i >= ARR_CNT)
				break;
			pToken = strtok(NULL, "[:@]");

		}
	
		// printf("out LAMP SECTION %s %s %s %s %s %s %s \n",pArray[1], pArray[2], pArray[3], pArray[4], pArray[5], pArray[6], pArray[7]);// test
		// printf("%s %s %s %s", pArray[0], pArray[1], pArray[2], pArray[3]);
		if(!strcmp(pArray[1],"LAMP")){ 
			
			if(!strcmp(pArray[2],"SET")){

				setRv = atoi(pArray[3]);
				setGv = atoi(pArray[4]);
			    setBv = atoi(pArray[5]);
				setOntime = atoi(pArray[6]);
				setMode = atoi(pArray[7]);
				setSpeed = atoi(pArray[8]);
			
			
				sprintf(sql_cmd,"select * from moodlamp where name='%s'",pArray[0]);// select부터 해야함 
				res = mysql_query(conn, sql_cmd);
					MYSQL_RES *result = mysql_store_result(conn);					
				if(!res)
				{
					int num_rows = mysql_num_rows(result);
					
						
					if(!num_rows){
						
						sprintf(sql_cmd, "insert moodlamp(name, red, green, blue, ontime, mode, speed) values('%s',%d,%d,%d,%d,%d)",pArray[0],setRv,setGv,setBv,setOntime,setMode,setSpeed);
						res = mysql_query(conn,sql_cmd);

						sqlDMLres(conn, res, "insert");
					}
					else{

  						sprintf(sql_cmd, "update moodlamp set red=%d,green=%d,blue=%d,ontime=%d,mode=%d,speed=%d where name='%s'",setRv,setGv, setBv,setOntime, setMode, setSpeed,pArray[0]);
						res = mysql_query(conn, sql_cmd);

						sqlDMLres(conn, res, "update");
						
					}
				}
				sprintf(sql_cmd,"[%s]%s@%s\n",pArray[0],pArray[1],pArray[2]);
				write(*sock, sql_cmd, strlen(sql_cmd));
			}
			else if(!strcmp(pArray[2],"GET")){
				getLampData(sock, conn, pArray,i);
			}
			else if (!strcmp(pArray[2], "ON")) {// 센싱이 되었다면 select로 킬 조명 보내주고 log에 인서트 해주면 됨
				printf("IN ON");
				getLampData(sock, conn, pArray, i);  // PIR 센서가 감지 됐으니 무드등에 보내줘야 할 데이터를 select 하고 
				insertLampLog(sock, conn, pArray);// 켜진 날짜와시간, RGB값을 log 테이블에 삽입 
				updateLampData(sock, conn, pArray);
			}
			else if (!strcmp(pArray[2], "OFF")) {
				printf("IN OFF");
				updateLampData(sock, conn, pArray);
			}
		}
		
//[KSH_SQL]GETDB@LAMP
//[KSH_SQL]SETDB@LAMP@ON
//[KSH_SQL]SETDB@LAMP@ON@KSH_LIN
	}
	
	mysql_close(conn);

}

void error_handling(char* msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}

void getLampData(int*sock, MYSQL* conn, char** pArray,int i) {
	int res = -1;
	char sql_cmd[200] = { 0 };
	MYSQL_ROW sqlrow;

	printf("in GetLampData\n ");
	if(i != 4)
	{

		sprintf(sql_cmd, "select red, green, blue, ontime, mode, speed from moodlamp where name='%s'", pArray[0]);
		res = mysql_query(conn, sql_cmd);

		if (!res)	
		{
			MYSQL_RES* result = mysql_store_result(conn);
			
			sqlrow = mysql_fetch_row(result);
                        
			if(!sqlrow)
			{
				mysql_free_result(result);
			}
			else
			{
				sprintf(sql_cmd, "[%s]%s@%s@%s@%s@%s@%s@%s@%s\n", pArray[0], pArray[1], pArray[2], sqlrow[0], sqlrow[1], sqlrow[2], sqlrow[3], sqlrow[4], sqlrow[5]);
			
				write(*sock, sql_cmd, strlen(sql_cmd));
			
				mysql_free_result(result);
			}
		}
		else
			fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));

	}
	else 
	{
		pArray[3][strcspn(pArray[3], "\r\n")] = '\0';
		sprintf(sql_cmd, "select red, green, blue, ontime, mode, speed from moodlamp where name='%s'", pArray[3]);
		res = mysql_query(conn, sql_cmd);
		
		if (!res)
		{

			MYSQL_RES* result = mysql_store_result(conn);
			if (!result) {
				fprintf(stderr, "mysql_store_result failed: %s\n", mysql_error(conn));
			}
			else
			{
				
				if (strcmp(pArray[3], "LDH_SMP"))
					printf("in GetLampData in 4 == i\n ");
				
				sqlrow = mysql_fetch_row(result);
				if (sqlrow) {
					sprintf(sql_cmd, "[%s]%s@%s@%s@%s@%s@%s@%s@%s\n", pArray[0], pArray[1], pArray[2], sqlrow[0], sqlrow[1], sqlrow[2], sqlrow[3], sqlrow[4], sqlrow[5]);
					write(*sock, sql_cmd, strlen(sql_cmd));
				}
			}
			mysql_free_result(result);
		}
		else
			fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		
	}
}

void insertLampLog(int* sock, MYSQL* conn, char** pArray)
{
	int res = -1;
	char sql_cmd[200] = { 0 };
	RGB* pRGB;
	getRGB(sock, conn, pArray, pRGB);

	sprintf(sql_cmd, "insert lamplog(name, ondate, ontime, red, green, blue) values('%s',CURDATE(), CURTIME(),%d,%d,%d)", pArray[3], pRGB->R, pRGB->G, pRGB->B);
	res = mysql_query(conn, sql_cmd);


	sqlDMLres(conn, res, "insert");
}

void getRGB(int* sock, MYSQL* conn, char** pArray, RGB* pRGB) {

	int res = -1;
	char sql_cmd[200] = { 0 };
	MYSQL_ROW sqlrow;

	sprintf(sql_cmd, "select red, green, blue from moodlamp where name='%s'", pArray[3]);
	res = mysql_query(conn, sql_cmd);

	sqlDMLres(conn, res, "select");
	if (!res)
	{
		MYSQL_RES* result = mysql_store_result(conn);
		int num_fields = mysql_num_fields(result);
		sqlrow = mysql_fetch_row(result);

		pRGB->R = atoi(sqlrow[0]);
		pRGB->G = atoi(sqlrow[1]);
		pRGB->B = atoi(sqlrow[2]);
	}
	else
		fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));

}


void sqlDMLres(MYSQL* conn, int res, const char* strDML) {

	if (!res)
		printf("%s %lu rows\n", strDML, (unsigned long)mysql_affected_rows(conn));
	else
		fprintf(stderr, "ERROR:%s[%d]\n", mysql_error(conn), mysql_errno(conn));

	return;
}

void updateLampData(int* sock, MYSQL* conn, char** pArray) {
	int res = -1;
	char sql_cmd[200] = { 0 };
	sprintf(sql_cmd, "update moodlamp set mode=%d where name='%s'", !strcmp(pArray[2], "ON") ? 1 : 2, pArray[3]);
	res = mysql_query(conn, sql_cmd);

	sqlDMLres(conn, res, "update");
}
