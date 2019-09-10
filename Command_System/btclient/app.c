#include <stdio.h>
#include <stdlib.h>
#include "ev3api.h"
#include "app.h"
#include "lcd.h"

#define DEBUG_PRINT

#define error( fmt, ... ) \
	lcd_print( \
		"<Err> " fmt "\n", ##__VA_ARGS__ \
	)

#ifndef DEBUG_PRINT
#define debug( fmt, ... ) ((void)0)
#else /* DEBUG_PRINT */
#define debug( fmt, ... ) lcd_print( fmt, ##__VA_ARGS__ )
#endif

static const int TOUCH_SENSOR_PORT = EV3_PORT_1;

/* コマンドコードの定義 */
enum ECommandCode {
	eCap = 0x10,
};

/* レスポンスコードの定義 */
enum EResCode {
	eCap_img = 0x1f,
	eCommand = 0x2f

};


/* コマンド(要求)パケット生成関数 */
static size_t encode_packet(uint8_t reqcode, uint16_t points[], size_t num_of_points, uint8_t **packet)
{
	uint8_t *data = NULL;
	size_t l_parameter_length = 0;
	size_t packet_size = 0;
	size_t packet_index = 0;

	if ( (packet == NULL) || (reqcode != eCap) || (num_of_points > 84) ) {
	  error("invalid argument");
	  return 0;
	}

	packet_size = 1/* Command Code */ + 1/* Parameter Length */ + l_parameter_length;

	data = (uint8_t *)malloc( packet_size );
	if ( data == NULL ) {
	  error("failed to malloc");
	  return 0;
	}
	
	/* command code */
	data[packet_index++] = reqcode;
	
	/* parameter length */
	data[packet_index++] = l_parameter_length;
	
	/* parameter */
	*packet = data;

	return packet_size;
}


static FILE *serial_open()
{
	FILE *bt = NULL;

	bt = ev3_serial_open_file(EV3_SERIAL_BT);
	if ( bt == NULL ) {
		error("failed to ev3_serial_open_file");
		return NULL;
	}

	return bt;
}

static int serial_write(FILE *bt, uint8_t *data, size_t data_len)
{
	if ( (bt == NULL) || (data == NULL) ) {
		error("invalid argument");
		return -1;
	}

	if ( fwrite(data, 1, data_len, bt) < data_len ) {
		error("failed to fwrite");
		return -1;
	}

	return 0;
}

static int serial_read(FILE *bt, uint8_t *code, uint8_t **data, size_t *data_len)
{
	uint8_t l_code;
	uint8_t l_data_len;
	uint8_t *l_data = NULL;

	if ( (bt == NULL) || (code == NULL) || (data == NULL) || (data_len == NULL) ) {
		error("invalid argument");
		return -1;
	}

	if ( fread(&l_code, sizeof(uint8_t), 1, bt) < 1 ) {
		error("failed to fread [code]");
		return -1;
	}
	
	if ( fread(&l_data_len, sizeof(uint8_t), 1, bt) < 1 ) {
		error("failed to fread [datalen]");
		return -1;
	}

	l_data = (uint8_t *)malloc(l_data_len);
	if ( l_data == NULL ) {
		error("failed to malloc");
		return -1;
	}
	if ( fread(l_data, sizeof(uint8_t), l_data_len, bt) < l_data_len ) {
		error("failed to fread [data]");
		free(l_data);
		return -1;
	}

	*code = l_code;
	*data_len = l_data_len;
	*data = l_data;

	return 0;
}

void main_task(intptr_t unused)
{
	int ret = -1;
	FILE *bt = NULL;
	uint8_t *request = NULL;
	size_t request_len = 0;
	uint8_t rescode;
	uint8_t *response = NULL;
	size_t response_len = 0;
	char *errmsg = NULL;
	size_t i = 0;

	init_lcd();
	
	lcd_print("program start.\n");

	// タッチセンサを設定
	ev3_sensor_config(TOUCH_SENSOR_PORT, TOUCH_SENSOR);

	// Bluetooth通信を開通
	bt = serial_open();
	if ( bt == NULL ) {
		error("failed to serial_open");
		goto end;
	}
	lcd_print("bluetooth connected.\n");

	// タッチセンサ押下待ち
	lcd_print("Press the touch sensor.\n");
	while(!ev3_touch_sensor_is_pressed(TOUCH_SENSOR_PORT));
	while(ev3_touch_sensor_is_pressed(TOUCH_SENSOR_PORT));

	request_len = encode_packet(eCap, NULL, 0, &request);

	if ( request_len == 0 ) {
	  error("failed to encode_packet");
	  goto end;
	}

	debug("req = ");
	for ( i = 0; i < request_len; i++ ) {
		debug("%02x", request[i]);
	}
	debug("\n");
	
	// メッセージを送信
	ret = serial_write(bt, request, request_len);

	if ( ret != 0 ) {
	  error("failed to serial_write");
	  goto end;
	}

	// 応答を受信
	ret = serial_read(bt, &rescode, &response, &response_len);
	if ( ret != 0 ) {
	  error("failed to serial_read");
	  goto end;
	}
	debug("rescode = %u\n", rescode);
	debug("resdata = ");
	for ( i = 0; i < response_len; i++ ) {
	  debug("%02x ", response[i]);
	  if(response[i] == 7){
	    debug("*");
	  }
	  if(response[i] == 3){
	    debug("*");
	  }

	}
	debug("\n");


end:
	if ( bt != NULL ) {
		fclose(bt);
		lcd_print("bluetooth disconnected.\n");
	}
	if ( request != NULL ) {
		free(request);
	}
	if ( response != NULL ) {
		free(response);
	}
	if ( errmsg != NULL ) {
		free(errmsg);
	}
	
	lcd_print("program end.\n");
	
	term_lcd();

	return;
}
