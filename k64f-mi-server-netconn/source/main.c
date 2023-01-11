/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

// Include para config k64
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"

// Include para printear en consola
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"

// Include para threads
#include "FreeRTOS.h"
#include "message_buffer.h"

// Include servidor
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/sys.h"
#include "ethernetif.h"
#include "smtp.h"

#include "myhtml.h"

// drivers varios
#include "led.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Media access control address used for interface configuration. */
#define MAC_ADDRESS {0x02, 0x12, 0x13, 0x10, 0x15, 0x11}		//default ejemplo SDK
#define TCP_PORT 80U /**< Listening port. */					//PORT 80 webpage
#define BUFFER_LENGTH 1400 /**< Bytes of data in TCP packets. */
#define COMMAND_SIZE 4 /**< Characters used for commands name. */
#define LED_COMMAND "led:" /**< Name of the LED command. */
#define LED_COLOR_INDEX 4 /**< Index of the color argument. */


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//void tcp_listener_thread(void * arg);

//void rx_command_check(char * buffer, uint16_t null_terminator_pos);

void led_thread(void * arg);

void stack_init_thread(void * arg); // me da la ip

uint8_t range_adjust(long value);

void http_server_netconn_init(void);

static void http_server_netconn_serve(struct netconn *conn);

static void http_server_netconn_thread(void *arg);

void my_smtp_result_fn(void *arg, u8_t smtp_result, u16_t srv_err, err_t err);

//void msg_thread(void * arg);

//void pwm_thread(void * arg);



/*******************************************************************************
 * Variables
 ******************************************************************************/

MessageBufferHandle_t led_handler; 		/**< Buffer handler of led_thread. */

uint8_t messageBuffer[BUFFER_LENGTH]; 	/**< Buffer to be used by the threads. */


char mail[50] = "fledesma@itba.edu.ar";

//TaskHandle_t tcp_listener_handle; /**< Task handle of tcp_listener_thread. */
//MessageBufferHandle_t msg_handler; /**< Buffer handler of msg_thread. */
//MessageBufferHandle_t pwm_handler; /**< Buffer handler of pwm_thread. */


/*******************************************************************************
 * const
 ******************************************************************************/

//const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
//const static char http_index_html[] = "<html><head><title>Congrats!</title></head><body><h1>Welcome to our lwIP HTTP server!</h1><p>This is a small test page, served by httpserver-netconn.</body></html>";


// Nota mega importante, los html tienen que ir con ' ' comillas simples, no va <html doctype>

//const static char http_index_colors_html[] = "<html> <head> <meta charset='UTF-8'> <meta http-equiv='X-UA-Compatible' content='IE=edge'> <meta name='viewport' content='width=device-width, initial-scale=1.0'> <title>IP CAMERA</title> </head> <body> <form method='get' action='/r'> <button type='submit'>ROJO</button> </form> <form method='get' action='/y'> <button type='submit'>Amarillo</button> </form> <form method='get' action='/b'> <button type='submit'>Azul</button> </form> <script> </script> </body> </html>";


/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void)
{
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();


	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	/* Disable Memory Protection Unit. */
	SYSMPU->CESR &= ~SYSMPU_CESR_VLD_MASK;

	/* Creation of the message buffers. */
	led_handler = xMessageBufferCreate(BUFFER_LENGTH);


	if (xTaskCreate(stack_init_thread, "stack_init_thread",								//Init stack thread
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
		{
			PRINTF("stack_init_thread creation failed.\n");
		}


	/** Initialize the HTTP server (start its thread) */
	sys_thread_new("http_server_netconn", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);



	if (xTaskCreate(led_thread, "led_thread",											// Init led thread
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("led_thread creation failed.\n");
	}


	PRINTF("All thread created succesfully\n");
	vTaskStartScheduler();

	return 0;
}


//Mail callback function
void my_smtp_result_fn(void *arg, u8_t smtp_result, u16_t srv_err, err_t err)
{
  printf("mail (%p) sent with results: 0x%02x, 0x%04x, 0x%08x\n", arg,
         smtp_result, srv_err, err);
}



/** Serve one HTTP connection accepted in the http thread */
static void
http_server_netconn_serve(struct netconn *conn)			//En cada request...
{
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  err = netconn_recv(conn, &inbuf);

  if (err == ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    PRINTF(buf);
    /* Is this an HTTP GET command? (only check the first 5 chars, since
    there are other formats for GET, and we're keeping it very simple )*/
    if (buflen>=5 &&
        buf[0]=='G' &&
        buf[1]=='E' &&
        buf[2]=='T' &&
        buf[3]==' ' &&
        buf[4]=='/' ) {

      /* Send the HTML header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
       */

	  err_t smtp_error = ERR_OK;
      switch (buf[5])
      {
		case 'r':
			turn_on_red();
			break;

		case 'g':
			turn_on_green();
			break;

		case 'b':
			turn_on_blue();
			break;

		case 'y':
			turn_on_yellow();
			break;

		case 't':
			turn_on_magenta();
			break;

		case 'c':
			turn_on_cyan();
			break;

		case 'w':
			turn_on_white();
			break;

		case 'o':
			turn_off_leds();
			break;

		case 'm':
			for(int i=0; i<50; i++)
			{
			        mail[i]=0;
			}

			int i = 0;
			do{
				mail[i] = buf[i+7];
				i++;
			}while(buf[i+7] != ' ');
			break;

		case 'n':
			smtp_set_server_port(2525);
			smtp_error = smtp_set_server_addr("mail.smtp2go.com");

			if(smtp_error == ERR_OK)
			{
				smtp_error = smtp_set_auth("bot_grupo4", "g4password");

				struct smtp_send_request smtp_send_mail_data;
				smtp_send_mail_data.from = "bot.grupo4@gmail.com";
				smtp_send_mail_data.to = mail;
				smtp_send_mail_data.subject = "ID:1234567890";
				smtp_send_mail_data.body = pablo1;
				smtp_send_mail_data.callback_fn = my_smtp_result_fn;
				smtp_send_mail_data.callback_arg = NULL;
				smtp_send_mail_data.static_data = pdFALSE;

				smtp_send_mail_int((void *)&smtp_send_mail_data);

			}
			break;

		default:
			break;
			//PRINTF("Invalid LED color.\n");
      }

      netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
      //netconn_write(conn, http_index_colors_html, sizeof(http_index_colors_html)-1, NETCONN_NOCOPY);
      netconn_write(conn, http_index_colors_html, sizeof(http_index_colors_html)-1, NETCONN_NOCOPY);



      /* Send our HTML page */
      //netconn_write(conn, http_index_html, sizeof(http_index_html)-1, NETCONN_NOCOPY);


    }
  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);

  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}




/** The main function, never returns! */
static void
http_server_netconn_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err;
  LWIP_UNUSED_ARG(arg);

  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  LWIP_ERROR("http_server: invalid conn", (conn != NULL), return;);

  /* Bind to port 80 (HTTP) with default IP address */
  netconn_bind(conn, NULL, 80);

  /* Put the connection into LISTEN state */
  netconn_listen(conn);

  do {
    err = netconn_accept(conn, &newconn);		// se queda aca esperando que le hagan un request
    if (err == ERR_OK) {
      http_server_netconn_serve(newconn);		//handle del request
      netconn_delete(newconn);
    }
  } while(err == ERR_OK);

  netconn_close(conn);
  netconn_delete(conn);
}





void stack_init_thread(void * arg)
{
	PRINTF("Obtaining IP.\n");
	PRINTF("Please wait...\n");
	struct netif netif;
	struct dhcp * dhcp;

	ip4_addr_t ip_address;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	ethernetif_config_t config =
	{
			.phyAddress = BOARD_ENET0_PHY_ADDRESS,
			.clockName = kCLOCK_CoreSysClk,
			.macAddress = MAC_ADDRESS,
	};

	/* Initialization of the TCP/IP stack. */
	tcpip_init(NULL, NULL);

	/* Set up the network interface. */
	netif_add(&netif, &ip_address, &netmask, &gateway, &config,
			ethernetif0_init, tcpip_input);
	netif_set_default(&netif);
	netif_set_up(&netif);

	/* Start DHCP negotiation. */
	dhcp_start(&netif);

	for(;;)
	{
		dhcp = netif_dhcp_data(&netif);

		if (dhcp->state == DHCP_STATE_BOUND)
		{
			PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
			PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
			PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));


			vTaskDelete(NULL);
		}
	}
}

/**
 * @brief Look for commands in char buffers.
 *
 * The first characters are compared against the current available commands,
 * if the comparison is successful a message is send to the specific thread.
 *
 * @param[in] buffer pointer to the char buffer.
 * @param[in] null_terminator_pos position in the buffer of the null terminator.
 */
//void rx_command_check(char * buffer, uint16_t null_terminator_pos)
//{
//	if (COMMAND_SIZE > null_terminator_pos)
//	{
//		PRINTF("Invalid command: too short.\n");
//	}
//	else
//	{
//		if (strncmp(buffer, LED_COMMAND, COMMAND_SIZE) == 0)
//		{
//			xMessageBufferSend(led_handler, (void *) buffer, strlen(buffer), 0);
//		}else
//		{
//			PRINTF("Invalid command.\n");
//		}
//	}
//}

/**
 * @brief Modify the color of the on board RGB LED.
 *
 * Using a letter sent as the command argument turn on one of the available
 * colors of the on board LED.
 * Also, can turn off all the LEDs.
 *
 * @param[in] arg argument to the thread.
 */
void led_thread(void *arg)
{
	char buffer[BUFFER_LENGTH];
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);

	for (;;)
	{
		xReceivedBytes = xMessageBufferReceive(led_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);

		if (xReceivedBytes)
		{
			switch (buffer[LED_COLOR_INDEX])
			{
			case 'r':
				turn_on_red();
				break;

			case 'g':
				turn_on_green();
				break;

			case 'b':
				turn_on_blue();
				break;

			case 'y':
				turn_on_yellow();
				break;

			case 'm':
				turn_on_magenta();
				break;

			case 'c':
				turn_on_cyan();
				break;

			case 'w':
				turn_on_white();
				break;

			case 'o':
				turn_off_leds();
				break;

			default:
				PRINTF("Invalid LED color.\n");
			}
		}
	}
}

/**
 * @brief Truncate values lower than 0 and higher than 100.
 *
 * A negative value becomes 0 and a positive value greater than 100 becomes 100.
 *
 * @param[in] value a number given by the user.
 * @return a value within range.
 */
//uint8_t range_adjust(long value)
//{
//	uint8_t corrected_value;
//
//	if (value < 0)
//	{
//		corrected_value = 0U;
//	}
//	else if (value > 100)
//	{
//		corrected_value = 100U;
//	}
//	else
//	{
//		corrected_value = (uint8_t) value;
//	}
//
//	return corrected_value;
//}

/*** end of file ***/
