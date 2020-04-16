/*
 * -----------------------------------------
 * Serial unit test between two Serial port
 * -----------------------------------------
 *
 * (C) 2020 Eurek Elettronica
 * written by: Gianluca Renzi
 * mailto: <gianlucarenzi@eurek.it>
 *
 * This unit test can be done with serial ports (RS232 type-of) and
 * RS485 (Half Duplex - Master - Slave protocol)
 * 
 * The same software must be installed in both computers and they
 * need to have at least two serial port connected each other
 * 
 * The test is passing from few bytes, up to 4k data ranging from
 * 1200 bps up to 115200 bps back and forth.
 * 
 * If something goes wrong try to reconnect the communication changing
 * from SLAVE to MASTER
 * 
 * I hope it will cover all issues for serial programming
 * 
 * Good luck! 
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/types.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <inttypes.h>
#include <fcntl.h>
/* Include definition for RS485 ioctls: TIOCGRS485 and TIOCSRS485 */
#include <sys/ioctl.h>
#include <errno.h>
#include <termios.h>
#include "serial.h"
#include "debug.h"
#include "ec_types.h"

static int debuglevel = DBG_INFO;
static int debuglevelThread = DBG_INFO;
static int errornumbersMain = 0;
static int errornumbersThread = 0;

#define TIMER_TICK        (50 * 1000L) /* 50msec TIMER RESOLUTION */

#define TIMEOUT_THREAD_MS    (1000)
#define TIMEOUT_MAIN_MS      (5000)

// Human date/time macros
#define USEC(a)       (a)
#define MSEC(a)       (USEC(a * 1000L))
#define SEC(a)        (MSEC(a * 1000L))
#define MIN(a)        (SEC(a * 60))
#define HOUR(a)       (MIN(a * 60))
#define DAY(a)        (HOUR(a * 24))

static pthread_mutex_t mutexLock;

//
// Definizioni degli stati
//
typedef enum {
	STATE_START = 0,

	// SLAVE STATES
	STATE_WAIT_COMMAND,
	STATE_COMMAND_RECEIVED,
	STATE_SEND_COMMAND_ACK,
	STATE_WAIT_SERIAL_PACKET_SIGNATURE,
	STATE_READ_SERIAL_PACKET,
	STATE_WRITE_SERIAL_PACKET_ACK,
	STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE,

	// MASTER STATES
	STATE_SEND_COMMAND,
	STATE_WAIT_COMMAND_ACK,
	STATE_WRITE_SERIAL_PACKET,
	STATE_WAIT_SERIAL_PACKET_ACK,
	STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER,
	STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE,

	// ISSUE STATES
	STATE_RESET_SERIAL,
	STATE_RESET,
	STATE_LAST, // Deve essere l'ultimo!
} t_state;

static const char *state_name[] = {
	[STATE_START] = "STATE_START",

	// SLAVE STATES
	[STATE_WAIT_COMMAND] = "STATE_WAIT_COMMAND",
	[STATE_COMMAND_RECEIVED] = "STATE_COMMAND_RECEIVED",
	[STATE_READ_SERIAL_PACKET] = "STATE_READ_SERIAL_PACKET",
	[STATE_WAIT_SERIAL_PACKET_SIGNATURE] = "STATE_WAIT_SERIAL_PACKET_SIGNATURE",
	[STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE] = "STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE",
	[STATE_WRITE_SERIAL_PACKET_ACK] = "STATE_WRITE_SERIAL_PACKET_ACK",
	[STATE_SEND_COMMAND_ACK] = "STATE_SEND_COMMAND_ACK",

	// MASTER STATES
	[STATE_SEND_COMMAND] = "STATE_SEND_COMMAND",
	[STATE_WAIT_COMMAND_ACK] = "STATE_WAIT_COMMAND_ACK",
	[STATE_WRITE_SERIAL_PACKET] = "STATE_WRITE_SERIAL_PACKET",
	[STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER] = "STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER",
	[STATE_WAIT_SERIAL_PACKET_ACK] = "STATE_WAIT_SERIAL_PACKET_ACK",
	[STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE] = "STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE",

	[STATE_RESET_SERIAL] = "STATE_RESET_SERIAL",
	[STATE_RESET] = "STATE_RESET",
	[STATE_LAST] = "STATE_LAST",
};

#define SERIAL_SIGNATURE_HEADER  0x12345678
#define SERIAL_SIGNATURE_FOOTER  0xdeadbeef
typedef struct {
	uint32_t header;
	uint32_t len;
	uint32_t footer;
} t_signature;

typedef struct {
	int fd;
	int baudrate;
	int pre;
	int post;
} t_port;

#define BUFFER_SIZE (4096)

static void print_hex_ascii_line(const unsigned char *payload, int len, int offset)
{
	int i;
	int gap;
	const unsigned char *ch;

	// offset
	printR("%05x   ", offset);

	// hex
	ch = payload;
	for (i = 0; i < len; i++)
	{
		printR("%02x ", *ch);
		ch++;
		// print extra space after 8th byte for visual aid
		if (i == 7)
			printR(" ");
	}
	// print space to handle line less than 8 bytes
	if (len < 8)
	{
		printR(" ");
	}

	// fill hex gap with spaces if not full line
	if (len < 16)
	{
		gap = 16 - len;
		for (i = 0; i < gap; i++)
		{
			printR("   ");
		}
	}
	printR("   ");

	// ascii (if printable)
	ch = payload;
	for(i = 0; i < len; i++)
	{
		if (isprint(*ch))
		{
			printR("%c", *ch);
		}
		else
		{
			printR(".");
		}
		ch++;
	}
	printR("\n");
}

// print packet payload data (avoid printing binary data)
void print_payload(const char *func, const unsigned char *payload, int len, int dbglvl)
{
	int len_rem = len;
	int line_width = 16;           // number of bytes per line
	int line_len;
	int offset = 0;                // zero-based offset counter
	const unsigned char *ch = payload;

	// Stampiamo il payload se siamo >= DBG_VERBOSE oppure
	// in errore.
	if ((dbglvl >= DBG_VERBOSE) || (dbglvl == DBG_ERROR))
	{
		printR("Enter %p LEN: %d from %s\n", payload, len, func);

		if (len <= 0)
		{
			printR("No LEN. Exit\n");
			return;
		}

		// data fits on one line
		if (len <= line_width)
		{
			print_hex_ascii_line(ch, len, offset);
			printR("Small Line. Exiting\n");
			return;
		}

		// data spans multiple lines
		for (;;)
		{
			// compute current line length
			line_len = line_width % len_rem;
			// print line
			print_hex_ascii_line(ch, line_len, offset);
			// compute total remaining
			len_rem = len_rem - line_len;
			// shift pointer to remaining bytes to print
			ch = ch + line_len;
			// add offset
			offset = offset + line_width;
			// check if we have line width chars or less
			if (len_rem <= line_width)
			{
				// print last line and get out
				print_hex_ascii_line(ch, len_rem, offset);
				break;
			}
		}
		printR("Exit\n");
	}
}

static inline void fill(unsigned char *buf, const char *str, size_t len, int times)
{
	int z;
	for (z = 0; z < times; z++)
	{
		strncpy((char *) buf, (char *) str, len);
		buf += len;
	}
}

static const char *str = "0123456789ABCDEFABCDEFGHIJKLMNOPQRSTUVWXYZ[]=-,.";

static inline int bufferlen(int baudrate)
{
	int rval;

	// La regola e' piu' andiamo veloci piu' scriviamo (potrebbe essere anche l'opposto)
	switch (baudrate)
	{
		case 230400:
			rval = 70;
			break;
		case 115200:
			rval = 45;
			break;
		case 57600:
			rval = 42;
			break;
		case 38400:
			rval = 40;
			break;
		case 19200:
			rval = 38;
			break;
		case 9600:
			rval = 35;
			break;
		case 4800:
			rval = 30;
			break;
		case 2400:
			rval = 20;
			break;
		default:
		case 1200:
			rval = 10;
			break;
	}
	return rval;
}

static inline void fillbuffer(unsigned char *buf, size_t len, int baudrate)
{
	int l = strlen(str);
	(void) len; // Avoid GCC warning

	// Questa funzione essendo chiamata da entrambi i thread, e' meglio
	// controllarla tramite il mutex
	pthread_mutex_lock(&mutexLock);
	fill(buf, str, l, bufferlen(baudrate));
	pthread_mutex_unlock(&mutexLock);
	//print_hex_ascii_line(buf,strlen( (const char *) buf), 0);
}

static void *serial_2_pthread(void *data)
{
	t_port port = *((t_port *) data);
	int serfd = port.fd;
	int baudrate2 = port.baudrate;

	t_state state = STATE_RESET;
	t_state state_next = STATE_LAST;
	unsigned char sbufferread[BUFFER_SIZE];
	unsigned char sbufferwrite[BUFFER_SIZE];
	long timeout = TIMEOUT_THREAD_MS;
	int rval = 0;
	int pre, post;
	t_signature signatureread;
	signatureread.header = SERIAL_SIGNATURE_HEADER;
	signatureread.footer = SERIAL_SIGNATURE_FOOTER;
	signatureread.len = 0;

	t_signature signaturewrite;
	signaturewrite.header = SERIAL_SIGNATURE_HEADER;
	signaturewrite.footer = SERIAL_SIGNATURE_FOOTER;
	signaturewrite.len = 0;

	int goodpackettx = 0;
	int goodpacketrx = 0;

	pre = port.pre;
	post = port.post;

	THREAD_NOISY("Enter: Port: fd: %d - BaudRate: %d PRE: %d - POST: %d\n",
		serfd, baudrate2, pre, post);

	for (;;)
	{
		switch (state)
		{
			case STATE_START:
				THREAD_NOISY("STATE_START\n");
				serial_flush_rx(serfd);
				serial_flush_tx(serfd);
				state_next = STATE_WAIT_COMMAND;
				break;

			// SLAVE STATES
			case STATE_WAIT_COMMAND:
				// Ci sono caratteri da leggere entro 15 secondi!
				// i 15 secondi possono aumentare o diminuire a seconda
				// del livello raggiunto dal test
				rval = serial_read_string(serfd, sbufferread, timeout);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on WAITING COMMAND\n");
						state_next = STATE_RESET;
					}
					// Se siamo stati interrotti ci riproviamo
				}
				else
				{
					if (rval == 0)
					{
						THREAD_NOISY("Nothing to read within %ld msecs\n", timeout);
						THREAD_VERBOSE("\t\t*** NOW MASTER ***\n");
						state_next = STATE_SEND_COMMAND;
					}
					else
					{
						THREAD_NOISY("Read %d from serial.\n", rval);
						state_next = STATE_COMMAND_RECEIVED;
					}
				}
				break;

			case STATE_COMMAND_RECEIVED:
				THREAD_NOISY("STATE_COMMAND_RECEIVED\n");
				if (strcmp("DOSLAVE\r\n", (const char *) sbufferread) == 0)
				{
					THREAD_NOISY("DO SLAVE RECEIVED. SENDING ACK\n");
					state_next = STATE_SEND_COMMAND_ACK;
				}
				else
				{
					// Ho ricevuto caratteri spuri. E' un problema, ritorno
					// allo stato di MASTER...
					THREAD_NOISY("UNKNOWN COMMAND / JUNK RECEIVED\n");
					THREAD_VERBOSE("\t\t*** NOW MASTER ***\n");
					serial_device_status(serfd);
					state_next = STATE_SEND_COMMAND;
				}
				break;

			case STATE_SEND_COMMAND_ACK:
				THREAD_NOISY("SENDING DOSLAVE CMD ACK\n");
				rval = serial_send_string(serfd, (const unsigned char *) "DOSLAVECMDACK\r\n");
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on SEND COMMAND ACK\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_VERBOSE("*** STATE_SEND_COMMAND_ACK NOT SENDING? Retry ***\n");
					}
					else
					{
						THREAD_NOISY("Switching STATE_WAIT_SERIAL_PACKET_SIGNATURE FROM MASTER\n");
						state_next = STATE_WAIT_SERIAL_PACKET_SIGNATURE;
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_SIGNATURE:
				rval = serial_read_raw(serfd, (unsigned char *) &signatureread, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on WAIT SERIAL PACKET SIGNATURE\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_NOISY("*** NOTHING TO READ/SIGNATURE ***\n");
						state_next = STATE_RESET;
					}
					else
					{
						DBG_N("SIGNATURE PACKET RECIVED FROM MASTER\n");
						if (rval != sizeof(t_signature))
						{
							THREAD_ERROR("RVAL: %d -- BAD SIGNATURE STATE_WAIT_SERIAL_PACKET_SIGNATURE:"
									"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
								rval, signatureread.header, signatureread.len, signatureread.footer);
							serial_device_status(serfd);
							state_next = STATE_RESET;
							errornumbersThread++;
						}
						else
						{
							THREAD_NOISY("STATE_WAIT_SERIAL_PACKET_SIGNATURE:"
									"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
								signatureread.header, signatureread.len, signatureread.footer);
							state_next = STATE_READ_SERIAL_PACKET;
						}
					}
				}
				break;

			case STATE_READ_SERIAL_PACKET:
				THREAD_NOISY("STATE_READ_SERIAL_PACKET\n");
				// Verifichiamo la validita' della signature ricevuta.
				// Dobbiamo metterci il meno possibile perche' i dati
				// stanno arrivando dalla seriale. E tra la lettura della
				// firma ad adesso ho gia' perso almeno 12 millisecondi
				// che e' il TIMER_TICK
				if (signatureread.header == SERIAL_SIGNATURE_HEADER &&
					signatureread.footer == SERIAL_SIGNATURE_FOOTER)
				{
					// La firma ricevuta va bene, leggiamo tutto il contenuto
					// del pacchetto
					rval = serial_read_raw(serfd, sbufferread, signatureread.len);
					if (rval < 0)
					{
						if (errno != EAGAIN && errno != EINTR)
						{
							THREAD_ERROR("Error on STATE_READ_SERIAL_PACKET\n");
							state_next = STATE_RESET;
							errornumbersThread++;
						}
					}
					else
					{
						if (rval == 0)
						{
							THREAD_NOISY("*** NOTHING TO READ ***\n");
							state_next = STATE_RESET;
						}
						else
						{
							THREAD_NOISY("STATE_READ_SERIAL_PACKET FROM MASTER\n\tRead: %d -- To Read: %d\n", rval, signatureread.len);
							if (rval != (int) signatureread.len)
							{
								THREAD_ERROR("BAD STATE_READ_SERIAL_PACKET LEN\n");
								serial_device_status(serfd);
								state_next = STATE_RESET;
								errornumbersThread++;
							}
							else
							{
								// Adesso ho letto tutto, rispediamo la firma indietro
								// e tutto il pacchetto al chiamante!
								THREAD_NOISY("STATE_READ_SERIAL_PACKETREAD\n");
								state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE;
							}
						}
					}
				}
				else
				{
					THREAD_ERROR("STATE_READ_SERIAL_PACKET: BAD SIGNATURE RECEIVED\n");
					serial_device_status(serfd);
					state_next = STATE_RESET;
					errornumbersThread++;
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE:
				THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_SIGNATURE\n");
				signaturewrite.header = signatureread.header;
				signaturewrite.len = signatureread.len;
				signaturewrite.footer = signatureread.footer;
				rval = serial_send_raw(serfd, (const unsigned char *) &signaturewrite, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("STATE_WRITE_SERIAL_PACKET_SIGNATURE ERROR\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					switch (rval)
					{
						case 0:
							THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_SIGNATURE NOT WRITE. RETRY\n");
							break;

						case sizeof(t_signature):
							// Ho scritto la firma, adesso il prima possibile
							// scrivo tutto il resto!
							THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_SIGNATURE OK\n");
							state_next = STATE_WRITE_SERIAL_PACKET_ACK;
							break;

						default:
							THREAD_ERROR("STATE_WRITE_SERIAL_PACKET_SIGNATURE not writing everything: %d\n",
								rval);
							state_next = STATE_RESET;
							errornumbersThread++;
							break;
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_ACK:
				// Il messaggio di risposta al pacchetto ricevuto,
				// e' lo stesso pacchetto...
				THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_ACK --- SAME PACKET BACK!\n");
				memcpy(sbufferwrite, sbufferread, signatureread.len);
				rval = serial_send_raw(serfd, sbufferwrite, signatureread.len);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on WRITING SERIAL PACKET ACK\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_ERROR("*** NOT WRITING - Retry ***\n");
					}
					else
					{
						THREAD_PRINT("SENT PACKET ACK FROM SLAVE OK: %d\n", goodpacketrx++);
						state_next = STATE_WAIT_SERIAL_PACKET_SIGNATURE;
					}
				}
				break;

			// MASTER STATES
			case STATE_SEND_COMMAND:
				THREAD_NOISY("STATE_SEND_COMMAND\n");
				rval = serial_send_string(serfd, (const unsigned char *) "DOSLAVE\r\n");
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on SEND COMMAND DO SLAVE r:%d -- e: %d\n", rval, errno);
						state_next = STATE_RESET_SERIAL;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_VERBOSE("Why NOT SENDING? Retry\n");
					}
					else
					{
						THREAD_NOISY("Switching to WAITING CMD ACK FROM SLAVE\n");
						state_next = STATE_WAIT_COMMAND_ACK;
					}
				}
				break;

			case STATE_WAIT_COMMAND_ACK:
				THREAD_NOISY("STATE_WAIT_COMMAND_ACK\n");
				// se stiamo andando a 1200bps la stringa di 32 caratteri
				// arriva in poco meno di 400 msec. mettiamoci anche
				// un tempo di elaborazione di altri 400 msec. Totale: 800
				rval = serial_read_string(serfd, sbufferread, 800);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("Error on STATE_WAIT_COMMAND_ACK\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_VERBOSE("TIMEOUT ERROR. RESET\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
					else
					{
						if (strcmp("DOSLAVECMDACK\r\n", (const char *) sbufferread) == 0)
						{
							THREAD_NOISY("DO SLAVE CMD ACKNOWLEDGED.\n");
							state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER;
						}
						else
						{
							THREAD_ERROR("GARBAGE/JUNK ON RECEIVING WAIT CMD ACK %s\n", sbufferread);
							serial_device_status(serfd);
							state_next = STATE_RESET;
							errornumbersThread++;
						}
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER:
				// Prima di scrivere il pacchetto, occorre preparare la signature
				// corretta...
				signaturewrite.header = SERIAL_SIGNATURE_HEADER;
				signaturewrite.footer = SERIAL_SIGNATURE_FOOTER;
				signaturewrite.len = bufferlen(baudrate2);
				THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER:"
					"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
					signaturewrite.header, signaturewrite.len, signaturewrite.footer);
				rval = serial_send_raw(serfd, (const unsigned char *) &signaturewrite, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EINTR && errno != EAGAIN)
					{
						THREAD_ERROR("STATE_WRITE_SERIAL_PACKET_SIGNATURE! Unable to write data!\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_NOISY("Timeout STATE_WRITE_SERIAL_PACKET_SIGNATURE. Wait...\n");
						// Retry
					}
					else
					{
						THREAD_NOISY("STATE_WRITE_SERIAL_PACKET_SIGNATURE OK\n");
						state_next = STATE_WRITE_SERIAL_PACKET;
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET:
				THREAD_NOISY("STATE_WRITE_SERIAL_PACKET\n");
				fillbuffer(sbufferwrite, sizeof(sbufferwrite), baudrate2);
				rval = serial_send_raw(serfd, (const unsigned char *) sbufferwrite, signaturewrite.len);
				if (rval < 0)
				{
					if (errno != EINTR && errno != EAGAIN)
					{
						THREAD_ERROR("STATE_WRITE_SERIAL_PACKET! Unable to write data!\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_NOISY("Timeout STATE_WRITE_SERIAL_PACKET. Wait...\n");
						// Retry write
					}
					else
					{
						if (rval == (int) signaturewrite.len)
						{
							THREAD_NOISY("STATE_WRITE_SERIAL_PACKET OK.\n");
							state_next = STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE;
						}
						else
						{
							THREAD_ERROR("STATE_WRITE_SERIAL_PACKET Error\n");
							serial_device_status(serfd);
							state_next = STATE_RESET;
							errornumbersThread++;
						}
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE:
				// Aspettiamo la firma dallo slave...
				THREAD_NOISY("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE\n");
				rval = serial_read_raw(serfd, (unsigned char *) &signatureread, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EINTR || errno != EAGAIN)
					{
						THREAD_ERROR("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE ERROR on reading!\n");
						state_next = STATE_RESET;
						errornumbersThread++;
					}
				}
				else
				{
					if (rval == 0)
					{
						THREAD_ERROR("Timeout STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE. Check SLAVE\n");
						serial_device_status(serfd);
						state_next = STATE_RESET;
						errornumbersThread++;
					}
					else
					{
						if (rval != sizeof(t_signature))
						{
							THREAD_ERROR("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE ERROR\n");
							state_next = STATE_RESET;
							serial_device_status(serfd);
							errornumbersThread++;
						}
						else
						{
							THREAD_NOISY("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE OK\n");
							state_next = STATE_WAIT_SERIAL_PACKET_ACK;
						}
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_ACK:
				THREAD_NOISY("STATE_WAIT_SERIAL_PACKET_ACK\n");
				// Proseguiamo nella lettura del pacchetto solo
				// se quello che abbiamo ricevuto ha l'header uguale
				// a quello che abbiamo spedito
				if (memcmp((unsigned char *) &signatureread, (unsigned char *) &signaturewrite, sizeof(t_signature)) == 0)
				{
					THREAD_NOISY("STATE_WAIT_SERIAL_PACKET_ACK SIGNATURE OK.\n");
					rval = serial_read_raw(serfd, sbufferread, signatureread.len);
					if (rval < 0)
					{
						if (errno != EAGAIN && errno != EINTR)
						{
							THREAD_ERROR("ERROR: STATE_WAIT_SERIAL_PACKET_ACK\n");
							state_next = STATE_RESET;
							errornumbersThread++;
						}
					}
					else
					{
						if (rval == 0)
						{
							THREAD_ERROR("STATE_WAIT_SERIAL_PACKET_ACK TIMEOUT\n");
							state_next = STATE_RESET;
							errornumbersThread++;
						}
						else
						{
							if (rval == (int) signatureread.len)
							{
								// Abbiamo letto tutto il pacchetto,
								// verifichiamo che sia corretto!
								if (memcmp(sbufferread, sbufferwrite, signatureread.len) == 0)
								{
									goodpackettx++;
									THREAD_PRINT("STATE_WAIT_SERIAL_PACKET_ACK Good Packet: %d\n", goodpackettx);
									state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER;
								}
								else
								{
									THREAD_ERROR("ERROR ON STATE_WAIT_SERIAL_PACKET_ACK\n");
									state_next = STATE_RESET;
									serial_device_status(serfd);
									errornumbersThread++;
								}
							}
							else
							{
								THREAD_ERROR("STATE_WAIT_SERIAL_PACKET_ACK ERROR ON READING PACKET\n");
								state_next = STATE_RESET;
								serial_device_status(serfd);
								errornumbersThread++;
							}
						}
					}
				}
				else
				{
					THREAD_ERROR("STATE_WAIT_SERIAL_PACKET_ACK WRONG SIGNATURE\n");
					state_next = STATE_RESET;
					errornumbersThread++;
				} 
				break;

			// ISSUE STATES
			case STATE_RESET_SERIAL:
				THREAD_NOISY("STATE_RESET_SERIAL\n");
				rval = serial_device_reset(serfd, baudrate2, pre, post);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						THREAD_ERROR("STATE_RESET_SERIAL ERROR\n");
						errornumbersThread++;
						goto outThread;
					}
				}
				else
				{
					THREAD_NOISY("PORT RESETTED TO DEFAULT\n");
				}
				state_next = STATE_RESET;
				break;

			case STATE_RESET:
				// Ogni volta che c'e' un'errore riduco il tempo di
				// attesa...
				if (timeout > 1000) timeout -= 1000; else timeout = TIMEOUT_THREAD_MS;
				memset(sbufferread, 0, sizeof(sbufferread));
				memset(sbufferwrite, 0, sizeof(sbufferwrite));
				memset(&signatureread, 0, sizeof(t_signature));
				memset(&signaturewrite, 0, sizeof(t_signature));
				state_next = STATE_START;
				goodpacketrx = 0;
				goodpackettx = 0;
				break;

			case STATE_LAST:
				// Deve essere l'ultimo!
				break;
		}

		if (state != state_next)
		{
			THREAD_NOISY("<LOOP> Changing state from %s to %s\n", state_name[state], state_name[state_next]);
			state = state_next;
		}
		// Non consumiamo troppa CPU!!
		usleep(TIMER_TICK);
	}

outThread:
	// cleanup
	THREAD_NOISY("Exit\n");
	return NULL;
}


//static void timerstart(void)
//{
//	gettimeofday( &startTimer, NULL);
//	DBG_N("TimerStart @ sec: %ld usec: %ld\n", startTimer.tv_sec,
//		startTimer.tv_usec);
//}

//static int timerelapsed(int timeout)
//{
//	struct timeval now;
//	DBG_N("Enter %ld - Timeout: %ld\n",
//		startTimer.tv_sec, startTimer.tv_sec + timeout);
//	// begin
//	gettimeofday(&now, NULL);
//	DBG_N("Now:     %ld\n", now.tv_sec);
//	if ((now.tv_sec) > (startTimer.tv_sec + timeout))
//		return 1;
//	else
//		return 0;
//}

extern const char *fwBuild;

static void banner(void)
{
	/* La versione viene automaticamente generata ad ogni build.
	 * Nel momento di OK del software, viene committata e il build e'
	 * lanciato con il flag 'prod' che identifica che si vuole usare
	 * il file version.c presente (quindi committato)
	 */
	fprintf(stdout, "\n\n");
	fprintf(stdout, ANSI_BLUE "TEST UNIT FOR SERIAL DEVICES" ANSI_RESET "\n");
	fprintf(stdout, ANSI_YELLOW );
	fprintf(stdout, "FWVER: %s", fwBuild);
	fprintf(stdout, ANSI_RESET "\n");
	fprintf(stdout, "\n\n");
}

static void signal_handle(int sig)
{
	char signame[8];

	switch( sig )
	{
		case SIGSEGV:
			sprintf(signame, "SIGSEGV");
			DBG_E("signal %s - %d caught\n", signame, sig);
			break;
		case SIGINT:
			sprintf(signame, "SIGINT");
			DBG_E("signal %s - %d caught\n", signame, sig);
			DBG_E("ErrorMain %d - ErrorThread %d\n", errornumbersMain, errornumbersThread);
			break;
		case SIGTERM:
			sprintf(signame, "SIGTERM");
			DBG_E("signal %s - %d caught\n", signame, sig);
			DBG_E("ErrorMain %d - ErrorThread %d\n", errornumbersMain, errornumbersThread);
			pthread_mutex_lock(&mutexLock);
			pthread_mutex_unlock(&mutexLock);
			break;
		case SIGUSR1:
			return;
			break; // NEVERREACHED
		case SIGUSR2:
			return;
			break; // NEVERREACHED
		default:
			sprintf(signame, "UNKNOWN");
			DBG_E("signal %s - %d caught\n", signame, sig);
			break;
	}
	pthread_mutex_destroy(&mutexLock);
	exit(sig);
}

static void version(const char * filename, const char *ver)
{
	char version[256];

	if (filename == NULL || ver == NULL)
		return;

	memset(version, 0, 256);

	sprintf(version, "mkdir -p /tmp/%s && echo %s > /tmp/%s/version",
			filename, ver, filename);
	if (system(version) != 0)
	{
		DBG_E("Error: %s gives error %d\n", version, errno);
	}
}

static int baud_rate_test[] = {
	38400, 1200, 19200, 2400, 115200, 4800, 57600, 4800, 38400, 9600, 230400, -1 };


int main(int argc, char *argv[])
{
	int ser1fd = -1;                // serial 1 file descriptor handle
	int ser2fd = -1;                // serial 2 file descriptor handle
	int serfd = -1;
	int baudrate1;
	int baudrate2;
	t_state state = STATE_RESET;
	t_state state_next = STATE_LAST;
	unsigned char sbufferread[BUFFER_SIZE];
	unsigned char sbufferwrite[BUFFER_SIZE];
	long timeout = TIMEOUT_MAIN_MS;
	int rval = 0;
	char device1[1024];
	char device2[1024];
	int goodpackettx = 0;
	int goodpacketrx = 0;
	pthread_t serial2Thread;
	int theThread;
	int pre1, pre2;
	int post1, post2;
	int pre, post;
	t_port port1;
	t_port port2;

	t_signature signatureread;
	signatureread.header = SERIAL_SIGNATURE_HEADER;
	signatureread.footer = SERIAL_SIGNATURE_FOOTER;
	signatureread.len = 0;

	t_signature signaturewrite;
	signaturewrite.header = SERIAL_SIGNATURE_HEADER;
	signaturewrite.footer = SERIAL_SIGNATURE_FOOTER;
	signaturewrite.len = 0;

	// avoid gcc warning
	argv = argv;
	argc = argc;

	version(argv[0], fwBuild);
	banner();

	// Adesso posso istanziare l'handle dei segnali che utilizza il mutex
	signal(SIGSEGV, signal_handle);
	signal(SIGINT, signal_handle);
	signal(SIGTERM, signal_handle);
	signal(SIGUSR1, signal_handle);
	signal(SIGUSR2, signal_handle);

	// Arguments check
	if (argc > 1) sprintf(device1, "%s", argv[1]); else sprintf(device1, "/dev/ttyUSB0");
	if (argc > 2) sprintf(device2, "%s", argv[2]); else sprintf(device2, "/dev/ttyUSB1");
	if (argc > 3) { rval = strtoul(argv[3], NULL, 10);
		rval = rval % ArraySize(baud_rate_test); // Limit the index to the array size
		baudrate1 = baud_rate_test[ rval ]; } else baudrate1 = 115200;
	if (argc > 4) { rval = strtoul(argv[4], NULL, 10);
		rval = rval % ArraySize(baud_rate_test); // Limit the index to the array size
		baudrate2 = baud_rate_test[ rval ]; } else baudrate2 = 9600;
	// Parametri di attesa pre-post
	if (argc > 5) { rval = strtoul(argv[5], NULL, 10);
		pre1 = rval; } else pre1 = 0;
	if (argc > 6) { rval = strtoul(argv[6], NULL, 10);
		post1 = rval; } else post1 = 0;
	if (argc > 7) { rval = strtoul(argv[7], NULL, 10);
		pre2 = rval; } else pre2 = 0;
	if (argc > 8) { rval = strtoul(argv[8], NULL, 10);
		post2 = rval; } else post2 = 0;


	for (rval = 0; rval < (int) ArraySize(baud_rate_test); rval++)
	{
		if (baud_rate_test[rval] == -1)
			break;
		DBG_I("BaudRate: %7d -- Index: %2d\n", baud_rate_test[rval], rval);
	}

	DBG_I("Using %s as device 1 @ BaudRate: %d - PRE: %d - POST: %d...\n",
		device1, baudrate1, pre1, post1);
	DBG_I("Using %s as device 2 @ BaudRate: %d - PRE: %d - POST: %d...\n",
		device2, baudrate2, pre2, post2);

	port1.fd = serial_device_init(device1, baudrate1, pre1, post1);
	if (port1.fd < 0)
	{
		DBG_E("Unable to initialize port 1 for device %s\n", device1);
		return -1;
	}
	else
	{
		serfd = port1.fd;
		port1.baudrate = baudrate1;
		pre = pre1;
		post = post1;
	}

	ser2fd = serial_device_init(device2, baudrate2, pre2, post2);
	if (ser2fd < 0)
	{
		DBG_E("Unable to initialize port 2 for device %s\n", device2);
		return -1;
	}
	else
	{
		port2.fd = ser2fd;
		port2.baudrate = baudrate2;
		port2.pre = pre2;
		port2.post = post2;
	}

	DBG_I("Creating mutexLock\n");
	if (pthread_mutex_init(&mutexLock, NULL) != 0)
	{
		DBG_E("Cannot create thread lock (mutexLock)\n");
		return -1;
	}
	else
	{
		DBG_N("Thread MUTEX Created\n");
	}

	DBG_I("Initialize pthread\n");
	theThread = pthread_create( &serial2Thread, NULL, serial_2_pthread, &port2 );
	if (theThread < 0)
	{
		DBG_E("Cannot create thread modem\n");
		goto out;
	}

	DBG_N("START STATE MACHINE\n");

	for (;;)
	{
		switch (state)
		{
			case STATE_START:
				DBG_N("STATE_START\n");
				serial_flush_rx(serfd);
				serial_flush_tx(serfd);
				state_next = STATE_WAIT_COMMAND;
				break;

			// SLAVE STATES
			case STATE_WAIT_COMMAND:
				// Ci sono caratteri da leggere entro 15 secondi!
				// i 15 secondi possono aumentare o diminuire a seconda
				// del livello raggiunto dal test
				rval = serial_read_string(serfd, sbufferread, timeout);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on WAITING COMMAND\n");
						state_next = STATE_RESET;
					}
					// Se siamo stati interrotti ci riproviamo
				}
				else
				{
					if (rval == 0)
					{
						DBG_N("Nothing to read within %ld msecs\n", timeout);
						DBG_V("\t\t*** NOW MASTER ***\n");
						state_next = STATE_SEND_COMMAND;
					}
					else
					{
						DBG_N("Read %d from serial.\n", rval);
						state_next = STATE_COMMAND_RECEIVED;
					}
				}
				break;

			case STATE_COMMAND_RECEIVED:
				DBG_N("STATE_COMMAND_RECEIVED\n");
				if (strcmp("DOSLAVE\r\n", (const char *) sbufferread) == 0)
				{
					DBG_N("DO SLAVE RECEIVED. SENDING ACK\n");
					state_next = STATE_SEND_COMMAND_ACK;
				}
				else
				{
					// Ho ricevuto caratteri spuri. E' un problema, ritorno
					// allo stato di MASTER...
					DBG_N("UNKNOWN COMMAND / JUNK RECEIVED\n");
					DBG_V("\t\t*** NOW MASTER ***\n");
					serial_device_status(serfd);
					state_next = STATE_SEND_COMMAND;
				}
				break;

			case STATE_SEND_COMMAND_ACK:
				DBG_N("SENDING DOSLAVE CMD ACK\n");
				rval = serial_send_string(serfd, (const unsigned char *) "DOSLAVECMDACK\r\n");
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on SEND COMMAND ACK\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_V("*** STATE_SEND_COMMAND_ACK NOT SENDING? Retry ***\n");
					}
					else
					{
						DBG_N("Switching STATE_WAIT_SERIAL_PACKET_SIGNATURE FROM MASTER\n");
						state_next = STATE_WAIT_SERIAL_PACKET_SIGNATURE;
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_SIGNATURE:
				rval = serial_read_raw(serfd, (unsigned char *) &signatureread, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on WAIT SERIAL PACKET SIGNATURE\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_N("*** NOTHING TO READ/SIGNATURE ***\n");
						state_next = STATE_RESET;
					}
					else
					{
						DBG_N("SIGNATURE PACKET RECIVED FROM MASTER\n");
						if (rval != sizeof(t_signature))
						{
							DBG_E("RVAL: %d -- BAD SIGNATURE STATE_WAIT_SERIAL_PACKET_SIGNATURE:"
									"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
								rval, signatureread.header, signatureread.len, signatureread.footer);
							state_next = STATE_RESET;
							serial_device_status(serfd);
							errornumbersMain++;
						}
						else
						{
							DBG_N("STATE_WAIT_SERIAL_PACKET_SIGNATURE:"
									"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
								signatureread.header, signatureread.len, signatureread.footer);
							state_next = STATE_READ_SERIAL_PACKET;
						}
					}
				}
				break;

			case STATE_READ_SERIAL_PACKET:
				DBG_N("STATE_READ_SERIAL_PACKET\n");
				// Verifichiamo la validita' della signature ricevuta.
				// Dobbiamo metterci il meno possibile perche' i dati
				// stanno arrivando dalla seriale. E tra la lettura della
				// firma ad adesso ho gia' perso almeno 12 millisecondi
				// che e' il TIMER_TICK
				if (signatureread.header == SERIAL_SIGNATURE_HEADER &&
					signatureread.footer == SERIAL_SIGNATURE_FOOTER)
				{
					// La firma ricevuta va bene, leggiamo tutto il contenuto
					// del pacchetto
					rval = serial_read_raw(serfd, sbufferread, signatureread.len);
					if (rval < 0)
					{
						if (errno != EAGAIN && errno != EINTR)
						{
							DBG_E("Error on STATE_READ_SERIAL_PACKET\n");
							state_next = STATE_RESET;
							errornumbersMain++;
						}
					}
					else
					{
						if (rval == 0)
						{
							DBG_N("*** NOTHING TO READ ***\n");
							state_next = STATE_RESET;
						}
						else
						{
							DBG_N("STATE_READ_SERIAL_PACKET FROM MASTER\n\tRead: %d -- To Read: %d\n", rval, signatureread.len);
							if (rval != (int) signatureread.len)
							{
								DBG_E("BAD STATE_READ_SERIAL_PACKET LEN\n");
								serial_device_status(serfd);
								state_next = STATE_RESET;
								errornumbersMain++;
							}
							else
							{
								// Adesso ho letto tutto, rispediamo la firma indietro
								// e tutto il pacchetto al chiamante!
								DBG_N("STATE_READ_SERIAL_PACKETREAD\n");
								state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE;
							}
						}
					}
				}
				else
				{
					DBG_E("STATE_READ_SERIAL_PACKET: BAD SIGNATURE RECEIVED\n");
					serial_device_status(serfd);
					state_next = STATE_RESET;
					errornumbersMain++;
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_SIGNATURE_SLAVE:
				DBG_N("STATE_WRITE_SERIAL_PACKET_SIGNATURE\n");
				signaturewrite.header = signatureread.header;
				signaturewrite.len = signatureread.len;
				signaturewrite.footer = signatureread.footer;
				rval = serial_send_raw(serfd, (const unsigned char *) &signaturewrite, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("STATE_WRITE_SERIAL_PACKET_SIGNATURE ERROR\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					switch (rval)
					{
						case 0:
							DBG_N("STATE_WRITE_SERIAL_PACKET_SIGNATURE NOT WRITE. RETRY\n");
							break;

						case sizeof(t_signature):
							// Ho scritto la firma, adesso il prima possibile
							// scrivo tutto il resto!
							DBG_N("STATE_WRITE_SERIAL_PACKET_SIGNATURE OK\n");
							state_next = STATE_WRITE_SERIAL_PACKET_ACK;
							break;

						default:
							DBG_E("STATE_WRITE_SERIAL_PACKET_SIGNATURE not writing everything: %d\n",
								rval);
							state_next = STATE_RESET;
							errornumbersMain++;
							break;
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_ACK:
				// Il messaggio di risposta al pacchetto ricevuto,
				// e' lo stesso pacchetto...
				DBG_N("STATE_WRITE_SERIAL_PACKET_ACK --- SAME PACKET BACK!\n");
				memcpy(sbufferwrite, sbufferread, signatureread.len);
				rval = serial_send_raw(serfd, sbufferwrite, signatureread.len);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on WRITING SERIAL PACKET ACK\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_E("*** NOT WRITING - Retry ***\n");
					}
					else
					{
						DBG_N("SENT PACKET ACK FROM SLAVE OK %d\n", goodpacketrx++);
						state_next = STATE_WAIT_SERIAL_PACKET_SIGNATURE;
					}
				}
				break;

			// MASTER STATES
			case STATE_SEND_COMMAND:
				DBG_N("STATE_SEND_COMMAND\n");
				rval = serial_send_string(serfd, (const unsigned char *) "DOSLAVE\r\n");
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on SEND COMMAND DO SLAVE r:%d -- e: %d\n", rval, errno);
						state_next = STATE_RESET_SERIAL;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_V("Why NOT SENDING? Retry\n");
					}
					else
					{
						DBG_N("Switching to WAITING CMD ACK FROM SLAVE\n");
						state_next = STATE_WAIT_COMMAND_ACK;
					}
				}
				break;

			case STATE_WAIT_COMMAND_ACK:
				DBG_N("STATE_WAIT_COMMAND_ACK\n");
				// se stiamo andando a 1200bps la stringa di 32 caratteri
				// arriva in poco meno di 400 msec. mettiamoci anche
				// un tempo di elaborazione di altri 400 msec. Totale: 800
				rval = serial_read_string(serfd, sbufferread, 800);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("Error on STATE_WAIT_COMMAND_ACK\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_V("TIMEOUT ERROR. RESET\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
					else
					{
						if (strcmp("DOSLAVECMDACK\r\n", (const char *) sbufferread) == 0)
						{
							DBG_N("DO SLAVE CMD ACKNOWLEDGED.\n");
							state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER;
						}
						else
						{
							DBG_E("GARBAGE/JUNK ON RECEIVING WAIT CMD ACK %s\n", sbufferread);
							serial_device_status(serfd);
							state_next = STATE_RESET;
							errornumbersMain++;
						}
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER:
				// Prima di scrivere il pacchetto, occorre preparare la signature
				// corretta...
				signaturewrite.header = SERIAL_SIGNATURE_HEADER;
				signaturewrite.footer = SERIAL_SIGNATURE_FOOTER;
				signaturewrite.len = bufferlen(baudrate2);
				DBG_N("STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER:"
					"\n\tHEADER: 0x%08x\n\tLEN: 0x%08x\n\tFOOTER: 0x%08x\n",
					signaturewrite.header, signaturewrite.len, signaturewrite.footer);
				rval = serial_send_raw(serfd, (const unsigned char *) &signaturewrite, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EINTR && errno != EAGAIN)
					{
						DBG_E("STATE_WRITE_SERIAL_PACKET_SIGNATURE! Unable to write data!\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_N("Timeout STATE_WRITE_SERIAL_PACKET_SIGNATURE. Wait...\n");
						// Retry
					}
					else
					{
						DBG_N("STATE_WRITE_SERIAL_PACKET_SIGNATURE OK\n");
						state_next = STATE_WRITE_SERIAL_PACKET;
					}
				}
				break;

			case STATE_WRITE_SERIAL_PACKET:
				DBG_N("STATE_WRITE_SERIAL_PACKET\n");
				fillbuffer(sbufferwrite, sizeof(sbufferwrite), baudrate2);
				rval = serial_send_raw(serfd, (const unsigned char *) sbufferwrite, signaturewrite.len);
				if (rval < 0)
				{
					if (errno != EINTR && errno != EAGAIN)
					{
						DBG_E("STATE_WRITE_SERIAL_PACKET! Unable to write data!\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_N("Timeout STATE_WRITE_SERIAL_PACKET. Wait...\n");
						// Retry write
					}
					else
					{
						if (rval == (int) signaturewrite.len)
						{
							DBG_N("STATE_WRITE_SERIAL_PACKET OK.\n");
							state_next = STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE;
						}
						else
						{
							DBG_E("STATE_WRITE_SERIAL_PACKET Error\n");
							state_next = STATE_RESET;
							errornumbersMain++;
						}
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE:
				// Aspettiamo la firma dallo slave...
				DBG_N("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE\n");
				rval = serial_read_raw(serfd, (unsigned char *) &signatureread, sizeof(t_signature));
				if (rval < 0)
				{
					if (errno != EINTR || errno != EAGAIN)
					{
						DBG_E("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE ERROR on reading!\n");
						state_next = STATE_RESET;
						errornumbersMain++;
					}
				}
				else
				{
					if (rval == 0)
					{
						DBG_E("Timeout STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE. Check SLAVE\n");
						serial_device_status(serfd);
						state_next = STATE_RESET;
						errornumbersMain++;
					}
					else
					{
						if (rval != sizeof(t_signature))
						{
							DBG_E("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE ERROR\n");
							state_next = STATE_RESET;
							serial_device_status(serfd);
							errornumbersMain++;
						}
						else
						{
							DBG_N("STATE_WAIT_SERIAL_PACKET_ACK_SIGNATURE OK\n");
							state_next = STATE_WAIT_SERIAL_PACKET_ACK;
						}
					}
				}
				break;

			case STATE_WAIT_SERIAL_PACKET_ACK:
				DBG_N("STATE_WAIT_SERIAL_PACKET_ACK\n");
				// Proseguiamo nella lettura del pacchetto solo
				// se quello che abbiamo ricevuto ha l'header uguale
				// a quello che abbiamo spedito
				if (memcmp((unsigned char *) &signatureread, (unsigned char *) &signaturewrite, sizeof(t_signature)) == 0)
				{
					DBG_N("STATE_WAIT_SERIAL_PACKET_ACK SIGNATURE OK.\n");
					rval = serial_read_raw(serfd, sbufferread, signatureread.len);
					if (rval < 0)
					{
						if (errno != EAGAIN && errno != EINTR)
						{
							DBG_E("ERROR: STATE_WAIT_SERIAL_PACKET_ACK\n");
							state_next = STATE_RESET;
							errornumbersMain++;
						}
					}
					else
					{
						if (rval == 0)
						{
							DBG_E("STATE_WAIT_SERIAL_PACKET_ACK TIMEOUT\n");
							state_next = STATE_RESET;
							errornumbersMain++;
						}
						else
						{
							if (rval == (int) signatureread.len)
							{
								// Abbiamo letto tutto il pacchetto,
								// verifichiamo che sia corretto!
								if (memcmp(sbufferread, sbufferwrite, signatureread.len) == 0)
								{
									goodpackettx++;
									DBG_I("STATE_WAIT_SERIAL_PACKET_ACK Good Packet: %d\n", goodpackettx);
									state_next = STATE_WRITE_SERIAL_PACKET_SIGNATURE_MASTER;
								}
								else
								{
									DBG_E("ERROR ON STATE_WAIT_SERIAL_PACKET_ACK\n");
									state_next = STATE_RESET;
									serial_device_status(serfd);
									errornumbersMain++;
								}
							}
							else
							{
								DBG_E("STATE_WAIT_SERIAL_PACKET_ACK ERROR ON READING PACKET\n");
								serial_device_status(serfd);
								state_next = STATE_RESET;
								errornumbersMain++;
							}
						}
					}
				}
				else
				{
					DBG_E("STATE_WAIT_SERIAL_PACKET_ACK WRONG SIGNATURE\n");
					serial_device_status(serfd);
					state_next = STATE_RESET;
					errornumbersMain++;
				} 
				break;

			// ISSUE STATES
			case STATE_RESET_SERIAL:
				DBG_N("STATE_RESET_SERIAL\n");
				rval = serial_device_reset(serfd, baudrate1, pre, post);
				if (rval < 0)
				{
					if (errno != EAGAIN && errno != EINTR)
					{
						DBG_E("STATE_RESET_SERIAL ERROR\n");
						goto out;
					}
				}
				else
				{
					DBG_N("PORT RESETTED TO DEFAULT\n");
				}
				state_next = STATE_RESET;
				break;

			case STATE_RESET:
				// Ogni volta che c'e' un'errore riduco il tempo di
				// attesa...
				if (timeout > 1000) timeout -= 1000; else timeout = TIMEOUT_MAIN_MS;
				memset(sbufferread, 0, sizeof(sbufferread));
				memset(sbufferwrite, 0, sizeof(sbufferwrite));
				memset(&signatureread, 0, sizeof(t_signature));
				memset(&signaturewrite, 0, sizeof(t_signature));
				state_next = STATE_START;
				goodpacketrx = 0;
				goodpackettx = 0;
				break;

			case STATE_LAST:
				// Deve essere l'ultimo!
				break;
		}

		if (state != state_next)
		{
			DBG_N("<LOOP> Changing state from %s to %s\n", state_name[state], state_name[state_next]);
			state = state_next;
		}
		// Non consumiamo troppa CPU!!
		usleep(TIMER_TICK);
	}

out:
	pthread_mutex_destroy(&mutexLock);

	close(ser1fd);
	close(ser2fd);
	return errornumbersMain;
}

