#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/serial.h>
#include "serial.h"
#include "ec_types.h"
#include "debug.h"

static int debuglevelDriver = DBG_ERROR;

#define MAX_SERIAL_ERRNO	5

/*
 * Questa funzione restituisce:
 * il risultato del comando passatogli
 * in caso di errore viene settata la variabile globale syserror;
 * > 0 se il comando restituisce qualcosa e quello che trova e` in
 * commandoutput.
 */
//static int syserror = -ECERR_IO;
//static char sysbuffer[4096];
//static char *systemcall(const char *command)
//{
//	char syscall[512];
//	FILE *stream;
//	int rval;
//	int i;
//	char lbuff[4096];
//
//	syserror = OK;
//
//	DBG_N("Enter\n");
//	if (command == NULL)
//	{
//		DBG_E("Empty Command\n");
//		syserror = -ECERR_IO;
//		return NULL;
//	}
//	else
//	{
//		DBG_V("Command: %s\n", command);
//	}
//
//	sprintf(syscall, "%s", command);
//
//	stream = popen(syscall, "r"); /* Leggiamo lo stdout della chiamata */
//	if (!stream)
//	{
//		DBG_E("Error on popen()\n");
//		perror("Error on popen()");
//		syserror = -ECERR_IO;
//		return NULL;
//	}
//
//	memset(lbuff, 0, sizeof(lbuff));
//	rval = fread(lbuff, 1, sizeof(lbuff), stream);
//	DBG_V("fread() returns: %d\n", rval);
//	if (rval < 0)
//	{
//		DBG_E("Error on reading stream\n");
//		syserror = -ECERR_IO;
//		pclose(stream);
//		return NULL;
//	}
//	else
//	if (rval == 0)
//	{
//		DBG_V("Nothing to read\n");
//		syserror = OK;
//		pclose(stream);
//		return NULL;
//	 }
//
//	DBG_V("Command %s returns LEN: %d:\n\t\n%s", command, rval, lbuff);
//	if (debuglevel > DBG_NOISY)
//	{
//		for (i = 0; i < (rval - 1); i++)
//		{
//			printR("[%d] %c -- 0x%02x\n", i, lbuff[i], lbuff[i]);
//		}
//	}
//	memset(sysbuffer, 0, 4096);
//	/* Eliminiamo il carattere di EOL */
//	memcpy(sysbuffer, lbuff, (rval - 1));
//	DBG_N("Exit\n");
//	pclose(stream);
//	return sysbuffer;
//}

extern void print_payload(const char *func, const unsigned char *payload, int len, int dbglvl);

static void dump_raw_data(const unsigned char *buffer, int len)
{
	print_payload(__FUNCTION__, buffer, len, DBG_VERBOSE);
}

void serial_device_status(int fd)
{
	struct serial_icounter_struct icount = { 0 };
	int ret = ioctl(fd, TIOCGICOUNT, &icount);
	if (ret != -1) {
		DRIVER_ERROR("TIOCGICOUNT: ret=%i, rx=%i, tx=%i, frame = %i, overrun = %i, parity = %i, brk = %i, buf_overrun = %i\n",
			ret, icount.rx, icount.tx, icount.frame, icount.overrun, icount.parity, icount.brk, icount.buf_overrun);
	}
}

int serial_device_init(const char *name, int baudrate, int pre, int post)
{
	int fd;
	const char * devicefamily = "/dev/tty";
	int rval;

	/* Consideriamo le seriali tutte RS485! */

	if (name == NULL || baudrate < 0)
	{
		DRIVER_ERROR("BAD PARAMETER\n");
		return -ECERR_BADPARAM;
	}

	DRIVER_NOISY("Enter with: %s and %d baudrate\n", name, baudrate);

	if (strncmp(name, devicefamily, strlen(devicefamily)))
	{
		DRIVER_ERROR( "Not supported device!\n" );
		return -ENODEV;
	}

	fd = open(name, O_RDWR | O_FSYNC | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
	{
		DRIVER_ERROR("open() %d %s for %s\n", errno, strerror( errno ), name);
		perror("open");
		return -ENODEV;
	}

	rval = serial_device_reset(fd, baudrate, pre, post);
	if (rval < 0)
	{
		DRIVER_ERROR("Unable to reset device %s at baudrate %d\n",
			name, baudrate);
		close(fd);
		return -ENODEV;
	}

	DRIVER_NOISY("rval: %d -- Exit with: %d\n", rval, fd);
	return fd;
}

int serial_send_break(int fd)
{
	int rval = 0;
	DRIVER_NOISY("Enter with: FD: %d\n", fd);
	// Send BREAK
	rval = tcsendbreak(fd, 250);
	return rval;
}

int serial_device_reset(int fd, int baudrate, int pre, int post)
{
	struct termios term;
	struct serial_rs485 rs485conf;

	if (baudrate < 0 || fd < 0)
	{
		DRIVER_ERROR("BAD PARAMETER\n");
		return -ECERR_BADPARAM;
	}

	DRIVER_NOISY("Enter with: FD: %d - %d baudrate\n", fd, baudrate);

	GET_PORT_STATE(fd, &term);

	cfmakeraw(&term);

	switch (baudrate)
	{
		case 1200:
			cfsetispeed( &term, B1200 );
			cfsetospeed( &term, B1200 );
			break;
		case 2400:
			cfsetispeed( &term, B2400 );
			cfsetospeed( &term, B2400 );
			break;
		case 4800:
			cfsetispeed( &term, B4800 );
			cfsetospeed( &term, B4800 );
			break;
		case 9600:
			cfsetispeed( &term, B9600 );
			cfsetospeed( &term, B9600 );
			break;
		case 19200:
			cfsetispeed( &term, B19200 );
			cfsetospeed( &term, B19200 );
			break;
		case 38400:
			cfsetispeed( &term, B38400 );
			cfsetospeed( &term, B38400 );
			break;
		case 57600:
			cfsetispeed( &term, B57600 );
			cfsetospeed( &term, B57600 );
			break;
		case 115200:
			cfsetispeed( &term, B115200 );
			cfsetospeed( &term, B115200 );
			break;
		case 230400:
			cfsetispeed( &term, B230400 );
			cfsetospeed( &term, B230400 );
			break;
		case 1000000:
			cfsetispeed( &term, B1000000 );
			cfsetospeed( &term, B1000000 );
			break;
		case 2000000:
			cfsetispeed( &term, B2000000 );
			cfsetospeed( &term, B2000000 );
			break;
		default:
			DRIVER_ERROR("Not Supported BaudRate: %d\n", baudrate);
			return -EINVAL;
	}

	/* Impostazioni termios */
	term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
		INLCR | IGNCR | ICRNL | IXON);
	term.c_oflag &= ~OPOST;
	term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	term.c_cflag &= ~(CSIZE | PARENB);
	term.c_cflag |= CS8;
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = 0;

	term.c_cc[VINTR] = _POSIX_VDISABLE;
	term.c_cc[VQUIT] = _POSIX_VDISABLE;
	term.c_cc[VSTART] = _POSIX_VDISABLE;
	term.c_cc[VSTOP] = _POSIX_VDISABLE;
	term.c_cc[VSUSP] = _POSIX_VDISABLE;
	term.c_cc[VEOF] = _POSIX_VDISABLE;
	term.c_cc[VEOL] = _POSIX_VDISABLE;
	term.c_cc[VERASE] = _POSIX_VDISABLE;
	term.c_cc[VKILL] = _POSIX_VDISABLE;

	DRIVER_VERBOSE( "FD: %d -- VMIN: %d -- VTIME: %d\n",
		fd, term.c_cc[VMIN], term.c_cc[VTIME]);

	SET_PORT_STATE(fd, &term);

	/* 
	 * RS485 Section. If it fails, does not care!
	 */
	if (ioctl (fd, TIOCGRS485, &rs485conf) < 0) {
		perror("(R) ioctl TIOCGRS485");
	} else {
		/* Enable RS485 mode: */
		rs485conf.flags |= SER_RS485_ENABLED;

		/* Set logical level for RTS pin equal to 1 when sending: */
		/* or, set logical level for RTS pin equal to 0 when sending: */
		rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);

		/* Set logical level for RTS pin equal to 1 after sending: */
		/* or, set logical level for RTS pin equal to 0 after sending: */
		rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;

		/* Set rts delay before send, if needed: */
		rs485conf.delay_rts_before_send = pre;

		/* Set rts delay after send, if needed: */
		rs485conf.delay_rts_after_send = post;

		if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
			perror("(W) ioctl TIOCSRS485");
		}
	}

	/* Flush any character arrived or not transmitted yet... */
	serial_flush_tx(fd);
	serial_flush_rx(fd);

	DRIVER_NOISY("Exit\n");

	return 0;
}


int send_serial_data(int fd, const unsigned char *buffer, int len)
{
	int rval = 0;

	DRIVER_NOISY("Enter Buffer: %p - Len: %d\n", buffer, len);

	if (fd < 0)
	{
		DRIVER_ERROR("Serial File Handler not ready\n");
		return -ECERR_IO;
	}

	if (debuglevelDriver >= DBG_VERBOSE)
	{
		DRIVER_NOISY("EXITING WRITE: ");
		dump_raw_data(buffer, len);
	}

	rval = write(fd, buffer, len);
	DRIVER_NOISY("Exit rval: %d\n", rval);
	return rval;
}

static int serial_wait_data(int fd, long timeout)
{
	fd_set rfds;
	struct timeval tv;
	int rval;
	int retval = -1;

	DRIVER_NOISY("Called with TO: %ld\n", timeout);

	if (fd < 0)
	{
		DRIVER_ERROR("Serial File Handler not ready\n");
		return -ECERR_IO;
	}

	// Timeout must be in milliseconds
	// Wait up to N seconds.
	// Watch file fd to see when it has input.
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	if ( timeout > 0 )
	{
		// Do not touch the requested timeout value
	}
	else
	{
		// If timeout is equal or less than 0, we setup a timeout of
		// 5 seconds to keep the CPU free of charge
		timeout = 5000L;
	}

	tv.tv_sec = timeout / 1000;
	tv.tv_usec = (timeout % 1000) * 1000;
	rval = select(fd+1, &rfds, NULL, NULL, &tv);

	if (rval < 0)
	{
		if (errno == EINTR)
		{
			DRIVER_VERBOSE("Interrupted SystemCall, Timeout\n");
			retval = 0;
		}
		else
		{
			DRIVER_ERROR( "SELECT ERROR\n");
			retval = -1;
		}
	}
	else
	if (rval == 0)
	{
		DRIVER_VERBOSE("SELECT TIMEOUT\n");
		retval = 0;
	}
	else
	{
		DRIVER_VERBOSE("SELECT RECEIVE\n");
		retval = 1;
	}

	DRIVER_NOISY("Exit with %d\n", retval);
	return retval;

}

/*
 * Restituisce < 0 se errore,
 * altrimenti 0 se non ci sono caratteri da leggere, altrimenti
 * se ne ho letti restituisco quanti ne ho letti ed il buffer
 */
int serial_read_string(int fd, unsigned char *buf, long to)
{
	int retval;
	int rval;
	unsigned char *buffer = buf;
	int serialread;
	long timeout;
	int i;

	// Se entro 2.5 secondi non ho nulla, allora posso dire che il
	// modem non risponde!
	DRIVER_NOISY("Enter\n");

	if (fd < 0)
	{
		DRIVER_ERROR("Serial File Handler not ready\n");
		return -ECERR_IO;
	}

	if (buf == NULL)
	{
		DRIVER_ERROR("Empty buffer\n");
		return -ECERR_IO;
	}

	/*
	 * Stiamo in attesa di avere dei dati su seriale...
	 * almeno 2.5 secondi
	 */
	if (to < 0)
		to = 2500;

	rval = serial_wait_data(fd, to);
	if (rval <= 0)
	{
		DRIVER_VERBOSE("Timeout waiting serial response\n");
		rval = 0;
	}
	else
	{
		/*
		 * Vediamo di leggere quello che è nel buffer o per lo meno
		 * fino alla fine stringa (0x0d)
		 */
		int toexit = 0;
		rval = 0;
		serialread = -1;
		/* Abbiamo circa un secondo per leggere tutto quello che arriva
		 * dal modem (almeno tra un carattere e l'altro...) */
		double timing_usec = 5000;
		timeout = (1000000L / timing_usec);
		do
		{
			/*
			 * Leggiamo i caratteri presenti volta per volta...
			 */
			ioctl(fd, FIONREAD, &serialread);
			if (serialread > 0)
			{
				/*
				 * Ci sono dei caratteri nel buffer della seriale,
				 * leggiamoli tutti!
				 */
				retval = read(fd, buffer, serialread);
				if (retval < 0)
				{
					int err = errno;
					DRIVER_ERROR("Error on serial: Errno: %d!\n", err);
					toexit = 1;
				}
				else
				{
					/*
					 * Ne ho letti meno di quelli presenti! Qualche
					 * errore? In ogni caso abbiamo in buffer quelli
					 * che sono riuscito a leggere!
					 */
					if (retval != serialread)
					{
						DRIVER_ERROR("Error on serial: %d!\n", retval);
						toexit = 1;
					}
					else
					{
						/* Reload Timeout for next chars... */
						timeout = (1000000L / timing_usec);
						DRIVER_VERBOSE("Read: %d Characters from Serial\n",
							serialread);
						/*
						 * Aggiorno il conteggio dei caratteri letti
						 * ed il puntatore al buffer per i prossimi
						 * caratteri da leggere!
						 */
						if (debuglevelDriver >= DBG_VERBOSE)
						{
							DRIVER_NOISY("READ (1): ");
							dump_raw_data(buffer, retval);
						}
					}
					rval += retval;
					buffer += retval;
					DRIVER_NOISY("Overall Read: %d Chars Read\n", rval);
					// Controllo il buffer dall'inizio per vedere se
					// tra i caratteri letti c'era la fine del comando
					// \r\n! SEQUENZA: 0x0d 0x0a
					for (i = 0; i < rval; i++)
					{
						DRIVER_NOISY("[%02d] = %c - 0x%02x\n", i,
							buf[i] >= ' ' ? buf[i] : '.',
							buf[i]);
						if (buf[i] == 0x0d && buf[i + 1] == 0x0a)
						{
							// Stampiamo anche quello che, di fatto,
							// costruisce il match.
							DRIVER_NOISY("[%02d] = . - 0x%02x\n", i + 1,
								buf[i + 1]);
							DRIVER_NOISY("<END-OF-COMMAND> Found.\n");
							if (debuglevelDriver >= DBG_VERBOSE && rval > 0)
							{
								DRIVER_NOISY("EXITING READ (2): ");
								dump_raw_data(buf, rval);
							}
							toexit = 1;
							break;
						}
					}
				}
			}
			// Se non dovessimo uscire, allora attendiamo un po' e
			// decrementiamo il timeout...
			if (! toexit)
			{
				usleep(timing_usec);
				timeout--;
			}
			if (timeout < 0)
			{
				DRIVER_NOISY("TIMEOUT REACHED!\n");
				toexit = 1;
			}
			else
			{
				DRIVER_NOISY("TIMEOUT: %ld\n", timeout);
			}
			serialread = -1; /* Invalidiamo per la prossima lettura */
		} while (toexit == 0) ;
		/*
		 * Giunto a questo punto ho letto almeno rval caratteri...
		 */
		*(buffer) = '\0'; // Ci metto almeno il fine stringa...
		DRIVER_NOISY("In the buffer %d --- %s\n", rval, buf);
	}
	DRIVER_NOISY("Exit with: %d\n", rval);
	return rval;
}

int serial_read_raw(int fd, unsigned char *buf, int len)
{
	int retval;
	int rval;
	unsigned char *buffer = buf;
	int serialread = -1;
	int toread = len;
	double timing_usec = 5000; // 5 msec ogni interruzione
	long timeout = 1000000 / timing_usec;

	DRIVER_NOISY("Enter\n");

	if (fd < 0)
	{
		DRIVER_ERROR("Serial File Handler not ready\n");
		return -ECERR_IO;
	}

	if (buf == NULL)
	{
		DRIVER_ERROR("Empty buffer\n");
		return -ECERR_IO;
	}

	/*
	 * Stiamo in attesa di avere dei dati su seriale...
	 * Diciamo almeno 4 secondi...
	 */
	rval = serial_wait_data(fd, 4000);
	if (rval <= 0)
	{
		DRIVER_VERBOSE("Timeout waiting serial response\n");
		rval = 0;
	}
	else
	{
		rval = 0;
		DRIVER_NOISY("READ LOOP START - WAITING %d BYTES\n", len);
		for (;;)
		{
			// Ci sono dei dati da leggere!
			retval = ioctl(fd, FIONREAD, &serialread);
			if (retval < 0)
			{
				if (errno != EINTR && errno != EAGAIN)
				{
					DRIVER_NOISY("IOCTL FIONREAD Error %d -- Retval: %d\n",
						errno, retval);
					// Questo e' un errore! Deve pensarci il chiamante!
					return retval;
				}
				else
				{
					DRIVER_NOISY("-- IOCTL FIONREAD EINTR/EAGAIN --\n");
				}
			}
			else
			{
				DRIVER_NOISY("SOMETHING TO READ (FROM IOCTL): %d SERIALREAD\n", serialread);
				// La ioctl mi ha detto che potrebbero esserci (forse)
				// dei caratteri
				if (serialread > 0)
				{
					// Al massimo leggiamo quelli che ci spettano!
					if (serialread > toread)
					{
						DRIVER_NOISY("LIMIT SERIALREAD TO: %d\n", toread); 
						serialread = toread;
					}

					retval = read(fd, buffer, serialread);
					DRIVER_NOISY("read() RETURNS: %d - SERIALREAD: %d\n",
						retval, serialread);
					if (retval < 0)
					{
						if (errno != EINTR && errno != EAGAIN)
						{
							// Questo e' un errore
							DRIVER_NOISY("READ Error %d -- Retval: %d\n",
								errno, retval);
							return retval;
						}
						else
						{
							DRIVER_NOISY("-- READ EINTR/EAGAIN --\n");
						}
					}
					else
					{
						if (retval == 0)
						{
							// Deleghiamo il controllo al chiamante.
							// Ho detto di leggere 'len' bytes e non 
							// ne ho letto nessuno...
							DRIVER_NOISY("NO CHARACTERS TO READ! ???\n");
							return 0;
						}
						else
						{
							// Aggiorno il totale...
							rval += retval;
							DRIVER_NOISY("READ: %d CHARS\n", rval);
							if (rval == len)
							{
								DRIVER_NOISY("READ DONE %d CHARS\n", len);
								break;
							}

							// Se non ne ho letti tutti, aggiorno che
							// devo leggerne i rimanenti!
							toread -= retval;
							DRIVER_NOISY("TO READ: %d\n", toread);

							// Aggiorno il puntatore al buffer
							buffer += retval;
						}
					}
				}
				// Se sono qui, vuol dire che non ho letto tutti
				// i caratteri che mi occorrono oppure che non mi
				// sono arrivati ancora tutti...
				// Attendo un po' e poi ci riproviamo!

				timeout = (1000000 / timing_usec);
				usleep(timing_usec);
				timeout--;
				if (timeout < 0)
				{
					DRIVER_NOISY("TIMEOUT REACHED!\n");
					break;
				}
			}
			serialread = -1; /* Invalidiamo per la prossima lettura */
			DRIVER_NOISY("\t*** REPLAY LOOP ***\n");
		}
	}

	DRIVER_NOISY("Exit with: %d\n", rval);
//	fprintf(stdout, "READ LEN: %d Exit with: %d\n", len, rval);

	if (debuglevelDriver >= DBG_VERBOSE && rval > 0)
	{
		DRIVER_NOISY("EXITING READ: ");
		dump_raw_data(buf, rval);
	}

	return rval;
}


int serial_send_raw(int fd, const unsigned char *string, int len)
{
	int rval;
	DRIVER_NOISY("Enter with buffer %p LEN: %d\n", string, len);
	rval = send_serial_data(fd, (unsigned char *) string, len);
	DRIVER_NOISY("Exit with: %d\n", rval);
	//fprintf(stdout, "\tWRITE len: %d -- %d\n", len, rval);
	return rval;
}


int serial_send_string(int fd, const unsigned char *string)
{
	int rval = 0;
	DRIVER_NOISY("Enter with buffer string %s\n", string);
	if (fd < 0)
	{
		DRIVER_ERROR("Serial File Handler not ready\n");
		return -ECERR_IO;
	}
	rval = send_serial_data(fd, (unsigned char *) string, strlen((const char *)string));
	DRIVER_NOISY("Exit with: %d\n", rval);
	return rval;
}


void serial_flush_rx(int serfd)
{
	DRIVER_NOISY("Serial Flush INPUT\n");
	if (serfd >= 0)
		tcflush(serfd, TCIFLUSH);
}

void serial_flush_tx(int serfd)
{
	DRIVER_NOISY("Serial Flush OUTPUT\n");
	if (serfd >= 0)
		tcflush(serfd, TCOFLUSH);
}
