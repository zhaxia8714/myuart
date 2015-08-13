#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdarg.h>

int m_stty_fd = 0;
int m_baudrate = 115200;
static struct termios m_oldt;

// restore terminal settings
void exit_cleanup(void) {
  tcsetattr(0, TCSANOW, &m_oldt); 
}

// make terminal read 1 char at a time
void disable_terminal_echo(void) {
  struct termios newt;

  // save terminal settings
  tcgetattr(0, &m_oldt); 
  // init new settings
  newt = m_oldt;  
  // change settings
  newt.c_lflag &= ~(ICANON | ECHO);
  // apply settings
  tcsetattr(0, TCSANOW, &newt);

  // make sure settings will be restored when program ends
  atexit(exit_cleanup); // will be called when the program exit
}

int stty_telos(int fd, int baudrate) {
  struct termios tty;
  speed_t speed;
  int i;

  switch(baudrate) {
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 38400:
    speed = B38400;
    break;
  case 57600:
    speed = B57600;
    break;
  case 115200:
    speed = B115200;
    break;
  case 230400:
    speed = B230400;
    break;
  default:
    fprintf(stderr, "unknown baudrate %d", baudrate);
    return -1;
  }

  if(tcflush(fd, TCIOFLUSH) == -1)
    fprintf(stderr, "Error: tcflush()\n");

  if(tcgetattr(fd, &tty) == -1)
    fprintf(stderr, "Error: tcgetattr()\n");

  cfmakeraw(&tty);

  // nonblocking read
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~HUPCL;
  tty.c_cflag &= ~CLOCAL;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if(tcsetattr(fd, TCSAFLUSH, &tty) == -1)
    fprintf(stderr, "Error: tcsetattr()\n");

  // nonblocking read and write
  if(fcntl(fd, F_SETFL, O_NONBLOCK) == -1)
    fprintf(stderr, "Error:fcntl()\n");

  tty.c_cflag |= CLOCAL;
  if(tcsetattr(fd, TCSAFLUSH, &tty) == -1)
    fprintf(stderr, "Error: tcsetattr()\n");

  i = TIOCM_DTR;
  if(ioctl(fd, TIOCMBIS, &i) == -1)
    fprintf(stderr, "Error: ioctl()\n");

  usleep(10*1000);  // wait for hardware 10ms

  // flush input and output buffers
  if(tcflush(fd, TCIOFLUSH) == -1)
    fprintf(stderr, "Error: tcflush()\n");

  return 0;
}

void serial_to_stdout(int fd) {
  char buf[2000] = {0};
  int size, pos;

  size = read(fd, buf, sizeof(buf));
  if (size < 0)
    return;

#ifdef __APPLE__
  // Linux backspace code: {0x7f, 0x1b, 0x5b, 0x4b}
  // MAC   backspace code: {0x08, 0x1b, 0x5b, 0x4b}

  // convert to MAC backspace code
  {
    int i;
    for (i = 0; i < size; i++) {
      if (buf[i] == 0x7f)
        buf[i] = 0x08;
    }
  }
#endif

  write(STDOUT_FILENO, buf, size);
}

void stdin_to_serial(int fd) {
  char ch = getchar();
  write(fd, &ch, 1);
}

void print_help() {
  fprintf(stdout,"usage: myuart stty_dev  [baudrate]\n\n");
  fprintf(stdout,"  example: myuart /dev/ttyUSB0 115200\n");
  fprintf(stdout,"  Options are:\n");
  fprintf(stdout,"     stty_dev    Serial device\n");
  fprintf(stdout,"     baudrate    9600,19200,38400,57600,115200 (default),230400\n");
  fprintf(stdout,"\n");
}

void sigcleanup(int signo) {
  if (m_stty_fd != 0)
    close(m_stty_fd);

  exit(0); // exit(0) will call exit_cleanup()
}

int  main (int argc, char **argv) {
  int max_fd, ret, c;
  fd_set rset, wset;

  if (argc < 2) {
    print_help();
    return 0;
  }

  if (argc > 2) {
    m_baudrate = atoi(argv[2]);
  }

  m_stty_fd = open(argv[1], O_RDWR | O_NONBLOCK);
  if (m_stty_fd < 0) {
    fprintf(stderr, "Error: open %s fail\n", argv[1]);
    return -1;
  }

  if (stty_telos(m_stty_fd, m_baudrate) < 0) {
    fprintf(stderr, "Error: configure %s fail\n", argv[1]);
    close(m_stty_fd);
    return -1;
  }

  disable_terminal_echo();
  signal(SIGINT, sigcleanup);

  while(1) {
    max_fd = 0;
    FD_ZERO(&rset);
    FD_ZERO(&wset);
  
    FD_SET(STDIN_FILENO, &rset);
    FD_SET(m_stty_fd, &rset);
    if(m_stty_fd > max_fd)
      max_fd = m_stty_fd;

    ret = select(max_fd + 1, &rset, &wset, NULL, NULL);
    if((ret == -1) && (errno != EINTR)) {
      fprintf(stderr, "Error:select\n");
      break;
    } else if(ret > 0) {
      if(FD_ISSET(m_stty_fd, &rset)) {
        serial_to_stdout(m_stty_fd);
      }

      if(FD_ISSET(STDIN_FILENO, &rset)) {
        stdin_to_serial(m_stty_fd); 
      }
    }
  }

  close(m_stty_fd);
  return 0;
}


