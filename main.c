#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define UART_BUFFER_SIZE 4096

int                   m_stty_fd      = 0;
int                   m_baudrate     = 115200;
int                   m_ctrl_c_count = 0;
static struct termios m_oldt;

// restore terminal settings
void exit_cleanup(void)
{
    tcsetattr(0, TCSANOW, &m_oldt);
}

// make terminal read 1 char at a time
void disable_terminal_echo(void)
{
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

int stty_telos(int fd, int baudrate)
{
    struct termios tty;
    speed_t        speed;
    int            i;

    switch (baudrate)
    {
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
#ifdef B230400
    case 230400:
        speed = B230400;
        break;
#endif
#ifdef B460800
    case 460800:
        speed = B460800;
        break;
#endif
#ifdef B500000
    case 500000:
        speed = B500000;
        break;
#endif
#ifdef B576000
    case 576000:
        speed = B576000;
        break;
#endif
#ifdef B921600
    case 921600:
        speed = B921600;
        break;
#endif
#ifdef B1000000
    case 1000000:
        speed = B1000000;
        break;
#endif
#ifdef B1152000
    case 1152000:
        speed = B1152000;
        break;
#endif
#ifdef B1500000
    case 1500000:
        speed = B1500000;
        break;
#endif
#ifdef B2000000
    case 2000000:
        speed = B2000000;
        break;
#endif
#ifdef B2500000
    case 2500000:
        speed = B2500000;
        break;
#endif
#ifdef B3000000
    case 3000000:
        speed = B3000000;
        break;
#endif
#ifdef B3500000
    case 3500000:
        speed = B3500000;
        break;
#endif
#ifdef B4000000
    case 4000000:
        speed = B4000000;
        break;
#endif
    default:
        fprintf(stderr, "unknown baudrate %d", baudrate);
        exit(1);
    }

    if (tcflush(fd, TCIOFLUSH) == -1)
    {
        fprintf(stderr, "tcflush() : %s\n", strerror(errno));
        exit(1);
    }

    if (tcgetattr(fd, &tty) == -1)
    {
        fprintf(stderr, "tcgetattr() : %s\n", strerror(errno));
        exit(1);
    }

    cfmakeraw(&tty);

    // nonblocking read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~HUPCL;
    tty.c_cflag &= ~CLOCAL;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSAFLUSH, &tty) == -1)
    {
        fprintf(stderr, "tcsetattr() : %s\n", strerror(errno));
        exit(1);
    }

    // nonblocking read and write
    if (fcntl(fd, F_SETFL, O_NONBLOCK) == -1)
    {
        fprintf(stderr, "fcntl() : %s\n", strerror(errno));
        exit(1);
    }

    tty.c_cflag |= CLOCAL;
    if (tcsetattr(fd, TCSAFLUSH, &tty) == -1)
    {
        fprintf(stderr, "tcsetattr() : %s\n", strerror(errno));
        exit(1);
    }

    i = TIOCM_DTR;
    if (ioctl(fd, TIOCMBIS, &i) == -1)
    {
        fprintf(stderr, "ioctl() : %s\n", strerror(errno));
        exit(1);
    }

    usleep(10 * 1000); // wait for hardware 10ms

    // flush input and output buffers
    if (tcflush(fd, TCIOFLUSH) == -1)
    {
        fprintf(stderr, "tcflush() : %s\n", strerror(errno));
        exit(1);
    }

    return 0;
}

void convertBackspace(char *aBuffer, size_t aSize)
{
#ifdef __APPLE__
    // Linux backspace code: {0x7f, 0x1b, 0x5b, 0x4b}
    // MAC   backspace code: {0x08, 0x1b, 0x5b, 0x4b}

    // convert to MAC backspace code
    {
        int i;
        for (i = 0; i < aSize; i++)
        {
            if (aBuffer[i] == 0x7f)
                aBuffer[i] = 0x08;
        }
    }
#else
    (void)aBuffer;
    (void)aSize;
#endif
}

void serial_to_stdout(int fd)
{
    char buf[UART_BUFFER_SIZE] = {0};
    int  size;

    size = read(fd, buf, sizeof(buf));
    if (size < 0)
    {
        return;
    }

    convertBackspace(buf, size);

    write(STDOUT_FILENO, buf, size);
}

void stdin_to_serial(int fd)
{
    char buf[UART_BUFFER_SIZE] = {0};
    int  size;

    size = read(STDIN_FILENO, buf, sizeof(buf));
    if (size < 0)
    {
        return;
    }

    write(fd, buf, size);
}

void print_help()
{
    fprintf(stdout, "usage: myuart stty_dev  [baudrate]\n\n");
    fprintf(stdout, "  Options:\n");
    fprintf(stdout, "     stty_dev    Serial device\n");
    fprintf(stdout, "     baudrate    9600,19200,38400,57600,115200 (default),230400, 460800,\n"
                    "                 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000,\n"
                    "                 2500000, 3000000, 3500000, 4000000\n");
    fprintf(stdout, "\n");
    fprintf(stdout, "  Example: myuart /dev/ttyUSB0 115200\n");
    fprintf(stdout, "  Press Ctrl + C twice without delay to exit!\n");
}

void sigcleanup(int signo)
{
    (void)signo;

    m_ctrl_c_count++;

    if (m_ctrl_c_count < 2)
    {
        if (m_stty_fd >= 0)
        {
            uint8_t buf[1] = {0x03}; // Send Ctrl + C to device
            write(m_stty_fd, buf, sizeof(buf));
        }
    }
    else
    {
        if (m_stty_fd != 0)
        {
            close(m_stty_fd);
        }

        exit(0); // exit(0) will call exit_cleanup()
    }
}

int main(int argc, char **argv)
{
    int    max_fd;
    int    ret;
    fd_set rset;
    fd_set wset;

    if (argc <= 2)
    {
        if ((argc == 1) || (strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0))
        {
            print_help();
            return 0;
        }
    }

    if (argc > 2)
    {
        m_baudrate = atoi(argv[2]);
    }

    m_stty_fd = open(argv[1], O_RDWR | O_NONBLOCK);
    if (m_stty_fd < 0)
    {
        fprintf(stderr, "open() : %s\n", strerror(errno));
        exit(1);
    }

    if (stty_telos(m_stty_fd, m_baudrate) < 0)
    {
        fprintf(stderr, "stty_telos() : %s\n", strerror(errno));
        close(m_stty_fd);
        exit(1);
    }

    disable_terminal_echo();
    signal(SIGINT, sigcleanup);

    while (1)
    {
        max_fd = 0;

        FD_ZERO(&rset);
        FD_ZERO(&wset);

        FD_SET(STDIN_FILENO, &rset);
        FD_SET(m_stty_fd, &rset);

        if (m_stty_fd > max_fd)
        {
            max_fd = m_stty_fd;
        }

        ret = select(max_fd + 1, &rset, &wset, NULL, NULL);
        if ((ret == -1) && (errno != EINTR))
        {
            fprintf(stderr, "select() : %s\n", strerror(errno));
            break;
        }
        else if (ret > 0)
        {
            if (FD_ISSET(m_stty_fd, &rset))
            {
                serial_to_stdout(m_stty_fd);
            }

            if (FD_ISSET(STDIN_FILENO, &rset))
            {
                m_ctrl_c_count = 0;
                stdin_to_serial(m_stty_fd);
            }
        }
    }

    close(m_stty_fd);

    return 0;
}
