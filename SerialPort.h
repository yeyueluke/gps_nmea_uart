/*
Copyright [year] <Copyright Owner>
*/
#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions      */
#include <errno.h>   /* ERROR Number Definitions           */

#include <iostream>
#include <string>
#include <cstring>
#include <memory>

// using namespace std;
#if 0

/* Read "n" bytes from a descriptor. */
ssize_t readn(int fd_, void *vptr, size_t n) {
    size_t nleft;
    ssize_t nread;
    char *ptr;

    ptr = vptr;
    nleft = n;
    while (nleft > 0) {
        if ((nread = read(fd_, ptr, nleft)) < 0) {
            if (errno == EINTR)
                nread = 0;      /* and call read() again */
            else
                return(-1);
        } else if (nread == 0) {
            break;              /* EOF */
        }

        nleft -= nread;
        ptr   += nread;
    }
    return(n - nleft);      /* return >= 0 */
}

/* end readn */
ssize_t Readn(int fd_, void *ptr, size_t nbytes) {
    ssize_t     n;

    if ((n = readn(fd_, ptr, nbytes)) < 0)
        printf("readn error");
    return(n);
}

ssize_t writen(int fd_, const void *buf, size_t num) {
    ssize_t res;
    size_t n;
    const char *ptr;

    n = num;
    ptr = buf;
    while (n > 0) {
    /* 开始写*/
        if ((res = write(fd_, ptr, n)) <= 0) {
        if (errno == EINTR)
            res = 0;
        else
            return (-1);
        }
        ptr += res;  /* 从剩下的地方继续写*/
        n -= res;
    }

    return (num);
}

static int  read_cnt;
static char *read_ptr;
static char read_buf[MAXLINE];

static ssize_t my_read(int fd_, char *ptr) {
    if (read_cnt <= 0) {
again:
        if ((read_cnt = read(fd_, read_buf, sizeof(read_buf))) < 0) {
            if (errno == EINTR)
                goto again;

            return(-1);
        } else if (read_cnt == 0) {
            return(0);
        }
        read_ptr = read_buf;
    }

    read_cnt--;
    *ptr = *read_ptr++;
    return(1);
}

ssize_t readline(int fd_, void *vptr, size_t maxlen) {
    ssize_t n, rc;
    char c, *ptr;

    ptr = vptr;
    for (n = 1; n < maxlen; n++) {
        if ((rc = my_read(fd_, &c)) == 1) {
            *ptr++ = c;
            if (c == '\n') {
                break;  /* newline is stored, like fgets() */
            }
        } else if (rc == 0) {
            *ptr = 0;
            return(n - 1);  /* EOF, n - 1 bytes were read */
        } else {
            return(-1);     /* error, errno set by read() */
        }
    }

    *ptr = 0;   /* null terminate like fgets() */
    return(n);
}

ssize_t readlinebuf(void **vptrptr) {
    if (read_cnt)
        *vptrptr = read_ptr;
    return(read_cnt);
}
/* end readline */

ssize_t Readline(int fd_, void *ptr, size_t maxlen) {
    ssize_t n;

    if ((n = readline(fd_, ptr, maxlen)) < 0)
        printf("readline error\n");
    return(n);
}
#endif
const int BUFFER_LEN = 1024;
class SerialPort {
 private:
    int fd_;  /*File Descriptor*/
    char buf_[BUFFER_LEN] = {0};

 public:
    SerialPort(const char* portName , int baud);
    ~SerialPort();

    char readChar();
    char* readString(int maxBuffer);
    char* readString();
    int writeChar(char c);
    int writeString(char* str);
    int writeString(std::string str);
    void clearBuf() { memset(buf_, 0, BUFFER_LEN);}
    // todo
    // std::string readLines(int n);
};

SerialPort::SerialPort(const char* portName, int serial_speed) {
    fd_ = open(portName, O_RDWR | O_NOCTTY);
    if (fd_ == -1)
        printf(" Error! in Opening port %s \n", portName);
    else
        printf("  %s Opened Successfully \n", portName);

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0)
        printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    // setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    // set baud rate to the parameter comm_speed
    if (serial_speed == 4800)
        cfsetispeed(&tty, B4800);
    else if (serial_speed == 9600)
        cfsetispeed(&tty, B9600);
    else if (serial_speed == 19200)
        cfsetispeed(&tty, B19200);
    else if (serial_speed == 38400)
        cfsetispeed(&tty, B38400);
    else if (serial_speed == 57600)
        cfsetispeed(&tty, B57600);
    else if (serial_speed == 115200)
        cfsetispeed(&tty, B115200);
    else if (serial_speed == 460800)
        cfsetispeed(&tty, B460800);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        printf("Error %d from tcsetattr: %s\n", errno, strerror(errno));
}

SerialPort::~SerialPort() {
    close(fd_);
}

/* return char */
char SerialPort::readChar() {
    char c;
    int byte_read = read(fd_, &c, 1);
    if (byte_read > 0) {
        return c;
    } else {
        printf("read failded\n");
        return -1;
    }
}

/* return char* string with maxBufferSize = 200  */
char* SerialPort::readString() {
    return readString(200);
}

/* return char* string with maxBufferSize  */
char* SerialPort::readString(int maxBufferSize) {
    int index = 0;
    char c = ' ';

    while (c != 0x0a && index < maxBufferSize - 2) {  // may be 0x0a(\n), 0x0d(\r)
        c = readChar();
        buf_[index] = c;
        index++;
    }

    return buf_;
}

int SerialPort::writeChar(char c) {
    char cc[1];
    cc[0] = c;
    int bytes_written = 0;
    bytes_written = write(fd_, cc, sizeof(cc));
    return bytes_written;
}

int SerialPort::writeString(char* str) {
    int bytes_written  = 0;
    bytes_written = write(fd_, str, sizeof(str));
    bytes_written += write(fd_, "\n", sizeof("\n"));
    return bytes_written;
}

int SerialPort::writeString(std::string str) {
    char *cstr = new char[str.size() + 1];
    strncpy(cstr, str.c_str(), str.size());
    int bytes_written  = 0;
    bytes_written = write(fd_, cstr, sizeof(cstr));
    bytes_written += write(fd_, "\n", sizeof("\n"));
    delete []cstr;
    return bytes_written;
}
#endif  // SERIALPORT_H_
