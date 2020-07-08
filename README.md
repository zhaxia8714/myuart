Myuart is a simple serial port communications program.

### Build
```
git clone https://github.com/zhaxia8714/myuart.git
make
sudo mv ./myuart /usr/bin
```

### Usage
```
myuart uart_dev [baudrate]    
```

Options:
- uart_dev: Serial device
- baudrate: 9600,19200,38400,57600,115200 (default),230400
     
### Example: 
```
myuart /dev/ttyUSB0 115200
```
