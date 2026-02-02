delete(serialportfind);
serialportlist("available")
serialObj = serialport("COM4", 115200);
configureTerminator(serialObj, "CR/LF");
flush(serialObj);

%Deberia introducir un mensaje para indicar que se escriba un setpoint

