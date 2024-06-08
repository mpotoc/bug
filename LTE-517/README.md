# configuration on raspberry pi zero W for Huawei MS2372-517 LTE stick

1. insert LTE stick for the first time if not yet inserted into the black box so that it can initialize itself before doing anything
2. you have to login over WIFI to raspberry pi
3. install minicom package -> sudo apt install minicom -y
4. then change privileges on serial file -> chmod +x serial 
5. run script -> ./serial
6. this should start the communication with the LTE stick and whereversim operator
7. you should now see you have internet in whereversim dashboard
8. open rc.local file -> sudo nano /etc/rc.local
9. copy this 
    -> (echo -ne "AT^NDISDUP=1,1,\"wm\"\r"; sleep 5; echo -ne "\x01x\r") | minicom -D /dev/ttyUSB2
    -> quotes around "wm" must be escaped (\"), it is written correctly in file if you open it up as nano README.md on raspbery
    -> to rc.local before exit 0 command
10. reboot the pi and observe if LTE stick is running correctly