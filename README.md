# Lunix-TNG-Linux-Driver-OsLab-NTUA
*Contains the development of a driver for Lunix:TNG, a light-temperature-voltage sensor, in Linux Kernel. This implementation was made for OsLab lecture in NTUA in 2019-2020.*

## Usage
* Download or clone the repository
* **$ make**
* With root permission:  
    * **\# ./lunixInit.sh**
* In another shell as a user:
    * **$ ./testLunix /dev/lunixY-XXXX Z**, *where XXXX={temp,light,batt}, Y=[0,15] AND Z=0/1 (coocked/raw data)*  
   *(to see the XXXX measurement of the Y sensor)*   
       *or simply*,  
    * **$ cat /dev/lunixY-XXXX**, where XXXX={temp,light,batt} AND Y=[0,15]

#### Usage without the script
* Download or clone the repository
* **$ make**
* With root permission:
    * **\# insmod ./lunix.ko**
    * **\# ./lunix_dev_nodes.sh**
    * **\# ./lunix-attach ttyS0**
* In another shell as a user:
    * **$ ./testLunix /dev/lunixY-XXXX Z**, *where XXXX={temp,light,batt}, Y=[0,15] AND Z=0/1 (coocked/raw data)*  
     *(to see the XXXX measurement of the Y sensor)*  
     *or simply*,  
    * **$cat /dev/lunixY-XXXX**, where XXXX={temp,light,batt} AND Y=[0,15]
