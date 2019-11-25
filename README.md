# Lunix-TNG-Linux-Driver-OsLab-NTUA
*Contains the development of a driver for Lunix:TNG, a light-temperature-voltage sensor, in Linux Kernel. This implementation was made for OsLab lecture in NTUA in 2019-2020.*

### Usage

* Download or clone the repository
* $ make
* With root permission:
    * $ insmod ./lunix.ko
    * ./lunix_dev_nodes.sh
    * ./lunix-attach ttyS0
* In another shell as a user:
    * cat /dev/lunixY-XXXX, where XXXX={temp,light,batt} AND Y=[0,15]  
      *(to see the XXXX measurement of the Y sensor)*
