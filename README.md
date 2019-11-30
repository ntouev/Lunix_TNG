***Trying to add mmap method***  
*TODO:*
* *fix mmap operations in lunix-chrdev.c*
* *fix testMmap.c for testing*

# Lunix-TNG-Linux-Driver-OsLab-NTUA
*Contains the development of a driver for Lunix:TNG, a light-temperature-voltage sensor, in Linux Kernel. This implementation was made for OsLab lecture in NTUA in 2019-2020.*

## Usage
* Download or clone the repository.
* **$ make**
* With root permission:  
    * **\# ./lunixInit.sh**  
     *This script unloads the previous version of the module (if that exists), loads the new one, make the nodes in the /dev directory for the device and finally attaches the lunix line discipline.*  
* *(To test basic methods)* In another shell as a user:
    * **$ cat /dev/lunixY-XXXX**, where XXXX={temp,light,batt} AND Y=[0,15]
    * Open multiple shells and run the above command.
    * Additionally check the case where parent and child process read from the same fd.
* *(To test ioctl method)* In another shell as a user:
    * **$ ./testIoctl /dev/lunixY-XXXX Z**, *where XXXX={temp,light,batt}, Y=[0,15] AND Z=0/1 (coocked/raw data)*  
   *(to see the XXXX measurement of the Y sensor)*   
