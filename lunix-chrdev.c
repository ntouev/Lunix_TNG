/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Gouliamou Maria-Ethel,
 * Ntouros Evangelos
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
    int ret = 0; //by default we are assuming that the state doesnt need refreshing
	struct lunix_sensor_struct *sensor;

	WARN_ON ( !(sensor = state->sensor));  //???
    if (sensor->msr_data[state->type]->last_update != state->buf_timestamp)
    {
        debug("State needs refreshing!\n");
        ret = 1; //now state needs refreshing
    }

    return ret;
}

/*
 * Updates the cached state of a character device based on sensor data. Must be
 * called with the character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
    unsigned long flags;        //spinlock's flag is an unsigned long int
    uint16_t value;             //lookup tables require uint16_t
    long int result;            //lookup tables give long int
    uint32_t current_timestamp; //defined in lunix_msr_data_struct
    int ret;

	debug("entering\n");
    sensor = state->sensor;
    ret = -EAGAIN; //there is no data available right now, try again later

    /*
	 * Grab the raw data quickly, hold the spinlock for as little as possible.
	 */

     // save the state, if locked already it is saved in flags
     spin_lock_irqsave(&sensor->lock, flags);
     value = sensor->msr_data[state->type]->values[0];
     current_timestamp = sensor->msr_data[state->type]->last_update;
     // return to the formally state specified in flags
     spin_unlock_irqrestore(&sensor->lock, flags);
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
     *
 	 * Now we can take our time to format them, holding only the private state
 	 * semaphore. This is implemented in open syscall.
 	 */
    if(lunix_chrdev_state_needs_refresh(state))
    {
        if (!state->raw_data)
        {   //coocked data
            if (state->type == BATT)
            {
                result = lookup_voltage[value];
            }
            else if (state->type == TEMP)
            {
                result = lookup_temperature[value];
            }
            else if (state->type == LIGHT)
            {
                result = lookup_light[value];
            }
            else
            {
                debug("Internal Error: Type doesnt match one the three available \
                                                            (BATT, TEMP, LIGHT)");
                ret = -EMEDIUMTYPE;    //wrong medium type
                goto out;
            }

            /*save formatted data in chrdev state buffer*/
            ret = 0;
            state->buf_timestamp = current_timestamp;
            state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, \
                                    "%ld.%03ld\n", result/1000, result%1000);
        }
        else{   //raw data
            debug("skipped lookup table conversion, returning raw bytes...\n");
            ret = 0;
            state->buf_timestamp = current_timestamp;
            state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, \
                                    "%x\n", value); //prints raw_data as hex
        }
    }
    else
    {
        ret = -EAGAIN;
    }

out:
	debug("leaving\n");
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
    unsigned int minor, sensor, type;
    int ret;
    struct lunix_chrdev_state_struct *state;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
    minor = MINOR(inode->i_rdev);   //LDD3 sel 55
    sensor = minor/8;               // 0-15
    type = minor%8;                 // 0-2
    debug("Done assosiating file with sensor %d of type %d\n", sensor, type);

	/* Allocate a new Lunix character device private state structure */

    //GFP_KERNEL	This is a normal allocation and might block.
    //This is the flag to use in process context code when it is safe to sleep
    state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
    state->type = type;
    state->sensor = &lunix_sensors[sensor];

    /*arxikopoihsh buffer*/
    state->buf_lim = 0;
    state->buf_timestamp = 0;
    filp->private_data = state; //points to the current state of the device
    state->raw_data = 0;        //by default, in coocked data mode

    sema_init(&state->lock,1);  //initialize semaphore with 1
    ret = 0;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
    debug("Releasing allocated memory for private file data\n");
    kfree(filp->private_data);
    debug("Done releasing private file data, exiting now..");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct lunix_chrdev_state_struct *state;
    debug("entering\n");

    //if cmd's type is not LUNIX_IOC_MAGIC (it is 60, the major number) return error
    if (_IOC_TYPE(cmd) != LUNIX_IOC_MAGIC) return -ENOTTY;
    //accept only 1 cmd
    if (_IOC_NR(cmd) > LUNIX_IOC_MAXNR) return -ENOTTY;

    state = filp->private_data;

    switch (cmd)
    {
        case LUNIX_IOC_DATA_TYPE_CONVERT:
            if (down_interruptible(&state->lock))   //is it needed?
                return -ERESTARTSYS;
            //if I have raw data I turn them into coocked and vice versa
            if (state->raw_data)
                state->raw_data = 0; //turned into coocked
            else
                state->raw_data = 1; //turned into raw
            up(&state->lock);       //if statement needed?

    }

    debug("successfully changed data type transfer, now state->raw_data=%d\n", state->raw_data);
	return 0;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret, remaining_bytes;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);
    debug("entering!\n");

	/* Lock */
    if (down_interruptible(&state->lock))
        return -ERESTARTSYS;

    /*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
            up(&state->lock);
            //if the file was opened with nonblocking operation return
            if (filp->f_flags & O_NONBLOCK)
                return -EAGAIN;
            if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
                //wait_event_interruptible returns nonzero when interrupted by signal
                return -ERESTARTSYS;
            //now grab again the lock because you woke up, and conitnue the read
            if (down_interruptible(&state->lock))
                return -ERESTARTSYS;
		}
	}
    debug("Ok, now I have fresh values\n");

    /* Determine the number of cached bytes to copy to userspace */
    remaining_bytes = state->buf_lim - *f_pos;

    if (cnt >= remaining_bytes)
    {
        cnt = remaining_bytes;
    }

    /*
     * copy_to_user(to, from, count)
     * returns the number of bytes that couldnt copy
     */
    if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt))
    {
        ret = -EFAULT;
        goto out;
    }

    //fix the position
    *f_pos = *f_pos + cnt;
    ret = cnt;

	/* Auto-rewind on EOF mode */
	if (*f_pos == state->buf_lim)
        *f_pos = 0;

out:
	/* Unlock */
    up(&state->lock);
	return ret;
}


static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    //Not implemented yet
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
{
    .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

//this function is called from lunix-module.c
int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3; //16*8 = 128???

	debug("initializing character device\n");

    //I 've already allocated -above- the struct (lunix_chrdev_cdev)
    //and now I initialize it. I give it the file operations
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

    /* register_chrdev_region */
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0); //"make" the first device

    ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix:TNG");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
    debug("device registered successfully\n");

	/* cdev_add */
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
