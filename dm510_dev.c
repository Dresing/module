/* Prototype module for third mandatory DM510 assignment */
#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
/* #include <asm/system.h> */
#include <asm/switch_to.h>
#ifndef DEFAULT_BUFFER
  #define DEFAULT_BUFFER 4000
#endif
#define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

struct buffer {

};

struct device {
    int    index;                      /* device index to identify read/write locations */
    wait_queue_head_t inq, outq;       /* read and write queues */
    char *start, *end;                /* begin of buf, end of buf */
    int buffersize;                    /* used in pointer arithmetic */
    char *rp, *wp;                     /* where to read, where to write */
    int nreaders, nwriters;
    struct fasync_struct *async_queue; /* asynchronous readers */
    struct semaphore sem;              /* mutual exclusion semaphore */
    struct cdev cdev;                  /* Char device structure */
};

static struct device *devices;

/* Prototypes - this would normally go in a .h file */
static int dm510_open( struct inode*, struct file* );
static int isError(int);
static int dm510_release( struct inode*, struct file* );
static ssize_t dm510_read( struct file*, char*, size_t, loff_t* );
static ssize_t dm510_write( struct file*, const char*, size_t, loff_t* );
long dm510_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int dm510_init_device(struct device*, int index);
int dm510_init_buffer(struct buffer*);

#define DEVICE_NAME "dm510_dev" /* Dev name as it appears in /proc/devices */
#define MAJOR_NUMBER 254
#define MIN_MINOR_NUMBER 0
#define MAX_MINOR_NUMBER 1

#define DEVICE_COUNT 2
/* end of what really should have been in a .h file */

/* file operations struct */
static struct file_operations dm510_fops = {
	.owner   = THIS_MODULE,
	.read    = dm510_read,
	.write   = dm510_write,
	.open    = dm510_open,
	.release = dm510_release,
  .unlocked_ioctl   = dm510_ioctl
};

char* zeroBuffer;


dev_t deviceNums = MKDEV(MAJOR_NUMBER, MIN_MINOR_NUMBER);


/* called when module is loaded */
int dm510_init_module( void ) {

  //Resgister a region for the driver and devices.
	int status = register_chrdev_region(deviceNums, DEVICE_COUNT , "tester");

  //Negative values indicate an error. No releasing is necesarry.
	if (isError(status)) {
		printk(KERN_NOTICE "Unable to get region, error %d\n", status);
		return -1;
	}

  //Allocate memory for the default number of devices
	devices = kmalloc(DEVICE_COUNT * sizeof(struct device), GFP_KERNEL);

  //Upon error allocating memory, unregister the region an report the error.
	if (devices == NULL) {
		unregister_chrdev_region(deviceNums, DEVICE_COUNT);
		return -1;
	}

  //Alocate buffers
  zeroBuffer = kmalloc(DEFAULT_BUFFER, GFP_KERNEL);

  //The buffers were not allocated, release lock and give a no memory error.
  if (!zeroBuffer) {
    unregister_chrdev_region(deviceNums, DEVICE_COUNT);
    printk(KERN_INFO "DM510: Register error. \n");
    return -ENOMEM;
  }

  //Iterating variable necesary as of c90 standard
	int i;

	//Ensure everything is emptied at the memory location to avoid junk errors.
	memset(devices, 0, DEVICE_COUNT * sizeof(struct device));

  //Initialize all the devices.
	for (i = 0; i < DEVICE_COUNT; i++) {
		dm510_init_device(&devices[i], i);
	}


  //Debugging message
	printk(KERN_INFO "DM510: Hellows from your devicos!\n");

  //Successful execution
	return 0;
}

/* Called when module is unloaded */
void dm510_cleanup_module( void ) {

  //Memory location of devices is empty, no work required.
	if (!devices){
      return;
  }

  //Iterating variable necesary as of c90 standard
	int i;

  //Starting cleaning up allocations.
	for(i = 0; i < DEVICE_COUNT; i++){

    //Clean the cdev part of the device struct
		cdev_del(&devices[i].cdev);

    //Free the memory allocat for the buffers.
		kfree(devices[i].start);
	}

  //Free up the memory address for the devices/buffers and unregister the driver regions.
	kfree(devices);
  kfree(zeroBuffer);
  unregister_chrdev_region(deviceNums, DEVICE_COUNT);

  //Ensure that the device pointer is indeed null.
	devices = NULL;

  //Debug message
	printk(KERN_INFO "DM510: Module unloaded.\n");


}


/* Called when a process tries to open the device file */
static int dm510_open( struct inode *inode, struct file *filp ) {

  //Instantiate device struct that will hold a pointer to the current device-struct
  struct device *dev;

  //Get the container where the cdev is contained (device struct)
	dev = container_of(inode->i_cdev, struct device, cdev);

  //Save a short-cut to the struct in the file-pointers private data.
	filp->private_data = dev;

  //Aquire the lock and stay interuptable. If failed, opening allowance.
	if (down_interruptible(&dev->sem)){
    return -ERESTARTSYS;
  }

  //Set pointer to the buffer in kernel space
  dev->start = zeroBuffer;

  //Set buffersize in device struct.
  dev->buffersize = DEFAULT_BUFFER;

  //End of the buffer is the start address plus the size.
  dev->end = dev->start + dev->buffersize;

  //Read and write pointers should start at the beginning of file (device)
  dev->rp = dev->wp = dev->start;

  //Increment number of readers/writers accordingly to mode and release the lock
	if (filp->f_mode & FMODE_READ)
		dev->nreaders++;
	if (filp->f_mode & FMODE_WRITE)
		dev->nwriters++;
	up(&dev->sem);

  //Debug message
  printk(KERN_INFO "DM510: File access granted.\n");

  //Disallow llseek
	return nonseekable_open(inode, filp);

}


/* Called when a process closes the device file. */
static int dm510_release( struct inode *inode, struct file *filp ) {

  //Get the device structure in the file-pointer's private data.
  struct device *dev = filp->private_data;

	//Require the lock and decrement the counter of reader/write depeding on mode.
	down(&dev->sem);
	if (filp->f_mode & FMODE_READ){
    dev->nreaders--;
  }
	if (filp->f_mode & FMODE_WRITE){
    dev->nwriters--;
  }

  //Release the lock
	up(&dev->sem);

  //Debug message
  printk(KERN_INFO "DM510: File released.\n");

  //Successful execution
	return 0;
}


/* Called when a process, which already opened the dev file, attempts to read from it. */
static ssize_t dm510_read( struct file *filp,
    char *buf,      /* The buffer to fill with data     */
    size_t count,   /* The max number of bytes to read  */
    loff_t *f_pos )  /* The offset in the file           */
{

	/* read code belongs here */

	return 0; //return number of bytes read
}


/* Called when a process writes to dev file */
static ssize_t dm510_write( struct file *filp,
    const char *buf,/* The buffer to get data from      */
    size_t count,   /* The max number of bytes to write */
    loff_t *f_pos )  /* The offset in the file           */
{

	/* write code belongs here */

  struct device *dev = filp->private_data;


  //printk(KERN_INFO "DM510: I wrote stuff.\n");
	return 5; //return number of bytes written
}

/* called by system call icotl */
long dm510_ioctl(
    struct file *filp,
    unsigned int cmd,   /* command passed from the user */
    unsigned long arg ) /* argument of the command */
{
	/* ioctl code belongs here */
	printk(KERN_INFO "DM510: ioctl called.\n");

	return 0; //has to be changed
}

static int isError(status){
		return (status < 0);
}

int dm510_init_device(struct device *device, int index){

	//Init queues
	init_waitqueue_head(&(device->inq));
	init_waitqueue_head(&(device->outq));

	//Init lock
	init_MUTEX(&device->sem);

	//Retreive minor number (device number) and set error as positive value.
	int error, devno = deviceNums + index;

	//Init cdev and set owner, device and index number (0-DEVICE_COUNT).
	cdev_init(&device->cdev, &dm510_fops);

	device->cdev.owner = THIS_MODULE;

  device->index = (index % DEVICE_COUNT);

	//Add devno to only one device
	error = cdev_add (&device->cdev, devno, 1);

	//If error has not been set to zero at initialization (success)
	if (error){
		printk(KERN_NOTICE "Error %d adding device %d", error, index);
	}

  printk(KERN_NOTICE "DevNo: %d", devno);


	return 0;
}

int dm510_init_buffer(struct buffer *buffer){
  //If the buffer in the device struct has not yet been defined.
	/*if (!buffer->start) {
    //Allocate memory for the buffer
		buffer->start = kmalloc(DEFAULT_BUFFER, GFP_KERNEL);
	}

  //The buffers were not allocated, release lock and give a no memory error.
  if (!buffer->start) {
    return -ENOMEM;
  }
  //Set buffersize in device struct.
	buffer->buffersize = DEFAULT_BUFFER;

  //End of the buffer is the start address plus the size.
	buffer->end = buffer->start + buffer->buffersize;

  //Read and write pointers should start at the beginning of file (device)
	buffer->rp = buffer->wp = buffer->start;
*/
  return 0;
}


module_init( dm510_init_module );
module_exit( dm510_cleanup_module );

MODULE_AUTHOR( "Your names here!" );
MODULE_LICENSE( "GPL" );
