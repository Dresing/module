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
#include <linux/sched.h>
/* #include <asm/system.h> */
#include <asm/switch_to.h>
#ifndef DEFAULT_BUFFER
  #define DEFAULT_BUFFER 1
#endif
#define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

struct buffer {
    wait_queue_head_t inq, outq;       /* read and write queues */
    char *start, *end;                /* begin of buf, end of buf */
    int buffersize;                    /* used in pointer arithmetic */
    char *rp, *wp;                     /* where to read, where to write */
    int nreaders, nwriters;
    struct semaphore sem;
};

struct device {
    int index;
    struct buffer *readBuffer;
    struct buffer *writeBuffer;
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
static int setBufferSize(struct buffer *buffer, int size);

#define DEVICE_NAME "dm510_dev" /* Dev name as it appears in /proc/devices */
#define MAJOR_NUMBER 254
#define MIN_MINOR_NUMBER 0
#define MAX_MINOR_NUMBER 1
#define MAX_READERS 5
#define MAX_WRITERS 1

#define DEVICE_COUNT 2

#define DM510_IOC_MAGIC  'q'
#define DM510_SET_BUFFER _IOWR(DM510_IOC_MAGIC,   0, int)
#define SCULL_P_IOCQSIZE _IOWR(DM510_IOC_MAGIC,   1, int)
/* ... more to come */

#define DN510_IOC_MAXNR 1
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

static struct buffer *zeroBuffer;
static struct buffer *firstBuffer;


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
		return -ENOMEM;
	}

  //Alocate buffers
  zeroBuffer = kmalloc(sizeof(struct buffer), GFP_KERNEL);
  firstBuffer = kmalloc(sizeof(struct buffer), GFP_KERNEL);

  //The buffers were not allocated, release lock and give a no memory error.
  if (!zeroBuffer) {
    unregister_chrdev_region(deviceNums, DEVICE_COUNT);
    printk(KERN_INFO "DM510: Register error. \n");
    return -ENOMEM;
  }
  if (!firstBuffer) {
    unregister_chrdev_region(deviceNums, DEVICE_COUNT);
    printk(KERN_INFO "DM510: Register error. \n");
    return -ENOMEM;
  }

  zeroBuffer->start = kmalloc(DEFAULT_BUFFER, GFP_KERNEL);
  if (!zeroBuffer->start) {
    unregister_chrdev_region(deviceNums, DEVICE_COUNT);
    printk(KERN_INFO "DM510: Register error. \n");
    return -ENOMEM;
  }

  firstBuffer->start = kmalloc(DEFAULT_BUFFER, GFP_KERNEL);
  if (!firstBuffer->start) {
    unregister_chrdev_region(deviceNums, DEVICE_COUNT);
    printk(KERN_INFO "DM510: Register error. \n");
    return -ENOMEM;
  }


  //Set buffersize in buffer struct.
  zeroBuffer->buffersize = DEFAULT_BUFFER;
  firstBuffer->buffersize = DEFAULT_BUFFER;

  //End of the buffer is the start address plus the size.
  zeroBuffer->end = zeroBuffer->start + zeroBuffer->buffersize;
  firstBuffer->end = firstBuffer->start + firstBuffer->buffersize;

  //Read and write pointers should start at the beginning of file (device)
  zeroBuffer->rp = zeroBuffer->wp = zeroBuffer->start;
  firstBuffer->rp = firstBuffer->wp = firstBuffer->start;

  //Init inumeration of reader/writers
  zeroBuffer->nwriters = zeroBuffer->nreaders = 0;
  firstBuffer->nwriters = firstBuffer->nreaders = 0;

  //Init queues
	init_waitqueue_head(&(zeroBuffer->inq));
	init_waitqueue_head(&(zeroBuffer->outq));
  init_waitqueue_head(&(firstBuffer->inq));
	init_waitqueue_head(&(firstBuffer->outq));

  init_MUTEX(&zeroBuffer->sem);
  init_MUTEX(&firstBuffer->sem);


  //Iterating variable necesary as of c90 standard
	int i;

	//Ensure everything is emptied at the memory location to avoid junk errors.
	memset(devices, 0, DEVICE_COUNT * sizeof(struct device));

  //Initialize all the devices.
	for (i = 0; i < DEVICE_COUNT; i++) {
		dm510_init_device(&devices[i], i);
	}


  //Debugging message


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

	}

  //Free up the memory address for the devices/buffers and unregister the driver regions.
	kfree(devices);
  kfree(zeroBuffer->start);
  kfree(zeroBuffer);
  kfree(firstBuffer->start);
  kfree(firstBuffer);
  unregister_chrdev_region(deviceNums, DEVICE_COUNT);

  //Ensure that the device pointer is indeed null.
	devices = NULL;

  //Debug message



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

  //Indentify the current device read/write buffer depending on its index.
  if(dev->index == 0){
    dev->readBuffer = zeroBuffer;
    dev->writeBuffer = firstBuffer;
  }
  else{
    dev->readBuffer = firstBuffer;
    dev->writeBuffer = zeroBuffer;
  }

  up(&dev->sem);


  //Increment number of readers/writers accordingly to mode and release the lock
	if (filp->f_mode & FMODE_READ){

    down(&dev->readBuffer->sem);

    if(dev->readBuffer->nreaders == MAX_READERS){
      up(&dev->readBuffer->sem);
      return -EBUSY;
    }

    dev->readBuffer->nreaders++;

    up(&dev->readBuffer->sem);

  }

	if (filp->f_mode & FMODE_WRITE){

    down(&dev->writeBuffer->sem);

    if(dev->writeBuffer->nwriters == MAX_WRITERS){

      up(&dev->writeBuffer->sem);

      //If read access was already granted, revert it.
      if(filp->f_mode & FMODE_READ){
        down(&dev->readBuffer->sem);
          dev->readBuffer->nreaders--;
        up(&dev->readBuffer->sem);
      }

      return -EBUSY;
    }

    dev->writeBuffer->nwriters++;

    up(&dev->writeBuffer->sem);

  }

  //Disallow llseek
	return nonseekable_open(inode, filp);

}


/* Called when a process closes the device file. */
static int dm510_release( struct inode *inode, struct file *filp ) {

  //Get the device structure in the file-pointer's private data.
  struct device *dev = filp->private_data;

  if (filp->f_mode & FMODE_READ){

    down(&dev->readBuffer->sem);
    dev->readBuffer->nreaders--;
    up(&dev->readBuffer->sem);
  }

  if (filp->f_mode & FMODE_WRITE){

    down(&dev->writeBuffer->sem);
    dev->writeBuffer->nwriters--;
    up(&dev->writeBuffer->sem);

  }

  //Successful execution
	return 0;
}


/* Called when a process, which already opened the dev file, attempts to read from it. */
static ssize_t dm510_read( struct file *filp,
    char *buf,      /* The buffer to fill with data     */
    size_t count,   /* The max number of bytes to read  */
    loff_t *f_pos )  /* The offset in the file           */
{

  struct device *dev = filp->private_data;

  if (down_interruptible(&dev->readBuffer->sem)){
    return -ERESTARTSYS;
  }

  while(dev->readBuffer->rp == dev->readBuffer->wp){
    up(&dev->readBuffer->sem);

    wake_up_interruptible(&dev->readBuffer->inq);

    if(wait_event_interruptible(dev->readBuffer->outq, (dev->readBuffer->rp != dev->readBuffer->wp))){
      return -ERESTARTSYS;
    }

    if (down_interruptible(&dev->readBuffer->sem)){
      return -ERESTARTSYS;
    }
  }

  count = min(count, (size_t)(dev->readBuffer->wp - dev->readBuffer->rp));


	if (copy_to_user(buf, dev->readBuffer->rp, count)) {
    up(&dev->readBuffer->sem);
		return -EFAULT;
	}

	dev->readBuffer->rp += count;

  up(&dev->readBuffer->sem);

  wake_up_interruptible(&dev->readBuffer->inq);

	return count; //return number of bytes read
}


/* Called when a process writes to dev file. f_post: The offset in the file */
static ssize_t dm510_write( struct file *filp, const char *buf, size_t count, loff_t *f_pos ) {

	/* write code belongs here */
  struct device *dev = filp->private_data;

  if (down_interruptible(&dev->writeBuffer->sem)){
    return -ERESTARTSYS;
  }

  while(dev->writeBuffer->wp == dev->writeBuffer->end){
    if(dev->writeBuffer->rp != dev->writeBuffer->end){
      up(&dev->writeBuffer->sem);
      if(wait_event_interruptible(dev->writeBuffer->inq, (dev->writeBuffer->rp == dev->writeBuffer->end))){
        return -ERESTARTSYS;
      }
      if (down_interruptible(&dev->writeBuffer->sem)){
        return -ERESTARTSYS;
      }
    }
    else{
      dev->writeBuffer->wp = dev->writeBuffer->start;
      dev->writeBuffer->rp = dev->writeBuffer->start;
    }
  }

	count = min(count, (size_t)(dev->writeBuffer->end - dev->writeBuffer->wp));

  if (copy_from_user(dev->writeBuffer->wp, buf, count)) {
    up (&dev->writeBuffer->sem);
    return -EFAULT;
  }

  dev->writeBuffer->wp += count;

  up(&dev->writeBuffer->sem);

  wake_up_interruptible(&dev->writeBuffer->outq);

	return count;
}

/* called by system call icotl */
long dm510_ioctl(struct file *filp, unsigned int cmd,  unsigned long arg ){

  if (_IOC_TYPE(cmd) != DM510_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > DN510_IOC_MAXNR) return -ENOTTY;

  int err;

  if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;


  struct device *dev = filp->private_data;


  switch(cmd) {
    case DM510_SET_BUFFER:
      return 0;
		break;

	  case SCULL_P_IOCQSIZE:
		  return 0;
    break;


	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	printk(KERN_INFO "DM510: ioctl called.\n");

  //Executed successfully.
	return 0;
}

static int isError(status){
		return (status < 0);
}

int dm510_init_device(struct device *device, int index){


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




	return 0;
}

static int setBufferSize(struct buffer *buffer, int size){


  struct buffer *holder = kmalloc(size, GFP_KERNEL);

  if (!holder) {
    return -ENOMEM;
  }

  //Copy content from old buffer into the new one.
  down(&buffer->sem);

  //New buffer must be bigger.
  if(size <= buffer->end - buffer->start){
    kfree(holder);
    up(&buffer->sem);
    return -EINVAL;
  }

  buffer->start = (struct buffer*) memcpy(holder, buffer->start, (size_t) (buffer->end - buffer->start));


  up(&buffer->sem);

  return 0;
}

module_init( dm510_init_module );
module_exit( dm510_cleanup_module );

MODULE_AUTHOR( "Your names here!" );
MODULE_LICENSE( "GPL" );
