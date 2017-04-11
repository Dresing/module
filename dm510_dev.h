/* Prototype module for third mandatory DM510 assignment */
#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif
#ifndef __DM510__
  /**
  * Includes
  */
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
  #include <asm/switch_to.h>

  /**
  * Structs
  */
  struct buffer {
      wait_queue_head_t inq, outq, readq;       /* read and write queues */
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


  /* Prototypes - this would normally go in a .h file */
  static int dm510_open( struct inode*, struct file* );
  static int dm510_release( struct inode*, struct file* );
  static ssize_t dm510_read( struct file*, char*, size_t, loff_t* );
  static ssize_t dm510_write( struct file*, const char*, size_t, loff_t* );
  long dm510_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
  int dm510_init_device(struct device*, int index);
  int dm510_init_buffer(struct buffer*);
  static int setBufferSize(struct buffer *buffer, int size);
  static int setMaxReaders(struct buffer *buffer, int num);


  /**
  * Global constants
  */
  #define DEVICE_NAME "dm510_dev" // Dev name as it appears in /proc/devices
  #define MAJOR_NUMBER 254  //Major number for the driver
  #define MIN_MINOR_NUMBER 0  //Minor number to indentify devices.
  #define MAX_MINOR_NUMBER 1  //Minor number to indentify devices.
  #define MAX_READERS_DEFAULT 5       //Default max number of readers
  #define MAX_WRITERS_DEFAULT 1       //Default max number of writers (Should not be altered)
  #define DEVICE_COUNT 2      //How many devices the driver can register
  #define DM510_IOC_MAGIC  'q'  //Magic number to uniqueiy ioc
  #define DEFAULT_BUFFER 20

  /**
  * Macros
  */
  #define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

  /**
  * All IOC calls and their
  */

  //Defines a buffersize, the buffer can only grow, not shrink.
  #define DM510_SET_WRITE_BUFFER _IOWR(DM510_IOC_MAGIC,   0, int)
  #define DM510_SET_READ_BUFFER _IOWR(DM510_IOC_MAGIC,   1, int)
  #define DM510_SET_MAX_READERS _IOWR(DM510_IOC_MAGIC,   2, int)
  #define DN510_IOC_MAXNR 3



  /**
  * Supported fileops
  */
  static struct file_operations dm510_fops = {
  	.owner   = THIS_MODULE,
  	.read    = dm510_read,
  	.write   = dm510_write,
  	.open    = dm510_open,
  	.release = dm510_release,
    .unlocked_ioctl   = dm510_ioctl
  };

  /**
  * Macros
  */
  dev_t deviceNums = MKDEV(MAJOR_NUMBER, MIN_MINOR_NUMBER);

  /**
  * Global variables
  */
  static struct device *devices;
  static struct buffer *zeroBuffer;
  static struct buffer *firstBuffer;
  int MAX_WRITERS = MAX_WRITERS_DEFAULT;
  int MAX_READERS = MAX_READERS_DEFAULT;

#endif
