/* Test program for 3rd mandatory assignment.
 *
 * A process writes ITS integers to /dev/dm510-0 while
 * another process read ITS integers from /dev/dm510-1.
 * A checksum of the written data is compared with a
 * checksum of the read data.
 *
 * This is done in both directions.
 */

 #include <stdio.h>
 #include <unistd.h>
 #include <sys/types.h>
 #include <stdlib.h>
 #include <fcntl.h>
 #include <errno.h>
 #include <linux/ioctl.h>
 #include <signal.h>

#define DM510_IOC_MAGIC  'q'
#define DM510_SET_WRITE_BUFFER _IOWR(DM510_IOC_MAGIC,   0, int)
#define DM510_SET_READ_BUFFER _IOWR(DM510_IOC_MAGIC,   1, int)
#define DM510_SET_MAX_READERS _IOWR(DM510_IOC_MAGIC,   2, int)
#define DN510_IOC_MAXNR 2


int open_readers(){
  pid_t pid;
  int fd, fd2;
  int sum = 0, i;
  int val;
  int cnt;

  int score = 0;

  pid = fork();

  if (pid == 0) {
      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDWR);
      perror("w/r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

  } else {
      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      ioctl(fd, DM510_SET_MAX_READERS, 20);
      close(fd);
      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDWR);
      perror("w/r open dm510-1");

      fd2 = open("/dev/dm510-1", O_RDWR);
      perror("w/r open dm510-1");

      close(fd);

      fd = open("/dev/dm510-1", O_RDONLY);
      perror("r open dm510-1");

      close(fd);

      wait(NULL);

  }

  return score;
}
int main(int argc, char *argv[])
{
  printf("\n ");
  printf("-- Opening files (Max 20, blocking -> faster)  -- \n ");
    sleep(1);
    open_readers();
    sleep(1);


}
