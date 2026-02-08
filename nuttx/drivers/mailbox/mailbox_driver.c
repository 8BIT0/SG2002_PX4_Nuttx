#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mbox/mbox.h>
#include <nuttx/semaphore.h>

#ifdef CONFIG_MBOX

#define DEVNAME_FMT    "/dev/mailbox"
#define DEVNAME_FMTLEN (strlen(DEVNAME_FMT) + 1)

static int     mboxdrvr_open(FAR struct file *filep);
static int     mboxdrvr_close(FAR struct file *filep);
static ssize_t mboxdrvr_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t mboxdrvr_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     mboxdrvr_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static const struct file_operations mboxdrvr_fops =
{
  mboxdrvr_open,    /* open     */
  mboxdrvr_close,   /* close    */
  mboxdrvr_read,    /* read     */
  mboxdrvr_write,   /* write    */
  NULL,             /* seek     */
  mboxdrvr_ioctl,   /* ioctl    */
  NULL,             /* poll     */
  NULL              /* unlink   */
};

static int mboxdrvr_open(FAR struct file *filep) {
    UNUSED(filep);
    return OK;
}

static int mboxdrvr_close(FAR struct file *filep) {
    UNUSED(filep);
    return OK;
}

static ssize_t mboxdrvr_read(FAR struct file *filep, FAR char *buffer, size_t buflen) {
    return 0;
}

static ssize_t mboxdrvr_write(FAR struct file *filep, FAR const char *buffer, size_t buflen) {
    return buflen;
}

static int mboxdrvr_ioctl(FAR struct file *filep, int cmd, unsigned long arg) {
    FAR struct inode *inode = NULL;
    FAR struct mailbox_driver_s *priv = NULL;
    int ret = -1;

    DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
    inode = filep->f_inode;

    priv = (FAR struct mbox_driver_s *)inode->i_private;
    DEBUGASSERT(priv);
    
    return ret;
}

int mbox_register(FAR struct mbox_dev_s *mbox) {
    FAR struct mbox_dev_s *priv;
    char devname[DEVNAME_FMTLEN];
    int ret;

    /* Allocate a mailbox device structure */

    priv = (FAR struct mbox_dev_s *)kmm_zalloc(sizeof(struct mbox_dev_s));
    if (priv) {
        /* Initialize the mailbox device structure */

        priv = mbox;

        /* Create the character device name */

        memset(devname, '\0', DEVNAME_FMTLEN);
        snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT);
        ret = register_driver(devname, &mboxdrvr_fops, 0666, priv);
        if (ret < 0) {
            /* Free the device structure if we failed to create the character
            * device.
            */

            kmm_free(priv);
            return ret;
        }

        /* Return the result of the registration */

        return OK;
    }

    return -ENOMEM;
}

#endif