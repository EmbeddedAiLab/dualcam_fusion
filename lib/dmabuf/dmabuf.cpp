#include "dmabuf/dmabuf.h"

#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

bool DmaBuffer::allocate(const char *heapPath, size_t size)
{
    release();

    if (size == 0)
        return false;

    /* 1. Open the heap device */
    int heapFd = ::open(heapPath, O_RDWR | O_CLOEXEC);
    if (heapFd < 0)
    {
        std::perror(heapPath);
        return false;
    }

    /* 2. Ask the heap for a buffer. The kernel returns a new fd that
     *    represents the DMA-BUF.                                      */
    struct dma_heap_allocation_data req;
    std::memset(&req, 0, sizeof(req));
    req.len      = size;
    req.fd_flags = O_RDWR | O_CLOEXEC;

    int rc = ::ioctl(heapFd, DMA_HEAP_IOCTL_ALLOC, &req);
    ::close(heapFd);                       /* heap fd no longer needed */

    if (rc < 0)
    {
        std::perror("DMA_HEAP_IOCTL_ALLOC");
        return false;
    }

    fd_ = static_cast<int>(req.fd);

    /* 3. Map it so the CPU can read/write it */
    void *p = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED)
    {
        std::perror("mmap dmabuf");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    vaddr_ = p;
    size_  = size;
    return true;
}

void DmaBuffer::release()
{
    if (vaddr_)
    {
        ::munmap(vaddr_, size_);
        vaddr_ = nullptr;
    }
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
    size_ = 0;
}

/* Helper that issues DMA_BUF_IOCTL_SYNC. Errors are ignored on purpose:
 * some heaps (e.g. uncached/reserved) do not need or support it.      */
static void syncBuffer(int fd, uint64_t flags)
{
    if (fd < 0)
        return;
    struct dma_buf_sync s;
    s.flags = flags;
    ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

void DmaBuffer::cpuAccessBegin(bool write)
{
    syncBuffer(fd_, DMA_BUF_SYNC_START | (write ? DMA_BUF_SYNC_RW : DMA_BUF_SYNC_READ));
}

void DmaBuffer::cpuAccessEnd(bool write)
{
    syncBuffer(fd_, DMA_BUF_SYNC_END | (write ? DMA_BUF_SYNC_RW : DMA_BUF_SYNC_READ));
}
