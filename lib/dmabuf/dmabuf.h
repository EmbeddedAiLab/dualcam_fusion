/*
 * dmabuf.h  -  one DMA-BUF buffer allocated from a Linux DMA heap.
 *
 * A DMA-BUF can be shared between hardware blocks without copying:
 *   camera driver (V4L2)  ->  RGA  ->  CPU (via mmap)
 */
#ifndef DMABUF_H
#define DMABUF_H

#include <cstddef>
#include <cstdint>

class DmaBuffer
{
public:
    DmaBuffer() = default;
    ~DmaBuffer() { release(); }

    /* Not copyable: the object owns a file descriptor and a mapping */
    DmaBuffer(const DmaBuffer &) = delete;
    DmaBuffer &operator=(const DmaBuffer &) = delete;

    /* Allocate `size` bytes from a heap such as "/dev/dma_heap/system".
     * Returns false (and prints the reason) on failure.               */
    bool allocate(const char *heapPath, size_t size);

    /* Unmap and close. Safe to call more than once. */
    void release();

    /* CPU cache synchronisation. Call Begin before the CPU touches the
     * memory after a device (camera/RGA) wrote it, and End when done.
     * `write` = true if the CPU also writes.                          */
    void cpuAccessBegin(bool write);
    void cpuAccessEnd(bool write);

    int      fd() const   { return fd_; }
    uint8_t *data() const { return static_cast<uint8_t *>(vaddr_); }
    size_t   size() const { return size_; }

private:
    int    fd_    = -1;
    void  *vaddr_ = nullptr;
    size_t size_  = 0;
};

#endif /* DMABUF_H */
