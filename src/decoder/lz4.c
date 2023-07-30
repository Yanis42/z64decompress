/* adapted from libdragon */

#include <assert.h>
#include <stdbool.h>

#include "private.h"

/*** General ***/
#define FALLTHROUGH __attribute__((fallthrough))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define likely(x) (x)
#define unlikely(x) (x)

typedef unsigned long long uint64_t;
typedef signed long ssize_t;

/*** Ring Buffer ***/
#define RING_BUFFER_SIZE (16 * 1024)

typedef struct decoder_ringbuf {
	unsigned char ringbuf[RING_BUFFER_SIZE];    ///< The ring buffer itself
	unsigned int ringbuf_pos;                   ///< Current write position in the ring buffer
} decoder_ringbuf;

void __ringbuf_init(decoder_ringbuf* ringbuf) {
    ringbuf->ringbuf_pos = 0;
}

void __ringbuf_write(decoder_ringbuf* ringbuf, unsigned char* src, int count) {
    while (count > 0) {
        int n = MIN(count, (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos));
        memcpy(ringbuf->ringbuf + ringbuf->ringbuf_pos, src, n);
        ringbuf->ringbuf_pos += n;
        ringbuf->ringbuf_pos &= RING_BUFFER_SIZE - 1;
        src += n;
        count -= n;
    }
}

void __ringbuf_copy(decoder_ringbuf* ringbuf, int copy_offset, unsigned char* dst, int count) {
    int ringbuf_copy_pos = (ringbuf->ringbuf_pos - copy_offset) & (RING_BUFFER_SIZE-1);
    int dst_pos = 0;

    while (count > 0) {
		int wn = count;
        wn = wn < (int)(RING_BUFFER_SIZE - ringbuf_copy_pos)     ? wn : (int)(RING_BUFFER_SIZE - ringbuf_copy_pos);
		wn = wn < (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos) ? wn : (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos);
        count -= wn;

        // Check if there's an overlap in the ring buffer between read and write pos, in which
        // case we need to copy byte by byte.
        if ((int)ringbuf->ringbuf_pos < ringbuf_copy_pos || 
            (int)ringbuf->ringbuf_pos > ringbuf_copy_pos+7) {
            while (wn >= 8) {
                // Copy 8 bytes at at time, using a unaligned memory access (LDL/LDR/SDL/SDR)
                typedef uint64_t u_uint64_t __attribute__((aligned(1)));
                uint64_t value = *(u_uint64_t*)&ringbuf->ringbuf[ringbuf_copy_pos];
                *(u_uint64_t*)&dst[dst_pos] = value;
                *(u_uint64_t*)&ringbuf->ringbuf[ringbuf->ringbuf_pos] = value;

                ringbuf_copy_pos += 8;
                ringbuf->ringbuf_pos += 8;
                dst_pos += 8;
                wn -= 8;
            }
        }

        // Finish copying the remaining bytes
        while (wn > 0) {
            unsigned char value = ringbuf->ringbuf[ringbuf_copy_pos];
            dst[dst_pos] = value;
            ringbuf->ringbuf[ringbuf->ringbuf_pos] = value;

            ringbuf_copy_pos += 1;
            ringbuf->ringbuf_pos += 1;
            dst_pos += 1;
            wn -= 1;
        }

        ringbuf_copy_pos %= RING_BUFFER_SIZE;
        ringbuf->ringbuf_pos %= RING_BUFFER_SIZE;
    }
}

/*** LZ4 Decoder ***/
#define LZ4_BUFFER_SIZE 128
#define LZ4_LITERALS_RUN_LEN 15
#define LZ4_MATCH_RUN_LEN 15
#define LZ4_MIN_MATCH_SIZE 4
#define HEADER_SIZE 16

typedef struct decoder {
    unsigned char buf[LZ4_BUFFER_SIZE] __attribute__((aligned(8)));
    unsigned char* src;
    size_t bufIndex;
    size_t bufSize;
    bool isEOF;
    size_t srcSize;
    size_t readBytes;

    unsigned char token;
    unsigned char state;
    int litLen;
    int matchLen;
    int matchOffset;

    decoder_ringbuf ringbuf;
} decoder;

enum decState {
    STATE_READ_TOKEN,
    STATE_LITERALS,
    STATE_MATCH,
    STATE_MAX
};

size_t romRead(unsigned char* buf, size_t size, unsigned char* src) {
    size_t readBytes = size;
    DMARomToRam(src, buf, size);
    return readBytes;
}

static void refill(decoder* lz4) {
    lz4->readBytes = romRead(lz4->buf, sizeof(lz4->buf), lz4->src);
    lz4->src += lz4->readBytes;
    lz4->bufSize = lz4->readBytes;
    lz4->bufIndex = 0;
    lz4->isEOF = (lz4->readBytes > lz4->srcSize);
}

static unsigned char readbyte(decoder* lz4) {
    if (lz4->bufIndex >= lz4->bufSize) {
        refill(lz4);
    }
    return lz4->buf[lz4->bufIndex++];
}

static void read(decoder* lz4, void* buf, size_t len) {
    while (len > 0) {
        int n = MIN(len, lz4->bufSize - lz4->bufIndex);
        memcpy(buf, lz4->buf + lz4->bufIndex, n);
        buf += n;
        len -= n;
        lz4->bufIndex += n;
        if (lz4->bufIndex >= lz4->bufSize) {
            refill(lz4);
        }
    }
}

void lz4_init(decoder* lz4, unsigned char* src, size_t srcSize) {
    lz4->src = src;
    lz4->isEOF = false;
    lz4->bufIndex = 0;
    lz4->bufSize = 0;
    lz4->srcSize = srcSize;
    lz4->readBytes = 0;

    lz4->token = 0;
    lz4->state = STATE_READ_TOKEN;
    lz4->litLen = 0;
    lz4->matchLen = 0;
    lz4->matchOffset = 0;

    __ringbuf_init(&lz4->ringbuf);
}

ssize_t lz4_read(decoder* lz4, void* buf, size_t len) {
    void* buf_orig = buf;
    int n;

	while (!lz4->isEOF && len > 0) {
        switch (lz4->state) {
            case STATE_READ_TOKEN:
                lz4->token = readbyte(lz4);
                lz4->litLen = ((lz4->token & 0xF0) >> 4);
                if(unlikely(lz4->litLen == LZ4_LITERALS_RUN_LEN)) {
                    unsigned char byte;
                    do {
                        byte = readbyte(lz4);
                        lz4->litLen += byte;
                    } while (unlikely(byte == 255));
                }
                lz4->state = STATE_LITERALS;
                FALLTHROUGH;
            case STATE_LITERALS:
                n = MIN(lz4->litLen, (long)len);
                read(lz4, buf, n);
                __ringbuf_write(&lz4->ringbuf, buf, n);
                buf += n;
                len -= n;
                lz4->litLen -= n;
                if (lz4->litLen) {
                    break;
                }
                lz4->matchOffset = readbyte(lz4);
                lz4->matchOffset |= ((unsigned short)readbyte(lz4) << 8);
                lz4->matchLen = (lz4->token & 0x0F);
                if(unlikely(lz4->matchLen == LZ4_MATCH_RUN_LEN)) {
                    unsigned char byte;
                    do {
                        byte = readbyte(lz4);
                        lz4->matchLen += byte;
                    } while (unlikely(byte == 255));
                }
                lz4->matchLen += LZ4_MIN_MATCH_SIZE;
                lz4->state = STATE_MATCH;
                FALLTHROUGH;
            case STATE_MATCH:
                n = MIN(lz4->matchLen, (long)len);
                __ringbuf_copy(&lz4->ringbuf, lz4->matchOffset, buf, n);
                buf += n;
                len -= n;
                lz4->matchLen -= n;
                if (lz4->matchLen) {
                    break;
                }
                lz4->state = STATE_READ_TOKEN;
        }
    }

    return buf - buf_orig;
}

/*** Main ***/
static decoder lz4;

unsigned int getSizeFromHeader(unsigned char* sizePtr) {
    return (sizePtr[0] << 24) | (sizePtr[1] << 16) | (sizePtr[2] << 8) | (sizePtr[3] << 0);
}

size_t lz4dec(void *src, void *dst, size_t sz) {
    unsigned char* _src = src;
    size_t decSize = getSizeFromHeader(_src + 4);
    lz4_init(&lz4, _src + HEADER_SIZE, sz);
    assert(lz4_read(&lz4, dst, sz) == (ssize_t)sz);
    return decSize;
}
