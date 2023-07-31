/* adapted from libdragon */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "private.h"

/*** General ***/
#define FALLTHROUGH __attribute__((fallthrough))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define ALIGN8(val) (((val) + 7) & ~7)

typedef signed long ssize_t;

/*** Ring Buffer ***/
#define RING_BUFFER_SIZE (16 * 1024)

typedef struct decoder_ringbuf {
	uint8_t ringbuf[RING_BUFFER_SIZE];    ///< The ring buffer itself
	unsigned int ringbuf_pos;                   ///< Current write position in the ring buffer
} decoder_ringbuf;

void __ringbuf_init(decoder_ringbuf* ringbuf) {
    ringbuf->ringbuf_pos = 0;
}

inline void __ringbuf_writebyte(decoder_ringbuf *ringbuf, uint8_t byte) {
    ringbuf->ringbuf[ringbuf->ringbuf_pos++] = byte;
    ringbuf->ringbuf_pos &= (RING_BUFFER_SIZE - 1);
}

void __ringbuf_write(decoder_ringbuf* ringbuf, uint8_t* src, int count) {
    while (count > 0) {
        int n = MIN(count, (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos));
        memcpy(ringbuf->ringbuf + ringbuf->ringbuf_pos, src, n);
        ringbuf->ringbuf_pos += n;
        ringbuf->ringbuf_pos &= RING_BUFFER_SIZE - 1;
        src += n;
        count -= n;
    }
}

void __ringbuf_copy(decoder_ringbuf* ringbuf, int copy_offset, uint8_t* dst, int count) {
    int ringbuf_copy_pos = (ringbuf->ringbuf_pos - copy_offset) & (RING_BUFFER_SIZE - 1);
    int dst_pos = 0;

    while (count > 0) {
		int wn = count;
        wn = wn < (int)(RING_BUFFER_SIZE - ringbuf_copy_pos)     ? wn : (int)(RING_BUFFER_SIZE - ringbuf_copy_pos);
		wn = wn < (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos) ? wn : (int)(RING_BUFFER_SIZE - ringbuf->ringbuf_pos);
        count -= wn;

        // Check if there's an overlap in the ring buffer between lz4_read and write pos, in which
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
            uint8_t value = ringbuf->ringbuf[ringbuf_copy_pos];
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
#define LZ4_EOF 0
#define HEADER_SIZE 8

typedef struct decoder_faststate {
    uint8_t token;
    int litLen;
    int matchLen;
    int matchOffset;
    int state;
} decoder_faststate;

typedef struct decoder {
    uint8_t buf[LZ4_BUFFER_SIZE] __attribute__((aligned(8)));
    uint8_t* src;
    uint8_t* srcEnd;
    int bufIndex;
    int bufSize;
    bool isEOF;
    decoder_faststate fastState;
    decoder_ringbuf ringbuf;
} decoder;

enum decState {
    STATE_READ_TOKEN,
    STATE_LITERALS,
    STATE_MATCH,
    STATE_MAX
};

size_t lz4_fread(decoder* lz4) {
    size_t i;

    if (lz4->src == lz4->srcEnd)
        return LZ4_EOF;

    i = (lz4->src + LZ4_BUFFER_SIZE) > lz4->srcEnd ? (lz4->srcEnd - lz4->src) : LZ4_BUFFER_SIZE;
    DMARomToRam(lz4->src, lz4->buf, i);
    lz4->src += i;
    assert(lz4->src <= lz4->srcEnd);
    return i;
}

static void lz4_refill(decoder* lz4) {
    lz4->bufSize = lz4_fread(lz4);
    lz4->bufIndex = 0;
    lz4->isEOF = (lz4->bufSize == LZ4_EOF);
}

static uint8_t lz4_readbyte(decoder* lz4) {
    if (lz4->bufIndex >= lz4->bufSize)
        lz4_refill(lz4);
    return lz4->buf[lz4->bufIndex++];
}

static void lz4_read(decoder* lz4, void* buf, size_t len) {
    while (!lz4->isEOF && len > 0) {
        int n = MIN(len, (size_t)(lz4->bufSize - lz4->bufIndex));
        memcpy(buf, lz4->buf + lz4->bufIndex, n);
        buf += n;
        len -= n;
        lz4->bufIndex += n;
        if (lz4->bufIndex >= lz4->bufSize)
            lz4_refill(lz4);
    }
}

void lz4_init(decoder* lz4, uint8_t* src, size_t srcSize) {
    lz4->src = src;
    lz4->srcEnd = src + srcSize;
    lz4->isEOF = false;
    lz4->bufIndex = 0;
    lz4->bufSize = 0;

    memset(&lz4->fastState, 0, sizeof(lz4->fastState));
    __ringbuf_init(&lz4->ringbuf);
}

ssize_t lz4_decompress(decoder* lz4, void* buf, size_t len) {
    decoder_faststate st = lz4->fastState;
    void* buf_orig = buf;
    int n;

	while (!lz4->isEOF && len > 0) {
        switch (st.state) {
            case STATE_READ_TOKEN:
                st.token = lz4_readbyte(lz4);
                st.litLen = ((st.token & 0xF0) >> 4);
                if(unlikely(st.litLen == LZ4_LITERALS_RUN_LEN)) {
                    uint8_t byte;
                    do {
                        byte = lz4_readbyte(lz4);
                        st.litLen += byte;
                    } while (unlikely(byte == 255) && !lz4->isEOF);
                }
                st.state = STATE_LITERALS;
                FALLTHROUGH;
            case STATE_LITERALS:
                n = MIN(st.litLen, (int)len);
                lz4_read(lz4, buf, n);
                __ringbuf_write(&lz4->ringbuf, buf, n);
                buf += n;
                len -= n;
                st.litLen -= n;
                if (st.litLen) {
                    break;
                }
                st.matchOffset = lz4_readbyte(lz4);
                st.matchOffset |= ((uint16_t)lz4_readbyte(lz4)) << 8;
                st.matchLen = (st.token & 0x0F);
                if(unlikely(st.matchLen == LZ4_MATCH_RUN_LEN)) {
                    uint8_t byte;
                    do {
                        byte = lz4_readbyte(lz4);
                        st.matchLen += byte;
                    } while (unlikely(byte == 255) && !lz4->isEOF);
                }
                st.matchLen += LZ4_MIN_MATCH_SIZE;
                st.state = STATE_MATCH;
                FALLTHROUGH;
            case STATE_MATCH:
                n = MIN(st.matchLen, (int)len);
                __ringbuf_copy(&lz4->ringbuf, st.matchOffset, buf, n);
                buf += n;
                len -= n;
                st.matchLen -= n;
                if (st.matchLen) {
                    break;
                }
                st.state = STATE_READ_TOKEN;
        }
    }

    lz4->fastState = st;
    return buf - buf_orig;
}

/*** Main ***/
static decoder lz4;

unsigned int getSizeFromHeader(uint8_t* sizePtr) {
    return (sizePtr[0] << 24) | (sizePtr[1] << 16) | (sizePtr[2] << 8) | (sizePtr[3] << 0);
}

size_t lz4dec(void *src, void *dst, size_t sz) {
    uint8_t* _src = src;
    size_t decSize = getSizeFromHeader(_src + 4);

    _src += HEADER_SIZE;
    sz -= HEADER_SIZE;

    lz4_init(&lz4, _src, sz);
    ssize_t result = lz4_decompress(&lz4, dst, sz);
    assert(result == (ssize_t)sz);

    return decSize;
}
