/* adapted from libdragon */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "private.h"

typedef signed long ssize_t;

/*** LZ4 Decoder ***/
#define LZ4_BUFFER_SIZE 128
#define LZ4_LITERALS_RUN_LEN 15
#define LZ4_MATCH_RUN_LEN 15
#define LZ4_MIN_MATCH_SIZE 4
#define LZ4_EOF (-1)
#define HEADER_SIZE 8
#define DECOMPRESS_LZ4_STATE_SIZE (16552)

unsigned int getSizeFromHeader(uint8_t* sizePtr) {
    return (sizePtr[0] << 24) | (sizePtr[1] << 16) | (sizePtr[2] << 8) | (sizePtr[3] << 0);
}

typedef struct FILE {
    uint8_t* start;
    uint8_t* end;
    size_t size;
    size_t decSize;
    size_t len;
} FILE;

typedef struct lz4dec_faststate_s {
   uint8_t token;       ///< Current token
   int lit_len;         ///< Number of literals to copy
   int match_len;       ///< Number of bytes to copy from the ring buffer
   int match_off;       ///< Offset in the ring buffer to copy from
   int fsm_state;       ///< Current state of the streaming state machine
} lz4dec_faststate_t;

typedef struct lz4dec_state_s {
   uint8_t buf[128] __attribute__((aligned(8)));     ///< File buffer
   FILE *fp;                        ///< File pointer to read from
	int buf_idx;                     ///< Current index in the file buffer
	int buf_size;                    ///< Size of the file buffer
   bool eof;                        ///< True if we reached the end of the file
   lz4dec_faststate_t st;           ///< Fast-access state
   decompress_ringbuf_t ringbuf;    ///< Ring buffer
} lz4dec_state_t;

static lz4dec_state_t lz4;

int getc(FILE* restrict fp) {
    if (fp->len >= fp->size)
        return LZ4_EOF;
    return fp->start[fp->len++];
}

size_t lz4_fread(void* restrict dst, size_t size, size_t count, FILE* restrict fp) {
    // https://stackoverflow.com/a/8590471
    register char *cp = dst;
    register int c;
    size_t read = 0;
    register size_t s;

    if (size)
        while (read < count) {
            s = size;
            do {
                if ((c = getc(fp)) != LZ4_EOF)
                    *cp++ = c;
                else
                    return read;
            } while (--s);
            read++;
        }

    return read;
}

static void lz4_refill(lz4dec_state_t *lz4)
{
   lz4->buf_size = lz4_fread(lz4->buf, 1, sizeof(lz4->buf), lz4->fp);
   lz4->buf_idx = 0;
   lz4->eof = (lz4->buf_size == 0);
}

static uint8_t lz4_readbyte(lz4dec_state_t *lz4)
{
   if (lz4->buf_idx >= lz4->buf_size)
      lz4_refill(lz4);
   return lz4->buf[lz4->buf_idx++];
}

static void lz4_read(lz4dec_state_t *lz4, void *buf, size_t len)
{
   while (len > 0) {
      int n = MIN(len, lz4->buf_size - lz4->buf_idx);
      memcpy(buf, lz4->buf + lz4->buf_idx, n);
      buf += n;
      len -= n;
      lz4->buf_idx += n;
      if (lz4->buf_idx >= lz4->buf_size)
         lz4_refill(lz4);
   }
}

void decompress_lz4_init(void *state, FILE *fp)
{
   lz4dec_state_t *lz4 = (lz4dec_state_t*)state;
   lz4->fp = fp;
   lz4->eof = false;
   lz4->buf_idx = 0;
   lz4->buf_size = 0;
   memset(&lz4->st, 0, sizeof(lz4->st));
   __ringbuf_init(&lz4->ringbuf);
}

ssize_t decompress_lz4_read(void *state, void *buf, size_t len)
{
   lz4dec_state_t *lz4 = (lz4dec_state_t*)state;
   lz4dec_faststate_t st = lz4->st;
   void *buf_orig = buf;
   int n;

   while (!lz4->eof && len > 0) {
      switch (st.fsm_state) {
      case 0: // read token
         st.token = lz4_readbyte(lz4);
         st.lit_len = ((st.token & 0xf0) >> 4);
         if (unlikely(st.lit_len == LZ4_LITERALS_RUN_LEN)) {
            uint8_t byte;
            do {
               byte = lz4_readbyte(lz4);
               st.lit_len += byte;
            } while (unlikely(byte == 255));
         }
         st.fsm_state = 1;
      case 1: // literals
         n = MIN(st.lit_len, len);
         lz4_read(lz4, buf, n);
         __ringbuf_write(&lz4->ringbuf, buf, n);
         buf += n;
         len -= n;
         st.lit_len -= n;
         if (st.lit_len)
            break;
         st.match_off = lz4_readbyte(lz4);
         st.match_off |= ((uint16_t)lz4_readbyte(lz4)) << 8;
         st.match_len = (st.token & 0x0f);
         if (unlikely(st.match_len == LZ4_MATCH_RUN_LEN)) {
            uint8_t byte;
            do {
               byte = lz4_readbyte(lz4);
               st.match_len += byte;
            } while (unlikely(byte == 255));
         }
         st.match_len += LZ4_MIN_MATCH_SIZE;
         st.fsm_state = 2;
      case 2: // match
         n = MIN(st.match_len, len);
         __ringbuf_copy(&lz4->ringbuf, st.match_off, buf, n);
         buf += n;
         len -= n;
         st.match_len -= n;
         if (st.match_len)
            break;
         st.fsm_state = 0;
      }
   }

   lz4->st = st;
   return buf - buf_orig;
}

size_t lz4dec(void *src, void *dst, size_t sz) {
    uint8_t* _src = src;
    FILE file;
    file.decSize = getSizeFromHeader(_src + 4);

    _src += HEADER_SIZE;
    sz -= HEADER_SIZE;

    file.start = _src;
    file.size = sz;
    file.end = file.start + file.size;
    file.len = 0;

    decompress_lz4_init(&lz4, &file);
    ssize_t result = decompress_lz4_read(&lz4, dst, file.decSize);
    assert(result == (ssize_t)file.decSize);

    return file.decSize;
}
